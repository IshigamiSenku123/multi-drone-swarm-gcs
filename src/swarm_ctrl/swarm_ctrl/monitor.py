#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import time
import math

class SwarmMonitor(Node):
    def __init__(self):
        super().__init__('swarm_monitor')
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('display_rate_hz', 2.0)
        self.num_drones = self.get_parameter('num_drones').value
        self.display_rate = self.get_parameter('display_rate_hz').value
        self.drone_namespaces = [f'drone{i}' for i in range(self.num_drones)]
        self.drone_poses = {}
        self.last_heartbeat = {}
        self.swarm_status = "Initializing..."
        for ns in self.drone_namespaces:
            self.create_subscription(PoseStamped, f'/{ns}/local_position/pose', lambda msg, namespace=ns: self.pose_callback(msg, namespace), qos_profile=qos_profile_sensor_data)
        self.create_subscription(String, '/swarm/status', self.status_callback, 10)
        self.timer = self.create_timer(1.0 / self.display_rate, self.display)
        self.get_logger().info('Swarm Monitor started')

    def pose_callback(self, msg, namespace):
        self.drone_poses[namespace] = msg
        self.last_heartbeat[namespace] = time.time()

    def status_callback(self, msg):
        self.swarm_status = msg.data

    @property
    def leader_ns(self):
        if "Leader:" in self.swarm_status:
            try: return self.swarm_status.split("Leader:")[1].split("|")[0].strip()
            except Exception: pass
        return 'drone0'

    def display(self):
        now = time.time()
        print("\033[2J\033[H", end="")
        print("=" * 70); print("                    PX4 SWARM MONITOR"); print("=" * 70)
        print(f"Status: {self.swarm_status}")
        print(f"Time:   {time.strftime('%H:%M:%S')}")
        print("-" * 70)
        print(f"{'Drone':<10} {'Online':<8} {'X (m)':<10} {'Y (m)':<10} {'Z (m)':<10} {'Alt (m)':<10} {'Age (ms)':<10}")
        print("-" * 70)
        for ns in self.drone_namespaces:
            if ns in self.drone_poses:
                p = self.drone_poses[ns].pose.position
                age_ms = (now - self.last_heartbeat.get(ns, now)) * 1000
                online = "YES" if age_ms < 2000 else "NO"
                age_str = f"{age_ms:.0f}" if age_ms < 10000 else "STALE"
                alt = -p.z
                print(f"{ns:<10} {online:<8} {p.x:<10.2f} {p.y:<10.2f} {p.z:<10.2f} {alt:<10.2f} {age_str:<10}")
            else:
                print(f"{ns:<10} {'NO':<8} {'---':<10} {'---':<10} {'---':<10} {'---':<10} {'---':<10}")
        print("-" * 70)
        if len(self.drone_poses) >= 2 and self.leader_ns in self.drone_poses:
            p0 = self.drone_poses[self.leader_ns].pose.position
            print("\nFORMATION CHECK:")
            for ns in self.drone_namespaces:
                if ns != self.leader_ns and ns in self.drone_poses:
                    p = self.drone_poses[ns].pose.position
                    dx, dy, dz = p.x - p0.x, p.y - p0.y, p.z - p0.z
                    dist = math.sqrt(dx**2 + dy**2 + dz**2)
                    print(f"  {ns} -> {self.leader_ns}:  dist={dist:.2f}m  offset=({dx:.2f}, {dy:.2f}, {dz:.2f})")
        print("=" * 70); print("Press Ctrl+C to exit")

def main(args=None):
    rclpy.init(args=args)
    node = SwarmMonitor()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
