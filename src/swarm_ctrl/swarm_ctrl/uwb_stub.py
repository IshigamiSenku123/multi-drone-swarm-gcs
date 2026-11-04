#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
import random

class UWBStub(Node):
    def __init__(self):
        super().__init__('uwb_stub')
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('publish_rate_hz', 20.0)
        self.declare_parameter('noise_xy_m', 0.15)
        self.declare_parameter('noise_z_m', 0.10)
        self.declare_parameter('dropout_prob', 0.02)
        self.num_drones = self.get_parameter('num_drones').value
        self.publish_rate = self.get_parameter('publish_rate_hz').value
        self.noise_xy = self.get_parameter('noise_xy_m').value
        self.noise_z = self.get_parameter('noise_z_m').value
        self.dropout_prob = self.get_parameter('dropout_prob').value
        self.drone_namespaces = [f'drone{i}' for i in range(self.num_drones)]
        self.gps_poses = {}
        for ns in self.drone_namespaces:
            self.create_subscription(PoseStamped, f'/{ns}/local_position/pose', lambda msg, namespace=ns: self.gps_callback(msg, namespace), qos_profile=qos_profile_sensor_data)
        self.uwb_pubs = {}
        for ns in self.drone_namespaces:
            self.uwb_pubs[ns] = self.create_publisher(PoseStamped, f'/{ns}/uwb_pose', 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_uwb)
        self.get_logger().info('UWB / SLAM STUB INITIALIZED')

    def gps_callback(self, msg, namespace):
        self.gps_poses[namespace] = msg

    def publish_uwb(self):
        for ns in self.drone_namespaces:
            if ns not in self.gps_poses: continue
            if random.random() < self.dropout_prob: continue
            gps = self.gps_poses[ns]
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = gps.pose.position.x + random.gauss(0, self.noise_xy)
            msg.pose.position.y = gps.pose.position.y + random.gauss(0, self.noise_xy)
            msg.pose.position.z = gps.pose.position.z + random.gauss(0, self.noise_z)
            msg.pose.orientation = gps.pose.orientation
            self.uwb_pubs[ns].publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = UWBStub()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
