#!/usr/bin/env python3
"""
Swarm Controller v3.0 — Merged Fix
Base: User's original Python (rclpy) code
Fixes applied:
  1. ENU frame: target_altitudes positive (z=UP) for MAVROS setpoint_position/local
  2. Equilateral triangle formation: 2m side length in ENU (x=East, y=North)
  3. Dedicated 20Hz setpoint thread: isolated from blocking service calls
  4. arm_confirmed set: stops re-sending ARM once accepted
  5. Non-blocking service_is_ready() check instead of wait_for_service(0.5)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from mavros_msgs.msg import State as MavrosState
from mavros_msgs.srv import CommandBool, SetMode
import time
import csv
import os
import threading
import math
from enum import Enum, auto

class Phase(Enum):
    INIT = auto(); PRE_OFFBOARD = auto(); SET_OFFBOARD = auto()
    ARM = auto(); TAKEOFF = auto(); FORMATION = auto()
    HOLD = auto(); FAULT = auto()

class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')
        self.declare_parameter('num_drones', 3)
        self.declare_parameter('setpoint_rate_hz', 20.0)
        self.declare_parameter('control_rate_hz', 10.0)
        self.declare_parameter('heartbeat_timeout_sec', 2.0)
        self.declare_parameter('pre_offboard_duration_sec', 2.0)
        self.declare_parameter('takeoff_duration_sec', 5.0)
        self.declare_parameter('formation_separation_m', 2.0)
        self.declare_parameter('altitude_stagger_m', 2.0)
        self.declare_parameter('base_altitude_m', 5.0)

        self.num_drones = self.get_parameter('num_drones').value
        self.setpoint_rate = self.get_parameter('setpoint_rate_hz').value
        self.control_rate = self.get_parameter('control_rate_hz').value
        self.heartbeat_timeout = self.get_parameter('heartbeat_timeout_sec').value
        self.pre_offboard_duration = self.get_parameter('pre_offboard_duration_sec').value
        self.takeoff_duration = self.get_parameter('takeoff_duration_sec').value
        self.formation_sep = self.get_parameter('formation_separation_m').value
        self.alt_stagger = self.get_parameter('altitude_stagger_m').value
        self.base_altitude = self.get_parameter('base_altitude_m').value

        self.drone_namespaces = [f'drone{i}' for i in range(self.num_drones)]
        # FIX 2: Equilateral triangle formation in ENU (x=East, y=North)
        self.formation_offsets = {
            'drone0': (0.0, 0.0, 0.0),
            'drone1': (self.formation_sep, 0.0, 0.0),
            'drone2': (self.formation_sep/2.0, self.formation_sep*math.sqrt(3)/2.0, 0.0),
        }
        # FIX 1: ENU frame — z positive = UP
        self.target_altitudes = {
            'drone0': self.base_altitude,
            'drone1': self.base_altitude + self.alt_stagger,
            'drone2': self.base_altitude + 2.0 * self.alt_stagger,
        }

        self.drone_poses = {}
        self.drone_states = {}
        self.last_heartbeat = {}
        self.current_setpoints = {}
        self.leader = 'drone0'
        self.phase = Phase.INIT
        self.pre_offboard_count = 0
        self.pre_offboard_required = int(self.pre_offboard_duration * self.setpoint_rate)
        self.fault_detected = False
        self.takeoff_start_time = None
        self.takeoff_progress = 0.0
        self.takeoff_start_z = 0.0
        self._last_formation_log = 0.0
        self._last_fault_log = 0.0
        self.pending_set_mode = {}
        self.pending_arming = {}
        self.offboard_confirmed = set()
        # FIX 4: Stop arming spam
        self.arm_confirmed = set()
        # FIX 3: Thread control
        self._keep_sp_running = True

        pose_topic = '/local_position/pose'
        for ns in self.drone_namespaces:
            self.create_subscription(PoseStamped, f'/{ns}{pose_topic}',
                lambda msg, n=ns: self.pose_callback(msg, n), qos_profile=qos_profile_sensor_data)
            self.create_subscription(MavrosState, f'/{ns}/state',
                lambda msg, n=ns: self.state_callback(msg, n), 10)

        self.setpoint_pubs = {}
        self.arm_clients = {}
        self.mode_clients = {}
        for ns in self.drone_namespaces:
            self.setpoint_pubs[ns] = self.create_publisher(PoseStamped, f'/{ns}/setpoint_position/local', 10)
            self.arm_clients[ns] = self.create_client(CommandBool, f'/{ns}/cmd/arming')
            self.mode_clients[ns] = self.create_client(SetMode, f'/{ns}/set_mode')

        self.status_pub = self.create_publisher(String, '/swarm/status', 10)
        # FIX 3: Replace ROS2 timer with dedicated thread
        self._sp_thread = threading.Thread(target=self._setpoint_loop, daemon=True)
        self._sp_thread.start()
        ctrl_period = 1.0 / self.control_rate
        self.ctrl_timer = self.create_timer(ctrl_period, self.control_loop)

        log_dir = os.path.expanduser('~/swarm_logs')
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = open(f'{log_dir}/swarm_log_{int(time.time())}.csv', 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['timestamp','leader','phase','d0_x','d0_y','d0_z','d0_online','d0_mode','d0_armed','d1_x','d1_y','d1_z','d1_online','d1_mode','d1_armed','d2_x','d2_y','d2_z','d2_online','d2_mode','d2_armed','sp0_x','sp0_y','sp0_z','sp1_x','sp1_y','sp1_z','sp2_x','sp2_y','sp2_z'])

        self.get_logger().info('========================================')
        self.get_logger().info('  SWARM CONTROLLER v3.0 (Merged Fix)')
        self.get_logger().info('========================================')
        self.get_logger().info('  ENU frame: z positive = UP')
        self.get_logger().info('  Setpoint thread: 20 Hz isolated')
        self.get_logger().info('========================================')
        self.get_logger().info('Waiting for all drones to connect...')

    # FIX 3: Dedicated setpoint thread
    def _setpoint_loop(self):
        period = 1.0 / self.setpoint_rate
        while rclpy.ok() and self._keep_sp_running:
            self.publish_setpoints()
            time.sleep(period)

    def pose_callback(self, msg, namespace):
        self.drone_poses[namespace] = msg
        self.last_heartbeat[namespace] = time.time()
        if namespace not in self.current_setpoints:
            self.current_setpoints[namespace] = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def state_callback(self, msg, namespace):
        self.drone_states[namespace] = msg

    def get_online_drones(self):
        now = time.time()
        return [ns for ns in self.drone_namespaces if ns in self.last_heartbeat and now - self.last_heartbeat[ns] < self.heartbeat_timeout]

    def is_connected(self, ns):
        return ns in self.drone_states and self.drone_states[ns].connected

    def is_armed(self, ns):
        return ns in self.drone_states and self.drone_states[ns].armed

    def get_mode(self, ns):
        return self.drone_states[ns].mode if ns in self.drone_states else 'UNKNOWN'

    def publish_setpoints(self):
        if self.phase == Phase.INIT:
            return
        for ns in self.drone_namespaces:
            if ns not in self.drone_poses:
                continue
            sp = PoseStamped()
            sp.header.stamp = self.get_clock().now().to_msg()
            sp.header.frame_id = 'map'
            if self.phase in (Phase.PRE_OFFBOARD, Phase.SET_OFFBOARD, Phase.ARM):
                p = self.drone_poses[ns].pose.position
                sp.pose.position.x = p.x; sp.pose.position.y = p.y; sp.pose.position.z = p.z
            elif self.phase == Phase.TAKEOFF:
                start_z = self.takeoff_start_z
                target_z = self.target_altitudes[ns]
                interp_z = start_z + (target_z - start_z) * self.takeoff_progress
                p = self.drone_poses[ns].pose.position
                sp.pose.position.x = p.x; sp.pose.position.y = p.y; sp.pose.position.z = interp_z
            elif self.phase == Phase.FORMATION:
                leader_p = self.drone_poses[self.leader].pose.position
                offset = self.formation_offsets[ns]
                sp.pose.position.x = leader_p.x + offset[0]
                sp.pose.position.y = leader_p.y + offset[1]
                sp.pose.position.z = self.target_altitudes[ns]
            elif self.phase in (Phase.HOLD, Phase.FAULT):
                p = self.drone_poses[ns].pose.position
                sp.pose.position.x = p.x; sp.pose.position.y = p.y; sp.pose.position.z = p.z
            sp.pose.orientation.x = 0.0; sp.pose.orientation.y = 0.0
            sp.pose.orientation.z = 0.0; sp.pose.orientation.w = 1.0
            self.setpoint_pubs[ns].publish(sp)
            self.current_setpoints[ns] = (sp.pose.position.x, sp.pose.position.y, sp.pose.position.z)

    def control_loop(self):
        now = time.time()
        self.check_pending_services()
        online = self.get_online_drones()

        if not self.fault_detected and self.phase not in (Phase.INIT, Phase.PRE_OFFBOARD):
            for ns in self.drone_namespaces:
                if ns not in online:
                    self.get_logger().error(f'FAULT: {ns} DISCONNECTED!')
                    self.fault_detected = True; self.phase = Phase.FAULT
                    for o_ns in online:
                        if o_ns not in self.pending_set_mode:
                            self.send_set_mode(o_ns, 'AUTO.LOITER')
                    return

        if self.phase == Phase.INIT:
            all_connected = all(self.is_connected(ns) for ns in self.drone_namespaces)
            all_poses = all(ns in self.drone_poses for ns in self.drone_namespaces)
            if all_connected and all_poses:
                self.get_logger().info(f'All {self.num_drones} drones connected. Starting PRE_OFFBOARD stream...')
                self.phase = Phase.PRE_OFFBOARD; self.pre_offboard_count = 0
            else:
                missing = [d for d in self.drone_namespaces if not self.is_connected(d) or d not in self.drone_poses]
                self.get_logger().info(f'Waiting for: {missing}')

        elif self.phase == Phase.PRE_OFFBOARD:
            self.pre_offboard_count += 1
            if self.pre_offboard_count >= self.pre_offboard_required:
                self.get_logger().info(f'PRE_OFFBOARD complete ({self.pre_offboard_count} msgs). Switching to OFFBOARD...')
                self.phase = Phase.SET_OFFBOARD
                self._set_offboard_start = now
            elif self.pre_offboard_count % int(self.setpoint_rate) == 0:
                self.get_logger().info(f'PRE_OFFBOARD... {self.pre_offboard_count}/{self.pre_offboard_required}')

        elif self.phase == Phase.SET_OFFBOARD:
            all_offboard = True
            for ns in self.drone_namespaces:
                mode = self.get_mode(ns)
                confirmed = ns in self.offboard_confirmed
                if mode != 'OFFBOARD' and not confirmed:
                    if ns not in self.pending_set_mode:
                        self.get_logger().info(f'Sending OFFBOARD to {ns} (mode: {mode})...')
                        self.send_set_mode(ns, 'OFFBOARD')
                    all_offboard = False
            if all_offboard:
                self.get_logger().info('All drones in OFFBOARD. Arming...'); self.phase = Phase.ARM
            elif hasattr(self, '_set_offboard_start') and now - self._set_offboard_start > 5.0:
                if len(self.offboard_confirmed) == self.num_drones:
                    self.get_logger().warn('Timeout but all confirmed. Proceeding to ARM.')
                    self.phase = Phase.ARM
                else:
                    self.get_logger().error(f'SET_OFFBOARD timeout. Only {len(self.offboard_confirmed)}/{self.num_drones} confirmed.')
                    self.phase = Phase.FAULT

        elif self.phase == Phase.ARM:
            all_armed = True
            for ns in self.drone_namespaces:
                # FIX 4: Check arm_confirmed to stop spam
                if self.is_armed(ns) or ns in self.arm_confirmed:
                    continue
                if ns not in self.pending_arming:
                    self.get_logger().info(f'Sending ARM to {ns}...')
                    self.send_arming(ns, True)
                all_armed = False
            if all_armed:
                self.get_logger().info('All drones ARMED. Taking off...')
                self.takeoff_start_z = self.drone_poses[self.leader].pose.position.z
                self.takeoff_progress = 0.0
                self.takeoff_start_time = self.get_clock().now()
                self.phase = Phase.TAKEOFF

        elif self.phase == Phase.TAKEOFF:
            elapsed = (self.get_clock().now() - self.takeoff_start_time).nanoseconds / 1e9
            self.takeoff_progress = min(1.0, elapsed / self.takeoff_duration)
            if self.takeoff_progress >= 1.0:
                self.get_logger().info('Takeoff complete. FORMATION active.'); self.phase = Phase.FORMATION
            elif int(elapsed) > 0 and int(elapsed) % 2 == 0:
                self.get_logger().info(f'Takeoff... {int(self.takeoff_progress * 100)}%')

        elif self.phase == Phase.FORMATION:
            log_elapsed = now - self._last_formation_log
            if log_elapsed > 5.0:
                p0 = self.drone_poses['drone0'].pose.position
                p1 = self.drone_poses['drone1'].pose.position
                p2 = self.drone_poses['drone2'].pose.position
                self.get_logger().info(f'FORMATION | Ldr({p0.x:.2f},{p0.y:.2f},{-p0.z:.1f}) | F1({p1.x:.2f},{p1.y:.2f},{-p1.z:.1f}) | F2({p2.x:.2f},{p2.y:.2f},{-p1.z:.1f})')
                self._last_formation_log = now

        elif self.phase == Phase.FAULT:
            if now - self._last_fault_log > 5.0:
                self.get_logger().error('FAULT STATE ACTIVE.')
                self._last_fault_log = now

        self.log_state(online)
        status = String()
        status.data = f'Leader:{self.leader} | Online:{len(online)}/{self.num_drones} | Phase:{self.phase.name}'
        self.status_pub.publish(status)

    # FIX 5: Non-blocking service check
    def send_set_mode(self, ns, mode):
        client = self.mode_clients[ns]
        if not client.service_is_ready():
            self.get_logger().warn(f'{ns}: set_mode unavailable'); return False
        req = SetMode.Request(); req.custom_mode = mode
        future = client.call_async(req)
        self.pending_set_mode[ns] = future; return True

    def send_arming(self, ns, arm):
        client = self.arm_clients[ns]
        if not client.service_is_ready():
            self.get_logger().warn(f'{ns}: arming unavailable'); return False
        req = CommandBool.Request(); req.value = arm
        future = client.call_async(req)
        self.pending_arming[ns] = future; return True

    def check_pending_services(self):
        for ns in list(self.pending_set_mode.keys()):
            future = self.pending_set_mode[ns]
            if future.done():
                try:
                    result = future.result()
                    if result.mode_sent:
                        self.get_logger().info(f'{ns}: Mode accepted')
                        self.offboard_confirmed.add(ns)
                    else:
                        self.get_logger().warn(f'{ns}: Mode rejected')
                except Exception as e: self.get_logger().warn(f'{ns}: set_mode exception: {e}')
                del self.pending_set_mode[ns]
        for ns in list(self.pending_arming.keys()):
            future = self.pending_arming[ns]
            if future.done():
                try:
                    result = future.result()
                    if result.success:
                        self.get_logger().info(f'{ns}: Arming accepted')
                        # FIX 4: Mark as confirmed
                        self.arm_confirmed.add(ns)
                    else:
                        self.get_logger().warn(f'{ns}: Arming rejected')
                except Exception as e: self.get_logger().warn(f'{ns}: arming exception: {e}')
                del self.pending_arming[ns]

    def log_state(self, online):
        now = time.time()
        row = [now, self.leader, self.phase.name]
        for ns in self.drone_namespaces:
            if ns in self.drone_poses:
                p = self.drone_poses[ns].pose.position
                mode = self.get_mode(ns); armed = 1 if self.is_armed(ns) else 0
                row.extend([p.x, p.y, p.z, 1 if ns in online else 0, mode, armed])
            else: row.extend([0.0, 0.0, 0.0, 0, 'UNKNOWN', 0])
        for ns in self.drone_namespaces:
            if ns in self.current_setpoints: sp = self.current_setpoints[ns]; row.extend([sp[0], sp[1], sp[2]])
            else: row.extend([0.0, 0.0, 0.0])
        self.csv_writer.writerow(row); self.log_file.flush()

    def destroy_node(self):
        # FIX 3: Stop thread before cleanup
        self._keep_sp_running = False
        self._sp_thread.join(timeout=1.0)
        self.log_file.close(); super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
