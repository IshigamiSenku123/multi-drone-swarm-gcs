# ROS 2 topology (per drone namespace droneX)
| Topic / service | Direction | Use |
|---|---|---|
| /droneX/state | sub | mode, armed, connected |
| /droneX/local_position/pose | sub | ENU position (heartbeat source) |
| /droneX/local_position/velocity_local | sub | ENU velocity (metrics only) |
| /droneX/battery | sub | voltage for D4 (sensor-data QoS) |
| /droneX/setpoint_position/local | pub | 20 Hz position setpoints |
| /droneX/cmd/arming | client | arm / disarm |
| /droneX/set_mode | client | OFFBOARD / AUTO.LOITER |
| /swarm/status | pub | phase + fault summary |
MAVROS2 is launched with namespace:=droneX, so there is NO /droneX/mavros/... prefix.
