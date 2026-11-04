# Fault detection & tolerance
Detections (10 Hz loop):
- D1 telemetry loss: pose heartbeat older than heartbeat_timeout_sec (2.0)
- D2 in-air disarm: state.armed false during FORMATION/FORWARD/HOLD
- D3 mode dropout: state.mode != OFFBOARD while armed+connected, 2 s grace
- D4 low battery: voltage below battery_low_threshold_v (10.5)
Policies: abort_all (default) = AUTO.LOITER for all online drones + FAULT phase;
isolate_and_continue = isolate the faulty drone (AUTO.LOITER if reachable),
survivors continue; abort only if no active drone remains.
Validated: F1 (pkill -9 -f "px4.*-i 1" mid-FORWARD) -> drone1 isolated,
survivors completed the 30 m path; log: results/f1_telemetry_loss_isolate_and_continue.csv
Pending: F2 (ros2 service call /drone1/set_mode ... AUTO.LOITER),
F3 (/drone1/cmd/arming false), F4 (-p battery_low_threshold_v:=100.0).
Out of scope by design: motor-level FTC (PX4 owns control allocation onboard);
see Hu et al., IEEE TAES 2025, for the research-grade approach.
