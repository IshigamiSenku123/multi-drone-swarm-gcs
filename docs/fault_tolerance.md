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
**Validation Status:** 
- ✅ **F1 (Telemetry Loss):** Validated. Drone isolated, survivors completed 30m path.
- ✅ **F2 (Mode Dropout):** Validated. 2s grace period respected, drone isolated, survivors continued.
- ✅ **F3 (In-Air Disarm):** Validated. Immediate detection, drone isolated, survivors continued.
- ✅ **F4 (Low Battery):** Validated. Threshold triggering and isolation working as designed.
Out of scope by design: motor-level FTC (PX4 owns control allocation onboard);
see Hu et al., IEEE TAES 2025, for the research-grade approach.
