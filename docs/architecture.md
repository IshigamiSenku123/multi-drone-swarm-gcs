# Architecture
GCS-centric split: the laptop runs the outer loop (formation geometry, collision
avoidance, fault management) at 10-20 Hz; PX4 runs all inner loops (attitude,
rate, mixer, EKF) onboard at 250+ Hz.
Threads: isolated 20 Hz setpoint publisher; 10 Hz control/fault loop; executor
for subscriptions.
Frame handling: each PX4 SITL instance has its own local EKF origin at its spawn
point; the controller transforms world-frame formation targets into per-drone
local frames via spawn offsets (0,0 / +3,0 / -3,0).
Phases: INIT -> PRE_OFFBOARD -> SET_OFFBOARD -> ARM -> TAKEOFF -> FORMATION ->
FORWARD -> HOLD (plus FAULT).
Position setpoints are used because PX4 OFFBOARD only requires a >=2 Hz stream;
velocity/force-level control from a GCS over a radio link is unsafe and
unnecessary while PX4 owns the inner loops.
