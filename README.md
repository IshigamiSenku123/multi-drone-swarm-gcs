# Multi-Drone Autonomous Swarm with GCS

GCS-centric formation control for a 3-drone quadrotor swarm on PX4 v1.13.3 SITL +
Gazebo Classic 11 + ROS 2 Humble (MAVROS 2.14). The ground station streams OFFBOARD
position setpoints at 20 Hz; PX4 closes all inner loops onboard.

## Features
- Triangular formations: equilateral / right-triangle / custom offsets (ROS 2 parameters)
- Coordinated forward translation while holding formation
- Per-vehicle local-EKF-frame correction for multi-vehicle SITL spawn offsets
- Potential-field inter-drone collision avoidance (parameter-gated, default OFF)
- GCS fault detection (telemetry loss, in-air disarm, mode dropout, low battery)
  with `abort_all` / `isolate_and_continue` policies
- Tkinter mission launcher GUI; 10 Hz CSV state logging

## Tested environment
Ubuntu 22.04 | ROS 2 Humble | Gazebo 11.10.2 | PX4 v1.13.3 | MAVROS 2.14.0 | Python 3.10

## Layout
    src/swarm_ctrl/   ROS 2 package (controller, GUI, monitor, setup.py, package.xml)
    scripts/          restart_swarm.sh (3x PX4 SITL + Gazebo + MAVROS + PX4 params)
    docs/             architecture, ROS 2 topology, fault tolerance, version history
    results/          validated flight logs

## Setup
    git clone https://github.com/PX4/PX4-Autopilot.git --branch v1.13.3 --depth 1
    cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh && make px4_sitl_default
    pip3 install --user pymavlink pyulog
    mkdir -p ~/swarm_ws/src && cp -r <this-repo>/src/swarm_ctrl ~/swarm_ws/src/
    cd ~/swarm_ws && colcon build --packages-select swarm_ctrl && source install/setup.bash

## Run
    ./scripts/restart_swarm.sh         # terminal 1, wait for SWARM READY
    ros2 run swarm_ctrl gui_launcher   # terminal 2, or the CLI below
    ros2 run swarm_ctrl swarm_controller --ros-args \
      -p formation_shape:=equilateral -p forward_distance_m:=30.0 \
      -p forward_speed_mps:=0.5 -p fault_policy:=isolate_and_continue

## Fault tolerance - validated results
- F1 telemetry loss (`pkill -9 -f "px4.*-i 1"` mid-FORWARD): detected within the 2 s
  heartbeat window; drone1 isolated; drones 0+2 completed the 30 m path.
  Log: results/f1_telemetry_loss_isolate_and_continue.csv
- F2 mode dropout / F3 in-air disarm / F4 low battery: harness ready, validation pending.
Injection commands: docs/fault_tolerance.md

## Version history
Tags v0.1-v0.6 are byte-exact snapshots of the private development archives;
v1.0.0 is this validated state. See docs/version_history.md.
Only main / v1.0.0 is supported.

## Copyright & Attribution
Copyright (c) 2026 Muhammad Nouman. MIT License - see LICENSE.
Attribution required: retain the copyright notice and cite via CITATION.cff
("Cite this repository" button).

## Acknowledgments
PX4 Autopilot; MAVROS; MDS (MAVSDK drone show) for the GCS-centric architecture;
Hu et al., "Safe and Reliable Distributed Fault-Tolerant Formation Control for
Quadrotor Swarms", IEEE TAES 2025 (potential-field collision avoidance reference).
