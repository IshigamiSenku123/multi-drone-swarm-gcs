#!/bin/bash
set -e

echo "=========================================="
echo "  PX4 SWARM RESTART (Verified v1.13.3)"
echo "=========================================="

# Kill everything
echo "[CLEANUP] Stopping all processes..."
killall -9 px4 2>/dev/null || true
killall -9 gzserver 2>/dev/null || true
killall -9 gzclient 2>/dev/null || true
killall -9 mavros_node 2>/dev/null || true
sleep 3

cd ~/PX4-Autopilot

# Verify build
if [ ! -f "build/px4_sitl_default/bin/px4" ]; then
    echo "ERROR: px4 binary not found."
    exit 1
fi

# Step 1: Start 3 PX4 instances
echo "[1/5] Starting 3 PX4 SITL instances..."
./Tools/sitl_multiple_run.sh 3 &
sleep 20

# Verify
for i in 0 1 2; do
    if pgrep -f "px4.*-i $i" > /dev/null; then
        echo "    Instance $i: RUNNING"
    else
        echo "    Instance $i: FAILED"; exit 1
    fi
done

# Step 2: Start Gazebo (PX4 needs this to initialize MAVLink streams)
echo "[2/5] Starting Gazebo..."
source /usr/share/gazebo/setup.sh
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:$(pwd)/build/px4_sitl_default/build_gazebo
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$(pwd)/Tools/sitl_gazebo/models
export LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:$(pwd)/build/px4_sitl_default/build_gazebo

gzserver Tools/sitl_gazebo/worlds/empty.world &
sleep 5

for i in 0 1 2; do
    rm -rf Tools/sitl_gazebo/models/iris_$i
    cp -r Tools/sitl_gazebo/models/iris Tools/sitl_gazebo/models/iris_$i
    sed -i "s/<mavlink_tcp_port>4560<\/mavlink_tcp_port>/<mavlink_tcp_port>$((4560+i))<\/mavlink_tcp_port>/g" Tools/sitl_gazebo/models/iris_$i/iris.sdf
    sed -i "s/name=\"iris\"/name=\"iris_$i\"/g" Tools/sitl_gazebo/models/iris_$i/iris.sdf
done

gz model --spawn-file=Tools/sitl_gazebo/models/iris_0/iris.sdf --model-name=iris_0 -x 0 -y 0 -z 0.83
sleep 1
gz model --spawn-file=Tools/sitl_gazebo/models/iris_1/iris.sdf --model-name=iris_1 -x 3 -y 0 -z 0.83
gz model --spawn-file=Tools/sitl_gazebo/models/iris_2/iris.sdf --model-name=iris_2 -x -3 -y 0 -z 0.83
gzclient &
sleep 5

# Step 3: Set PX4 parameters via pymavlink (BEFORE MAVROS2, using correct ports)
echo "[3/5] Setting PX4 SITL parameters..."
python3 << 'PYEOF'
import sys
from pymavlink import mavutil
import time

# PX4 sends outgoing MAVLink to these SDK ports (NOT 14580/14581/14582)
configs = [
    {'name': 'drone0', 'port': 14540, 'sysid': 1},
    {'name': 'drone1', 'port': 14541, 'sysid': 2},
    {'name': 'drone2', 'port': 14542, 'sysid': 3},
]

all_ok = True
for cfg in configs:
    print(f"  [{cfg['name']}] Binding to UDP:{cfg['port']} (PX4 SDK output port)...")
    try:
        # Use 'udpin' to BIND to the port and LISTEN for PX4's outgoing data
        master = mavutil.mavlink_connection(f'udpin:127.0.0.1:{cfg["port"]}')
        hb = master.wait_heartbeat(timeout=10)
        if hb is None:
            print(f"    ERROR: No heartbeat from PX4"); all_ok = False; continue
        print(f"    Heartbeat OK (sysid={master.target_system}, mode={hb.custom_mode})")

        # Parameters for SITL without RC/GCS
        params = [
            ('COM_RC_IN_MODE', 1),
            ('COM_DL_LOSS_EN', 0),
            ('NAV_RCL_ACT', 0),
        ]
        
        for param_name, value in params:
            for attempt in range(3):
                master.mav.param_set_send(
                    master.target_system, master.target_component,
                    param_name.encode(), float(value),
                    mavutil.mavlink.MAV_PARAM_TYPE_INT32
                )
                time.sleep(0.3)
            print(f"    {param_name}={value} sent")

        # Save
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
            0, 1, 0, 0, 0, 0, 0, 0
        )
        print(f"    Saved")
        time.sleep(0.5)
        master.close()

    except Exception as e:
        print(f"    ERROR: {e}"); all_ok = False

if not all_ok:
    print("WARNING: Some parameters may not have been set.")
    sys.exit(1)
print("All parameters configured.")
PYEOF

if [ $? -ne 0 ]; then
    echo "WARNING: Parameter setting failed. Arming may be rejected."
fi

# Step 4: MAVROS2 (now ports 14540/41/42 are free)
echo "[4/5] Starting MAVROS2..."
source $HOME/.bashrc
ros2 launch mavros px4.launch fcu_url:=udp://:14540@localhost:14580 namespace:=drone0 tgt_system:=1 tgt_component:=1 &
sleep 5
ros2 launch mavros px4.launch fcu_url:=udp://:14541@localhost:14581 namespace:=drone1 tgt_system:=2 tgt_component:=1 &
sleep 5
ros2 launch mavros px4.launch fcu_url:=udp://:14542@localhost:14582 namespace:=drone2 tgt_system:=3 tgt_component:=1 &
sleep 10

# Step 5: Verify
echo "[5/5] Verifying..."
source $HOME/.bashrc
for i in 0 1 2; do
    echo -n "  drone$i: "
    ros2 topic echo /drone$i/state --once 2>/dev/null | grep -o "connected: true" || echo "NOT CONNECTED"
done

echo ""
echo "=========================================="
echo "  SWARM READY"
echo "=========================================="
echo ""
echo "Next: ~/swarm_ws/install/swarm_ctrl/bin/swarm_controller"
echo "      ~/swarm_ws/install/swarm_ctrl/bin/monitor"
echo ""
echo "Press Ctrl+C to shutdown"
echo "=========================================="

trap "echo ''; echo 'SHUTTING DOWN...'; killall -9 px4; killall -9 gzserver; killall -9 gzclient; killall -9 mavros_node; exit 0" INT
while true; do sleep 1; done
