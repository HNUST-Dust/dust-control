# Standalone communication benchmarks

These tools remain available for performance investigations and are deliberately
outside the default production dependency graph. The legacy CAN/USB tools use
historical firmware protocols and remain independent of the current USB v4
controller; the ROS state benchmark below uses the current message and node. The active MPC algorithm is in src/chassis/mpc; EtherCAT
code is maintained on its separate development branch. Run
`bash tests/benchmarks/can_usb_stress_test/tools/build_native.sh` inside Linux to build
and run the benchmark's offline checks. Real-device benchmark commands retain
their original opt-in requirements and are not launched by the main build.

## Current ROS 2 state / Foxglove benchmark

`ros_state_performance.py` measures the production simulated controller separately
from a synthetic 1 kHz publisher. The latter busy-waits in a **test-only process**
to isolate transport capacity from VM timer wakeup limits; it consumes one CPU and
is neither installed nor built by default. Neither path opens USB or enables motors.

With ROS, the generated Python messages and `tests/requirements.txt` available:

```bash
source /opt/ros/humble/setup.bash
cmake -S . -B build -DBUILD_TESTING=ON -DDUST_BUILD_ROS2=ON
cmake --build build --target dust_control_node ros_state_benchmark_publisher --parallel 2
export PYTHONPATH="$PWD/build/rosidl_generator_py:$PYTHONPATH"
export LD_LIBRARY_PATH="$PWD/build:$LD_LIBRARY_PATH"
# Set BRIDGE_PREFIX to the prefix returned by: ros2 pkg prefix foxglove_bridge
export ROS_DOMAIN_ID=169 ROS_LOCALHOST_ONLY=1
.cache/test-venv/bin/python tests/benchmarks/ros_state_performance.py \
  --bridge-prefix "$BRIDGE_PREFIX" --source controller \
  --seconds 10 --warmup 2 --output /tmp/controller-state.json
.cache/test-venv/bin/python tests/benchmarks/ros_state_performance.py \
  --bridge-prefix "$BRIDGE_PREFIX" --source synthetic \
  --binary build/ros_state_benchmark_publisher \
  --seconds 10 --warmup 2 --output /tmp/transport-state.json
```

Each source runs three cases: fast ROS subscriber, ROS plus a real Bridge WebSocket
client, and a slow (~20 Hz) ROS subscriber plus the WebSocket client. Subscribers use
BestEffort / Volatile / KeepLast(1); endpoint reliability is checked at runtime.
Some RMW discovery implementations return history depth UNKNOWN (0); depth one is
configured explicitly at the publisher, subscriber and Bridge.

Reports include receive rate, same-host source age percentiles, arrival intervals,
sequence gaps, payload throughput, CPU (one core = 100%), process-tree RSS and source
missed releases. Gaps include intentional latest-state selection and are **not** a
network packet-loss rate. Source age includes non-RT scheduling and Python client
cost, not just DDS wire time. The WebSocket measurement excludes the Foxglove GUI
renderer. Bridge CPU/RSS includes its launch process; summed RSS counts shared pages
in each process. Run sequentially without concurrent builds or other benchmarks.

Results: [2026-10-25 report](../../docs/benchmarks/ros_state_1khz_2026-10-25.md).

For an RT scheduling comparison, append `--realtime` to the controller command in
an environment with the required scheduling privileges and sufficient memlock.
The script still forces simulated input and never enables hardware output.
