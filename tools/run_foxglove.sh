#!/usr/bin/env bash
# Load the custom telemetry schema for a bare-CMake ROS build.
set -eo pipefail
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
build_directory=$(realpath -e -- "${1:-$project_root/build/lqr14-hardware}")
install_directory="$project_root/.cache/lqr14-ros-install"
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cmake --install "$build_directory" --prefix "$install_directory" > "$project_root/.cache/foxglove-install.log"
export AMENT_PREFIX_PATH="$install_directory${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$install_directory/lib:$build_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec ros2 launch "$project_root/launch/foxglove.launch.py" address:=0.0.0.0
