#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/ros_env.sh"
dust_source_ros
build_directory="$(realpath -e -- "${DUST_BUILD_DIR:-$root/build/production}")"
install_directory="$build_directory/ros-install"
# Install the same build's schema/typesupport; never mix independent build directories.
cmake --install "$build_directory" --prefix "$install_directory" > "$build_directory/foxglove-install.log"
export AMENT_PREFIX_PATH="$install_directory${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$install_directory/lib:$build_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec ros2 launch "$root/launch/foxglove.launch.py" "address:=${DUST_FOXGLOVE_ADDRESS:-127.0.0.1}" "port:=${DUST_FOXGLOVE_PORT:-8765}"
