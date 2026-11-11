#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
source "$root/scripts/ros_env.sh"
dust_source_ros
if [[ "$EUID" -eq 0 ]]; then ulimit -l unlimited; fi
build_directory="$(realpath -e -- "${DUST_BUILD_DIR:-$root/build/production}")"
if [[ "$#" -eq 0 ]]; then
  config_directory="$(realpath -e -- "${DUST_CONFIG_DIR:-$root/config}")"
  set -- --ros-args -p "config_directory:=$config_directory"
fi
export LD_LIBRARY_PATH="$build_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$build_directory/dust_control_node" "$@"
