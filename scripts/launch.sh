#!/usr/bin/env bash
# Starts one foreground process. Never edits configuration or enables motors.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
mode=${1:-control}
if [[ "$mode" == -h || "$mode" == --help ]]; then
  echo "Usage: $0 [control [CONFIG_DIRECTORY] | foxglove [ADDRESS [PORT]]]"
  echo "Defaults: control reads repository config; Foxglove listens on 127.0.0.1:8765."
  exit 0
fi
case "$mode" in control|foxglove) ;; *) echo "Expected control or foxglove" >&2; exit 2;; esac
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$mode" == control && $# -gt 1 ]] || [[ "$mode" == foxglove && $# -gt 2 ]]; then
  echo "Too many arguments; use --help" >&2; exit 2
fi
# Resolve a supplied relative config path before changing to the repository root.
if [[ "$mode" == control ]]; then config=$(realpath -e -- "${1:-$root/config}"); fi
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -r "$root/install/setup.bash" ]]; then
  echo "Missing install/setup.bash; build the project first." >&2; exit 1
fi
source "$root/install/setup.bash"
cd "$root"
set -x
if [[ "$mode" == control ]]; then
  exec ros2 run dust_control dust_control_node --ros-args -p "config_directory:=$config"
fi
exec ros2 launch dust_control foxglove.launch.py "address:=${1:-127.0.0.1}" "port:=${2:-8765}"
