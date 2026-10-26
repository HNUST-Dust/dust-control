#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$EUID" -eq 0 ]]; then ulimit -l unlimited; fi
if [[ "$#" -eq 0 ]]; then
  set -- --ros-args -p "config_directory:=$root/config"
fi
build_directory="${DUST_BUILD_DIR:-$root/build/production}"
export LD_LIBRARY_PATH="$build_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$build_directory/dust_control_node" "$@"
