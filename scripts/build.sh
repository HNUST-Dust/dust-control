#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-production}"
if [[ $# -gt 0 ]]; then shift; fi
source "$root/scripts/ros_env.sh"
if [[ "$preset" != headless ]]; then dust_source_ros; fi
cd "$root"
cmake --preset "$preset" -S "$root" "$@"
cmake --build --preset "$preset" --parallel "${DUST_BUILD_JOBS:-2}"
