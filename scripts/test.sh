#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-development}"
case "$preset" in hardware|development|headless|ethercat-ci) ;; *)
  echo "Test preset must be hardware, development, headless or ethercat-ci" >&2; exit 2;;
esac
shift "$(( $# > 0 ? 1 : 0 ))"
"$root/scripts/build.sh" "$preset" "$@"
source "$root/scripts/ros_env.sh"
if [[ "$preset" != headless ]]; then dust_source_ros; fi
cd "$root"
ctest --preset "$preset"
