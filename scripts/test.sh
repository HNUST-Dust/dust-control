#!/usr/bin/env bash
# Test automation only. Daily build/run commands are documented in docs/BUILD_RUN.md.
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-development}"
case "$preset" in hardware|development|headless|ethercat-ci) ;; *)
  echo "Test preset must be hardware, development, headless or ethercat-ci" >&2; exit 2;;
esac
if [[ $# -gt 0 ]]; then shift; fi
if [[ "$preset" != headless ]]; then
  setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [[ ! -r "$setup" ]]; then echo "ROS setup not found: $setup" >&2; exit 1; fi
  set +u
  source "$setup"
  set -u
fi
cd "$root"
cmake --preset "$preset" -S "$root" "$@"
cmake --build --preset "$preset" --parallel "${DUST_BUILD_JOBS:-2}"
ctest --preset "$preset"
