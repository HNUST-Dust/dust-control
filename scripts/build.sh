#!/usr/bin/env bash
# Optional convenience wrapper; equivalent explicit commands: docs/BUILD_RUN.md.
set -eo pipefail
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-orb}" in
  orb) preset=ros-orb ;;
  nuc) preset=ros-nuc ;;
  -h|--help) echo "Usage: $0 [orb|nuc] [extra CMake arguments...]"; exit 0 ;;
  *) echo "Expected orb or nuc" >&2; exit 2 ;;
esac
if [[ $# -gt 0 ]]; then shift; fi
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
cd "$root"
set -x
colcon build --paths . --build-base build/ros --install-base install --merge-install \
  --cmake-args --preset "$preset" "$@"
