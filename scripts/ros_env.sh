#!/usr/bin/env bash
# Shared helper; source from bash launch/build scripts.
dust_source_ros() {
  local setup="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
  if [[ ! -r "$setup" ]]; then
    echo "ROS setup not found: $setup. Use headless for module-only tests." >&2
    return 1
  fi
  local had_nounset=false
  [[ $- == *u* ]] && had_nounset=true
  set +u
  source "$setup"
  if $had_nounset; then set -u; fi
}
