#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# 配置显式指定，不修改配置或权限；裸 CMake 构建也能找到本版本 ROS 类型库。
set -eo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 CONFIG_DIRECTORY [BUILD_DIRECTORY]" >&2
    exit 2
fi
project_root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
config_directory=$(realpath -e -- "$1")
build_directory=$(realpath -e -- "${2:-$project_root/build/lqr14-hardware}")
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
export LD_LIBRARY_PATH="$build_directory${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec "$build_directory/dust_control_node" --ros-args -p "config_directory:=$config_directory"
