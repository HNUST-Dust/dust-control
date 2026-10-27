#!/usr/bin/env bash
# 构建 RTT 测试。SOEM 2.x 需要 CMake >= 3.28；
# 若系统 CMake 太旧，自动使用 SOEM/.tools 中自带的 CMake。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SOEM_DIR="${SOEM_DIR:-$ROOT/../SOEM}"
BUILD_DIR="${BUILD_DIR:-$ROOT/build}"

find_cmake() {
  local candidate="$1"
  local version
  version="$("$candidate" --version 2>/dev/null | head -n1 | awk '{print $3}')"
  [[ -z "$version" ]] && return 1
  [[ "$(printf '%s\n' "3.28" "$version" | sort -V | head -n1)" == "3.28" ]]
}

CMAKE_BIN=""
if command -v cmake >/dev/null 2>&1 && find_cmake "$(command -v cmake)"; then
  CMAKE_BIN="$(command -v cmake)"
else
  BUNDLED="$(find "$SOEM_DIR/.tools" -path '*/cmake/data/bin/cmake' -type f 2>/dev/null | head -n1 || true)"
  if [[ -n "$BUNDLED" ]] && find_cmake "$BUNDLED"; then
    CMAKE_BIN="$BUNDLED"
  else
    echo "error: CMake >= 3.28 not found" >&2
    exit 1
  fi
fi

"$CMAKE_BIN" -S "$ROOT" -B "$BUILD_DIR" \
  -DCMAKE_BUILD_TYPE=Release \
  -DSOEM_DIR="$SOEM_DIR"
"$CMAKE_BIN" --build "$BUILD_DIR" -j"$(nproc)"

echo "==> done: $BUILD_DIR/ecat_rtt_test"
