#!/usr/bin/env bash
# Compatibility entry for the previous NUC commissioning directory.
set -euo pipefail
if [[ $# -lt 1 || $# -gt 2 ]]; then
  echo "Usage: $0 CONFIG_DIRECTORY [BUILD_DIRECTORY]" >&2; exit 2
fi
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export DUST_CONFIG_DIR="$1"
export DUST_BUILD_DIR="${2:-$root/build/lqr14-hardware}"
exec "$root/scripts/run.sh"
