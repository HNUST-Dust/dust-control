#!/usr/bin/env bash
# Compatibility entry; the generic launcher uses the selected build's schema.
set -euo pipefail
if [[ $# -gt 1 ]]; then echo "Usage: $0 [BUILD_DIRECTORY]" >&2; exit 2; fi
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
export DUST_BUILD_DIR="${1:-$root/build/lqr14-hardware}"
export DUST_FOXGLOVE_ADDRESS="${DUST_FOXGLOVE_ADDRESS:-0.0.0.0}"
exec "$root/scripts/foxglove.sh"
