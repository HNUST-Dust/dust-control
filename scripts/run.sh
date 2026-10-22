#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$EUID" -eq 0 ]]; then ulimit -l unlimited; fi
exec "${DUST_BUILD_DIR:-$root/build/production}/chassis_controller" --config "$root/config" "$@"
