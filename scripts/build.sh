#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
preset="${1:-production}"
cd "$root"
cmake --preset "$preset" -S "$root"
cmake --build --preset "$preset" --parallel "${DUST_BUILD_JOBS:-2}"
