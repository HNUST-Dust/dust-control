#!/usr/bin/env bash
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
version="${1:?Usage: package.sh vMAJOR.MINOR[.PATCH]}"
if [[ ! "$version" =~ ^v[0-9]+\.[0-9]+(\.[0-9]+)?$ ]]; then
  echo "Expected a release tag such as v0.1" >&2; exit 2
fi
build_dir="${DUST_BUILD_DIR:-$root/build/production}"
for option in BUILD_TESTING DUST_BUILD_MPC DUST_BUILD_TOOLS; do
  if ! grep -q "^$option:BOOL=OFF$" "$build_dir/CMakeCache.txt"; then
    echo "Release packaging requires a production build ($option must be OFF)" >&2
    exit 2
  fi
done
package="dust-control-wbr_infantry-$version-linux-$(uname -m)"
mkdir -p "$root/build" "$root/dist"
if [[ -e "$root/dist/$package.tar.gz" ]]; then
  echo "Refusing to overwrite an existing release package" >&2; exit 1
fi
staging="$(mktemp -d "$root/build/package.XXXXXX")"
trap 'rm -rf -- "$staging"' EXIT
prefix="$staging/$package"
cmake --install "$build_dir" --prefix "$prefix"
install -Dm644 "$root/LICENSE" "$prefix/share/dust-control/LICENSE"
install -Dm644 "$root/docs/RELEASE_PACKAGE.md" "$prefix/README.md"
{
  echo "robot=wbr_infantry"
  echo "transport=usb-bulk"
  echo "version=$version"
  echo "commit=$(git -C "$root" rev-parse HEAD)"
  echo "platform=linux-$(uname -m)"
  echo "built_utc=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
} > "$prefix/BUILD_INFO.txt"
tar -C "$staging" -czf "$root/dist/$package.tar.gz" "$package"
(cd "$root/dist" && sha256sum "$package.tar.gz" > SHA256SUMS)
echo "Package: $root/dist/$package.tar.gz"
