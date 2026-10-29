#!/usr/bin/env bash
# Startup-only comparison with the prior 125us benchmark's ASPM/FIFO conditions.
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo'; exit 2; }
bench=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
tag=${1:-wkc-aspmoff-fifo-01}
mode=${2:-fifo}
[[ $mode == fifo || $mode == other ]] || { echo 'Mode must be fifo or other'; exit 2; }
if [[ $mode == other ]]; then
    [[ ${SUDO_USER:-root} != root ]] || { echo 'OTHER mode requires sudo from the bench user'; exit 2; }
fi
[[ $tag =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
pci=/sys/bus/pci/devices/0000:58:00.0
[[ $(basename "$(readlink "$pci/driver")") == ec_igc ]] || exit 2
[[ $(cat "$pci/vendor") == 0x8086 ]] || exit 2
binary="$bench/dust_ethercat_startup_can3_long_probe"
[[ -x $binary ]] || exit 2
[[ -f $bench/ethercat_peripheral_can3.yaml ]] || exit 2
[[ $(ethercat master) == *'Active: no'* ]] || { echo 'Master is active'; exit 2; }
log="$bench/$tag.log"
[[ ! -e $log && ! -e $bench/$tag.csv ]] || { echo 'Result tag already exists'; exit 2; }
names=(l1_aspm l1_1_aspm l1_1_pcipm)
saved=()
for name in "${names[@]}"; do
    [[ -w $pci/link/$name ]] || exit 2
    saved+=("$(cat "$pci/link/$name")")
done
exec > >(tee "$log") 2>&1
restore() {
    result=$?
    trap - EXIT
    for i in "${!names[@]}"; do
        printf '%s\n' "${saved[$i]}" > "$pci/link/${names[$i]}" || result=1
        echo "restored ${names[$i]}=$(cat "$pci/link/${names[$i]}")"
    done
    exit "$result"
}
trap restore EXIT
echo 'Requires a fresh gateway reset since previous startup/Stop.'
echo 'Startup-only: no Arm or motor business commands. CAN3 protection retained.'
sha256sum "$binary" "$bench/ethercat_peripheral_can3.yaml"
for name in l1_1_aspm l1_1_pcipm l1_aspm; do
    printf '0\n' > "$pci/link/$name"
    echo "temporary $name=$(cat "$pci/link/$name")"
done
if [[ $mode == other ]]; then
    runuser -u "$SUDO_USER" -- "$binary" "$bench/ethercat_peripheral_can3.yaml" "$bench/$tag.csv" 1
else
    "$binary" "$bench/ethercat_peripheral_can3.yaml" "$bench/$tag.csv" 1
fi
