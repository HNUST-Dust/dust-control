#!/usr/bin/env bash
# 专用 I225 EtherCAT 口临时关闭 ASPM 做 RTT 对比；退出时恢复全部原值。
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo'; exit 2; }
bench=/tmp/dust-ecat-bench
pci=/sys/bus/pci/devices/0000:58:00.0
[[ $(basename "$(readlink "$pci/driver")") == ec_igc ]] || { echo 'Expected ec_igc at 58:00.0'; exit 2; }
[[ $(cat "$pci/vendor") == 0x8086 ]] || exit 2
[[ -x $bench/dust_ethercat_dm_rtt && -f $bench/ethercat_dm_rtt_250us.yaml ]] || exit 2
exec > >(tee "$bench/dm-rtt-250us-aspmoff.log") 2>&1
names=(l1_aspm l1_1_aspm l1_1_pcipm)
saved=()
for name in "${names[@]}"; do
    [[ -w $pci/link/$name ]] || exit 2
    saved+=("$(cat "$pci/link/$name")")
done
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
for name in l1_1_aspm l1_1_pcipm l1_aspm; do
    printf '0\n' > "$pci/link/$name"
    echo "temporary $name=$(cat "$pci/link/$name")"
done
"$bench/dust_ethercat_dm_rtt" "$bench/ethercat_dm_rtt_250us.yaml" "$bench/dm-rtt-250us-aspmoff.csv" 1000
