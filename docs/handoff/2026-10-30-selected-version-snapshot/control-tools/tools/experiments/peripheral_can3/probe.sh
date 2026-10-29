#!/usr/bin/env bash
# CAN3 three-address disable/zero-torque RTT with UART2/5 RX capture.
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo for FIFO performance comparison'; exit 2; }
bench=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
samples=${1:-2000}
tag=${2:-peripheral-can3-fifo-01}
[[ $samples =~ ^[0-9]+$ && $samples -ge 1 && $samples -le 2000 ]] || exit 2
[[ $tag =~ ^[a-zA-Z0-9_-]+$ ]] || exit 2
[[ -x $bench/dust_ethercat_peripheral_can3_rtt && -f $bench/ethercat_peripheral_can3.yaml ]] || exit 2
[[ $(ethercat master) == *'Active: no'* ]] || { echo 'Master is active'; exit 2; }
pci=/sys/bus/pci/devices/0000:58:00.0
[[ $(basename "$(readlink "$pci/driver")") == ec_igc ]] || exit 2
for name in l1_aspm l1_1_aspm l1_1_pcipm; do
    [[ $(cat "$pci/link/$name") == 0 ]] || { echo "ASPM must be disabled: $name"; exit 2; }
done
[[ ! -e $bench/$tag.log && ! -e $bench/$tag.csv && ! -e $bench/$tag.csv.uart.csv ]] || {
    echo 'Result tag already exists'; exit 2;
}
exec > >(tee "$bench/$tag.log") 2>&1
echo 'Requires fresh gateway reset since previous Stop.'
echo 'CAN3 only: 0x200 zero torque, 0x000 and 0x003 disable; CAN2/IMU excluded; UART RX only.'
sha256sum "$bench/dust_ethercat_peripheral_can3_rtt" "$bench/ethercat_peripheral_can3.yaml"
chrt -f 40 "$bench/dust_ethercat_peripheral_can3_rtt" "$bench/ethercat_peripheral_can3.yaml" "$bench/$tag.csv" "$samples"
