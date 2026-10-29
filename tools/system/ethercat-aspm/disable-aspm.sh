#!/usr/bin/env bash
# Dedicated EtherCAT I225 only; intentionally leave power saving disabled.
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo' >&2; exit 2; }
pci=/sys/bus/pci/devices/0000:58:00.0
[[ $(cat "$pci/vendor") == 0x8086 && $(cat "$pci/device") == 0x15f2 ]] || {
    echo 'EtherCAT PCI identity mismatch at 0000:58:00.0' >&2; exit 1;
}
for name in l1_1_aspm l1_1_pcipm l1_aspm; do
    [[ -w $pci/link/$name ]] || { echo "Unavailable ASPM control: $name" >&2; exit 1; }
done
for name in l1_1_aspm l1_1_pcipm l1_aspm; do
    printf '0\n' > "$pci/link/$name"
done
for name in l1_aspm l1_1_aspm l1_1_pcipm; do
    value=$(cat "$pci/link/$name")
    echo "EtherCAT 0000:58:00.0 $name=$value"
    [[ $value == 0 ]] || exit 1
done
