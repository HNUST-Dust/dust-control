#!/usr/bin/env bash
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo' >&2; exit 2; }
root=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
binary=/usr/local/sbin/dust-ethercat-disable-aspm
dropin=/etc/systemd/system/ethercat.service.d/60-disable-aspm.conf
bash -n "$root/disable-aspm.sh"
systemctl cat ethercat.service >/dev/null
for name in l1_aspm l1_1_aspm l1_1_pcipm; do
    test -w "/sys/bus/pci/devices/0000:58:00.0/link/$name"
done
[[ $(cat /sys/bus/pci/devices/0000:58:00.0/vendor) == 0x8086 ]]
[[ $(cat /sys/bus/pci/devices/0000:58:00.0/device) == 0x15f2 ]]
for pair in "disable-aspm.sh:$binary" "60-disable-aspm.conf:$dropin"; do
    source=${pair%%:*}
    target=${pair#*:}
    if [[ -e $target ]] && ! cmp -s "$root/$source" "$target"; then
        echo "Existing configuration differs; inspect before replacing: $target" >&2
        exit 1
    fi
done
install -d -m 0755 /etc/systemd/system/ethercat.service.d
install -m 0755 "$root/disable-aspm.sh" "$binary"
install -m 0644 "$root/60-disable-aspm.conf" "$dropin"
systemctl daemon-reload
"$binary"
echo 'Applied now; EtherCAT service starts will reapply these settings. No service restart performed.'
systemctl is-enabled ethercat.service
