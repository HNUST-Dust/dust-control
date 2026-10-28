#!/usr/bin/env bash
# Run fixed DM-disable benchmarks; a separate operator flashes/verifies each
# image and releases the matching numbered gate only after the master exits.
set -euo pipefail
[[ $EUID == 0 ]] || { echo 'Run with sudo'; exit 2; }
umask 022
run_id=${1:-}
[[ $run_id =~ ^thread-ab-[a-zA-Z0-9_-]+$ ]] || { echo 'Expected a prepared thread-ab-RUN directory'; exit 2; }
bench="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
state="$bench/$run_id"
[[ -d $state && -f $state/prepared && ! -e $state/status ]] || { echo 'Run directory missing, or already used'; exit 2; }
[[ -f $bench/ethercat_rtt_aspm_probe.sh && -x $bench/dust_ethercat_dm_rtt ]] || exit 2
exec > >(tee "$state/coordinator.log") 2>&1
finish() {
    result=$?
    trap - EXIT
    if ((result)); then printf 'FAILED rc=%s\n' "$result" > "$state/status"; fi
    exit "$result"
}
trap finish EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
variants=(multi single single multi)
for index in "${!variants[@]}"; do
    phase=$((index+1))
    variant=${variants[$index]}
    printf 'WAIT %s %s\n' "$phase" "$variant" > "$state/status"
    echo "Waiting for verified firmware: phase=$phase variant=$variant"
    released=0
    for ((attempt=0;attempt<600;attempt++)); do
        if [[ -f $state/release && $(cat "$state/release") == "$run_id:$phase:$variant" ]]; then
            released=1
            break
        fi
        sleep 1
    done
    ((released)) || { echo 'Firmware release timeout'; exit 3; }
    printf 'RUNNING %s %s\n' "$phase" "$variant" > "$state/status"
    tag="${run_id}-p${phase}-${variant}"
    # The child owns temporary ASPM settings and restores them on exit. The
    # tool only sends CAN3 ID3 FF FF FF FF FF FF FF FD, never enable/motion.
    bash "$bench/ethercat_rtt_aspm_probe.sh" 125 1000 "$tag"
    printf 'DONE %s %s\n' "$phase" "$variant" > "$state/status"
    echo "Phase complete: $phase $variant"
done
printf 'COMPLETE\n' > "$state/status"
echo 'ABBA comparison complete; master tools have exited.'
