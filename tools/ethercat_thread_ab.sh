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
start_phase=${2:-1}
[[ $start_phase =~ ^[1-4]$ ]] || exit 2
[[ -d $state && -f $state/prepared ]] || { echo 'Run directory missing'; exit 2; }
if [[ -e $state/status ]]; then
    [[ $start_phase -gt 1 && $(cat "$state/status") == FAILED* ]] || { echo 'Run is active, complete, or not resumable'; exit 2; }
    variants=(multi single single multi)
    for ((prior=1;prior<start_phase;prior++)); do
        tag="${run_id}-p${prior}-${variants[$((prior-1))]}"
        grep -q '^PASS commands=1000 timeouts=0 WKC_misses=0;' "$bench/dm-rtt-125us-${tag}.log"
        [[ $(wc -l < "$bench/dm-rtt-125us-${tag}.csv") -eq 1001 ]] || exit 2
    done
    mv "$state/status" "$state/status-before-resume-p${start_phase}-$(date +%s)"
    # A previous release must not start the failed phase before a fresh flash.
    rm -f "$state/release"
elif [[ $start_phase != 1 ]]; then
    echo 'Resume requires a failed run'; exit 2
fi
[[ -f $bench/ethercat_rtt_aspm_probe.sh && -x $bench/dust_ethercat_dm_rtt ]] || exit 2
exec > >(tee -a "$state/coordinator.log") 2>&1
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
    ((phase >= start_phase)) || continue
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
