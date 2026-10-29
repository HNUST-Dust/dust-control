# Incremental normal PHY link maintenance

Pending hardware test: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 2000 dc-maint-step-01 250000

Only CAN3 ID3 disable command; same96/400-byte PDO,500us cycle, Sync0shift250000ns.
At most one normal MDIO read per DC owner iteration. Port0 doubleBMSR+BMCR,
Port1 doubleBMSR and failure/link-transition behavior retained. Quiet profiling
configuration unchanged from dc-maint-check-01. Native actual-function tests
PASS across DC quiet/DC raw/nonDC, including stepped per-read failures and
one-read-per-step bound. Target build PASS181456B ROM/212084B RAM.

After the master exits, export RAM using the matching saved ELF under firmware
build/bench-host/dc-maint-step-01 before reset/reflash. Compare wait histogram,
max/mean, maintenance attribution, handler budget, faults and final RTT; do not
claim gain from a single change in RTT mode distribution. Raw diagnostics mode
retains original blocking behavior. A single MDIO read is still synchronous.

Hardware run complete: PASS2000, zero timeout/WKC miss. See result.json and
profile.json for snapshots and comparison. No overall latency gain established.
Link maintenance no longer appears in the top4 recorded waits, but this does
not bound all maintenance waits. Remaining tails involve semaphore wake/PDI,
SSC main loop and overlapping IRQ time. Keep500us; do not infer250us feasibility
from the absence of >=250us samples in one run. No new firmware flash after
this export; incremental-maintenance image remains installed.
