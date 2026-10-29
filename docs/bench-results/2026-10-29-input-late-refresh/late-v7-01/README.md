# Write-before-mapping CAN refresh — 2026-10-29

Baseline saved ssc-v7-01; candidate uses same v7 trace ABI, coalescing off,
event worker + CPU word ESC, DMA off, earlier empty-cache optimization retained.
125 us PDO, 10 ms command interval, 64 sequential CAN3 disable probes only.

## Outcome

Host PASS: 64 commands, zero timeouts/WKC misses. Firmware 64/64 complete,
ordered and sequence matched; trace overflow zero, all joined rows ok.
Counters version1: eligible checks 3464, successful late rebuilds 14, failures0.
Rebuild fraction of checks: 0.404%. Counters include CAN generation changes
and expiry updates; 14 is not necessarily 14 unique matching motor replies.
13 selected records first match their CAN snapshot in coarse SSC phase, which
is consistent with the new input mapping checkpoint. The implementation ran.

| Mean interval us | Baseline | Candidate |
|---|---:|---:|
| Capture → snapshot |72.717|71.070|
| Snapshot → publish |14.746|19.212|
| Publish → input-map copy |6.446|16.033|
| ESC write |53.018|49.439|
| Capture → ESC write end (aggregate) |149.225|156.842|
| CAN callback → ESC write end (aggregate) |183.530|204.667|
| Rx mapping → ESC write end (aggregate) |512.996|522.790|
| Host RTT |827.045|842.747|

Host P95 874.902 / 998.455 us; maximum 1123.134 / 999.234 us.
This is one short instrumented run per image, not an ABBA performance estimate.
No overall latency improvement is established. A reduced snapshot timestamp
alone must not be counted as end-to-end benefit. Code layout, preemption,
instrumentation and PDO observation phase also differ between runs.

## Interpretation and next candidate

A late rebuild can replace a stale pending image without adding a hardware
write, and native tests verify exact UART bytes/cursor rules and protection.
However, every eligible input mapping now invokes ServiceCanReceive, which
takes the capture mutex and performs metadata/expiry checks. Only 14 out of
3464 checks rebuild an image. Publish→input mapping is a larger observed
interval, consistent with the extra work in this path, but its entire measured
increase cannot be assigned to that one function from these samples.

Keep this implementation default off. A next candidate should make the
no-change check cheap (e.g. synchronized generation notification) and take the
snapshot mutex only on actual change or required expiry. This must preserve
expiry, timestamp wrap, Stop/Arm and concurrent updates; an unsynchronized read
of worker-owned fields or a flag cleared after a new event can lose feedback.
Keep bounds and single SSC ownership. Do not add more expensive per-cycle
checks or suppress new RxPDO acknowledgements to make the number improve.
No need to run performance acceptance ABBA for this unproven variant yet.

## Reproducibility / state

Full scripts/test_protocol.sh passed (504 cross-end layout cases, all three
SSC variants, IO, CAN/UART, ESC transfer/clock and fault contracts). Actual
HPM5E31 trace build passed. Raw CSV/log/bin, refresh-stats.bin, decoded.csv,
summary.json, actual config/diff, source patch, hashes and policy document are
retained. Matching ELF/HEX in firmware build/bench-host/late-v7-01.

Before export master Idle / Active:no, LAB PREOP, no bench process, ASPM three
values restored to1. RAM export used matching saved ELF, halt/read/resume with
no reset. Board will be restored to saved ssc-v7-01 baseline; append verification
and final state after restoration. Firmware build/dm_rtt_event_trace remains
late-refresh candidate; it must not be used to decode/dump a restored baseline.


Final restoration: saved ssc-v7-01 HEX written and verified 176076 bytes via
west flash --hex-file. LAB slave observed PREOP three times; master inactive.
Both experimental publication switches are off in the restored image. Use
build/bench-host/ssc-v7-01/zephyr.elf for baseline RAM exports; do not use the
candidate build directory ELF. ASPM was restored by the benchmark as recorded.
