# Pending ESC write and SSC loop tail v7 — 2026-10-29

Same actual .config and empty-cache optimization as ssc-v6-01. 125 us PDO,
10 ms command interval, 64 CAN3 ID3 disable probes, feedback 0x13. Rx 96 B /
Tx 400 B. Host PASS, zero timeouts and WKC misses. 64/64 firmware records are
complete and ordered; trace count 64, overflow zero. Optional checkpoints have
different presence counts (see summary.json); absence is not zero latency.

Host RTT mean 827.045 us, P95 874.902 us, max 1123.134 us. This is a heavily
instrumented localization run, not a direct comparison against earlier images
or a hard real-time bound. DM pairing is sequential without a nonce echo.

## Same-record decomposition: capture during ESC write

48/64 captures occur in esc_write. All 48 contain the entire following chain
and take the matching snapshot through poll_publish. Means and percentile
statistics use this same subset, avoiding sums across differing record counts.

| Interval | N | Mean us | P95 us | Max us |
|---|---:|---:|---:|---:|
|capture_to_pending_write_end|48|34.799|64.250|73.625|
|pending_write_end_to_ssc_return|48|11.777|15.500|17.917|
|ssc_return_to_poll_entry|48|4.183|5.167|5.250|
|poll_entry_to_ssc_service|48|5.444|8.917|13.792|
|ssc_service_to_snapshot_begin|48|4.661|8.333|13.500|
|snapshot_begin_to_snapshot_locked|48|5.997|8.292|8.792|
|snapshot_locked_to_can_snapshot|48|3.959|7.417|11.000|

Capture→snapshot mean is 70.821 us in this subset. Capture→pending-write-end
is 34.800 us; the remaining write-end→snapshot is 36.022 us. P95 values do not
add. First interval includes worker tail (5.338 us mean in this subset),
rescheduling, remaining transfer, preemption and instrumentation, not pure ESC
service time. Worker wait entry does not guarantee actual blocking.

The final post-write path consists of SSC tail (11.777 us), return/checkpoints
(4.183 us), poll work before CAN service (5.444 us), then service-to-request,
mutex and copying. Those observations support checking the main-thread refresh
placement and redundant process-image work; they do not justify moving SSC or
ESC writes to the CAN worker, recursive publication from ESC write completion,
or assuming that eliminating one marker interval yields the same host RTT gain.

## Whole run

- capture_to_worker_wait: n=64, mean=6.135 us
- capture_to_pending_write_end: n=54, mean=39.799 us
- pending_write_end_to_ssc_return: n=54, mean=12.069 us
- ssc_return_to_poll_entry: n=58, mean=4.228 us
- poll_entry_to_ssc_service: n=60, mean=5.823 us
- capture_to_poll_entry: n=60, mean=52.830 us
- capture_to_ssc_service: n=64, mean=56.534 us
- capture_to_snapshot_begin: n=64, mean=62.488 us
- snapshot_begin_to_snapshot_locked: n=64, mean=5.980 us
- snapshot_locked_to_can_snapshot: n=64, mean=4.249 us
- capture_to_can_snapshot: n=64, mean=72.717 us
- pending_write_end_to_can_snapshot: n=54, mean=37.225 us

The whole-run capture→snapshot mean is 72.717 us. Phase counts: {'app_io': 1, 'app_process': 1, 'app_publish': 4, 'esc_read': 2, 'esc_write': 48, 'outer': 1, 'ssc': 6, 'wait': 1}.
PENDING_WRITE_END is distinct from ESC_END of the matching publication and
cannot be filled after the matching snapshot. For non-esc_write captures it
only means the first input write completed while feedback was pending, not
necessarily a write interrupted at capture.

## Next implementation candidates

The localization is now concrete enough to evaluate a single-factor candidate:
reduce main-thread work before the next receive snapshot, or avoid unnecessary
repeated input writes when the published image has not changed. Any input-write
change must preserve SSC/SyncManager handling, state transitions and fresh
publication semantics. A checkpoint after an in-flight write alone cannot
interrupt or shorten that write. Prefer existing main-thread integration hooks;
do not call Core/SSC/ESC from the worker or re-enter PDO mapping from ESC access.

No scheduling or publication change was made in this run. Quantify any candidate
with the same trace ABI, then trace-off repeated ABBA. Raw CSV/log/bin, decoded
CSV, grouped summary, same-subset chain, config, source patches and image hashes
are archived here. Matching ELF/HEX saved in firmware build/bench-host/ssc-v7-01.

Before export: master Idle / Active:no, LAB PREOP, no running benchmark, all
ASPM values restored to 1. RAM dump used matching saved ELF, halt/read/resume
without reset. MCU resumed; current board image remains v7.
