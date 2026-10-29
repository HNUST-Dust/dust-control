# SSC snapshot-wait attribution v6 — 2026-10-29

Same actual config as worker-v5-opt-01, empty-cache optimization retained.
125 us PDO, 10 ms command interval, 64 sequential CAN3 ID3 disable probes;
Rx 96 B / Tx 400 B. No nonce echo; no strict causal device-pairing claim.

Host PASS: 64 commands, zero timeouts and WKC misses. All 64 firmware records
complete/ordered, optional markers all present, trace overflow zero. Host RTT
mean 829.080 us, P95 998.463 us, max 1123.798 us. This heavily instrumented
localization run is not a performance comparison against v5 or trace-off.

## Snapshot wait decomposition

| Interval | Mean us | P95 us | Max us |
|---|---:|---:|---:|
| Capture → worker next wait entry (nested interval) |10.038|12.625|12.625|
| Capture → next SSC CAN service |49.684|97.708|107.292|
| SSC CAN service → snapshot request |6.384|see additional-intervals.json|see additional-intervals.json|
| Snapshot request → mutex acquired |3.902|6.417|7.625|
| Mutex acquired → matching snapshot copied |4.108|6.667|8.250|
| Capture → snapshot (aggregate) |64.079|114.917|125.250|
| Snapshot → publication |13.057|17.125|21.000|

77.5% of mean capture→snapshot interval precedes the next SSC CAN service
checkpoint. The mutex interval includes trace entry/exit and is not pure lock
contention. Worker next wait entry is after mutex release but does not prove
blocking if another token exists. It is nested in capture→service, not additive.
See additional-intervals.json for wait-entry→service (still not pure scheduler
cost) and service→request statistics computed per record.

## Owner phase at matched capture

| Phase at capture | Count | Capture→snapshot mean us | Snapshot route |
|---|---:|---:|---|
|app_process|4|47.563|app_publish: 4|
|app_publish|3|119.292|poll_publish: 3|
|esc_read|1|66.792|app_publish: 1|
|esc_write|33|76.590|poll_publish: 33|
|maintenance|3|49.722|poll_publish: 3|
|poll_can|1|50.000|poll_publish: 1|
|poll_io|1|30.417|poll_publish: 1|
|poll_tick|2|28.959|poll_publish: 2|
|ssc|9|51.653|poll_publish: 9|
|wait|7|29.470|poll_publish: 7|

33/64 captures interrupt the owner in an ESC block write; these all obtain the
matching snapshot via the later gateway poll and average 76.590 us. Three
captures during app_publish follow an already taken old snapshot and must wait
for a later poll (119.292 us mean, n=3). Correct trace matching excludes that old
publication, as required. Coarse phase tags describe the owner at capture only:
they do not measure continuous phase duration or establish that the entire wait
is ESC transfer time. Small groups are descriptive, not robust distributions.

## Localization and next experiment

The main opportunity is before the next CAN service, not snapshot array copying
or mutex removal. Actual SSC MainLoop unconditionally calls PDO_InputMapping in
FreeRun, then timer/state/mailbox/error processing, before returning to the next
gateway poll. A feedback arriving during the current write must finish that
write and reach another main-thread checkpoint; the worker cannot publish PDO.

Next isolate current ESC write completion, SSC loop tail and gateway poll
pre-service work in the **same pending feedback window**. The existing ESC_END
belongs to the matching feedback's later publication, so it must not be reused
as the end of the old write interrupted by the feedback. Use a distinct marker
or sidecar for that boundary. Only then choose a bounded main-thread refresh
point; avoid calling SSC/Core/ESC from the worker or recursively from ESC write.
The 10 us busy wait is not the dominant phase in this run (7/64, 29.470 us mean
capture→snapshot); do not remove it based solely on an assumed 63 us cost.

Raw trace.bin, host CSV/log, decoded.csv, grouped summary.json, actual config,
image hashes and source patches are retained here. Matching ELF/HEX preserved in
firmware build/bench-host/ssc-v6-01. Nine host tests plus native trace/IO/SSC and
target build passed. V5 measured replay remains byte-identical.

Before RAM export: LAB PREOP, master Idle / Active:no, ASPM all 1, no bench
process. Dump used saved matching ELF, halt/read/resume without reset. Candidate
v6 remains flashed; CPU resumed. No movement frames were sent.
