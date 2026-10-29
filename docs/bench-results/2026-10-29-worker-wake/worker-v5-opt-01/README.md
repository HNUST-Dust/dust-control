# Empty-cache time-read candidate — 2026-10-29

Candidate: skip retained-snapshot age filtering and no-new-frame expiry scans
when feedback_count is zero. Receive queues still drain on every pass, with
unchanged per-event age validation, disabled-capacity handling, drops, batch,
generation, expiry, Stop/Arm and snapshot ownership semantics.

Baseline: worker-v5-01; candidate uses byte-identical actual .config and the
same DTR1 v5 markers. optimization.patch isolates the two guarded branches;
firmware.patch contains all changes against base 1d0866c. Image hashes and
actual config are saved. Matching ELF/HEX are preserved in firmware workspace
build/bench-host/worker-v5-opt-01. IO threaded/single-threaded, trace and SSC
integration contracts pass; actual HPM5E31 trace build passes.

Board measurements complete; localized scanning interval improved in this run,
but no end-to-end RTT improvement is established.
Compare lock→CAN3 entry and full queue→capture intervals; retain trace overhead
and host sampling phase limitations. A later trace-off ABBA is required to
quantify end-to-end performance.


## Measured result

Both baseline and candidate: 64/64 complete ordered records, all optional v5
markers present, no trace overflow. Candidate host PASS, zero timeout and WKC
miss. Actual .config is byte-identical. All ASPM settings restored to 1.

| Mean interval us | Baseline | Candidate |
|---|---:|---:|
| Lock acquired → CAN3 entry |17.577|4.921|
| Lock acquired → CAN3 dequeued |20.461|10.257|
| Queue → CAN3 dequeued |47.705|32.745|
| Dequeued → cache capture |5.791|12.322|
| Queue → cache capture (aggregate) |53.496|45.067|
| Cache capture → SSC snapshot |43.522|63.270|
| Queue → SSC snapshot (aggregate) |97.018|108.337|
| SSC snapshot → publication |15.764|18.702|
| Host RTT |815.374|823.184|

The targeted lock→CAN3 entry mean falls by 12.656 us (72.0%); the broader
lock→dequeue interval falls by 10.204 us (49.9%). Queue→capture falls by only
8.429 us, while queue→SSC snapshot increases by 11.319 us. Do not count the
entire local reduction as end-to-end benefit. Host P95 is 874.519 / 874.526 us.
These are one short instrumented run per image, not a causal performance
estimate, ABBA comparison, or hard real-time bound. Clock and code layout /
preemption / host PDO phase can affect distributions.

The code safely removes work on empty caches, and this run supports the local
mechanism. Keep it as a candidate; overall latency acceptance remains pending
SSC phase analysis and trace-off repeated ABBA. No motion frames were sent.
Candidate remains flashed. RAM export used saved candidate ELF, no reset;
CPU resumed, LAB PREOP and master inactive after export.
