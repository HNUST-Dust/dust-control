# CAN3 scan boundary trace v5 — 2026-10-29

64 sequential CAN3 ID 0x003 disable requests, feedback 0x013; 125 us PDO,
10 ms command interval, Rx 96 B / Tx 400 B. Event worker, CPU word ESC,
DMA off, trace on. Host PASS, zero timeouts and WKC misses. Trace count 64,
overflow 0; 64/64 records complete and ordered, all optional markers present.
Base firmware commit 1d0866c + firmware.patch. Actual config, source patches,
SHA256 and raw CSV/log/bin are retained. Matching ELF/HEX also saved in the
firmware workspace at build/bench-host/worker-v5-01 (outside this git archive).

| Interval | Mean us | P95 us | Max us |
|---|---:|---:|---:|
| Queue → worker execution marker |19.506|25.458|45.208|
| Worker execution → lock acquired marker |7.738|24.417|56.417|
| Lock acquired → CAN3 scan entry |17.577|26.667|37.583|
| CAN3 scan entry → CAN3 dequeued |2.884|6.208|6.250|
| Lock acquired → CAN3 dequeued (aggregate) |20.461|30.875|41.750|
| Queue → CAN3 dequeued (aggregate) |47.705|72.667|99.833|
| Dequeued → cache capture |5.791|10.958|14.083|
| Cache capture → SSC snapshot |43.522|81.125|92.792|
| SSC snapshot → publication |15.764|21.042|26.458|
| ESC input write |51.009|63.667|75.583|

Host RTT mean 815.374 us, P95 874.519 us, max 999.126 us. This is an
instrumented localization run, not a performance comparison. P95 values do
not add; aggregated intervals overlap constituent intervals. No clocks are
subtracted across host and MCU. Sequential DM pairing has no nonce echo.

85.9% of mean post-lock-to-dequeue interval precedes CAN3 scan entry. That
interval includes worker bookkeeping, CAN0–2 iterations, instrumentation and
possible preemption; it is not a pure empty-port cost. The CAN3 entry-to-dequeue
interval includes retained snapshot filtering, message queue get and trace.
The v4/v5 wake and lock distributions vary, and code layout/instrumentation
changed. Their RTT means do not establish an improvement or regression.

A narrow candidate for a subsequent single-factor experiment is to avoid
Now() reads when feedback_count is zero: both retained-snapshot filtering and
expiry scanning currently read time even when no feedback needs checking.
Keep draining receive queues and maintaining drops, generation, batch and
configuration/Stop cleanup; do not simply skip disabled buses. Validate with
native IO tests and compare the same trace ABI, then use trace-off ABBA for
end-to-end quantification. SSC capture wait still needs phase attribution.

Before dump: master Idle / Active:no, LAB PREOP; ASPM all three restored to 1.
Dump used saved matching ELF, halt/read/resume, with no reset.
