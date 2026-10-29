# Worker wake trace v4 — 2026-10-29

64 sequential CAN3 ID 0x003 disable requests, feedback 0x013; 125 us PDO,
10 ms command interval, Rx 96 B / Tx 400 B. Event worker, CPU word ESC,
DMA off, trace on. Host PASS, zero timeouts and WKC misses. All 64 trace
records complete and ordered, all 64 contain both optional worker markers.
No trace overflow. Raw CSV/log/bin, actual config, source patches and SHA256
are retained here; firmware base commit 1d0866c, with firmware.patch applied.

| Interval | Mean us | P95 us | Max us |
|---|---:|---:|---:|
| Queue → worker execution marker |13.493|19.375|21.042|
| Worker execution → lock acquired marker |3.876|6.833|29.000|
| Lock acquired → CAN3 dequeued |16.500|28.833|32.583|
| Queue → CAN3 dequeued |33.869|48.333|63.792|
| Dequeued → cache capture |4.323|6.417|10.125|
| Cache capture → SSC snapshot |53.625|108.250|118.458|
| SSC snapshot → publication |16.675|20.417|40.625|
| ESC input write |49.174|50.375|68.667|

Host RTT mean 803.580 us, P95 874.544 us, max 998.896 us. This is an
instrumented 64-sample localization run, not a performance comparison or a
hard real-time bound. Sequential DM pairing has no nonce echo.

The queue interval is not all scheduler cost. Lock acquisition accounts for
only a small part of its mean here (the 29 us maximum needs separate study).
The post-lock interval includes scans of CAN0–2 before CAN3, message queue
operations, timestamp collection, instrumentation and possible preemption.
It does not establish that empty-port scans alone cost 16.5 us. Next isolate
bus scan boundaries / ISR exit and SSC phase before changing scheduling.
New trace calls also perturb timing; do not compare this run directly to
trace-off performance, or sum per-stage P95 values.

Before RAM export: master Idle / Active:no, LAB PREOP, ASPM three values all
restored to 1. Export used the matching ELF, halt/read/resume without reset.
