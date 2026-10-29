# Corrected publication trace

Firmware 1d0866c requires a sequence-matched SSC snapshot before a publication is eligible for tracing. Both 64-command rounds PASS with no timeout/WKC miss. Decoder joins 64/64 complete records per round, zero flags. Independent MCU/NUC clocks are never subtracted. Tracing is enabled here, so these RTT values are not pooled with the 2000-sample performance aggregates.

| MCU segment (µs) | Poll mean | Event mean | Event P95 |
|---|---:|---:|---:|
| ESC read 96 B | 16.098 | 13.811 | 13.792 |
| Read end → Rx mapping | 4.223 | 2.897 | 2.958 |
| Rx mapping → protocol processing | 11.386 | 10.180 | 13.708 |
| Protocol processing | 24.573 | 19.875 | 22.042 |
| Process end → accepted | 2.093 | 2.763 | 4.875 |
| Accepted → CAN enqueue | 2.878 | 2.010 | 2.708 |
| CAN enqueue → submit | 6.513 | 4.562 | 6.042 |
| CAN submit → feedback callback | 297.074 | 275.602 | 334.583 |
| Callback → receive queue | 3.757 | 2.972 | 6.667 |
| Queue → worker dequeue | 26.997 | 27.956 | 53.917 |
| Dequeue → worker capture | 6.157 | 2.427 | 5.875 |
| Capture → SSC snapshot | 65.204 | 45.370 | 71.667 |
| SSC snapshot → publication | 17.438 | 18.746 | 24.333 |
| Publication → input mapping | 8.857 | 5.744 | 9.000 |
| Input mapping → ESC write start | 1.684 | 1.010 | 2.083 |
| ESC write 400 B | 64.375 | 50.013 | 53.958 |
| Rx mapping → ESC write end (aggregate) | 538.985 | 469.230 | 585.458 |

The sequential event stages from Rx mapping through ESC write sum to the aggregate ~469.23 µs (rounding aside). ESC read and its mapping transition precede this aggregate. CAN submit→driver return (23.171 µs) and submit→TX completion (153.288 µs) are nested in submit→feedback; do not add them again. Per-stage P95 values cannot be summed into end-to-end P95.

Trace-enabled host mean RTT: poll 868.021 µs; event 781.865 µs. The difference between host RTT and MCU segment duration includes PDO phase and host/network handling; these records do not individually locate all wire/host waiting. Use the trace-disabled four-round result to quantify optimization benefit.

Next priorities, keeping event-driven multithreading and CPU ESC access:

1. Explain the remaining receive-queue→worker delay (mean27.956/P95 53.917 µs): inspect scheduler handoff, interrupt masking and worker execution boundaries before changing priority or adding busy polling. The payload processing itself averages only2.427 µs.
2. Explain capture→SSC snapshot (mean45.370/P95 71.667 µs): correlate SSC loop phase and sections preceding snapshot acquisition, then consider a bounded event-driven early CAN refresh. Preserve sole SSC/PDO ownership by the main thread and stable feedback snapshots.
3. ESC write is still50.013 µs for400 B, but the previously measured synchronous DMA path was slower. Optimize CPU transfer/published image work only after profiling actual bounds and required PDO fields.
4. CAN submit→callback is the largest segment, but includes arbitration, command transmission, motor response and interrupt delivery. Use TX-complete/hardware receive timestamps or an external bus trace to distinguish them; do not label all275.602 µs as CPU overhead. Do not change motor timing assumptions without evidence.

Final board restored to event-driven multithread CPU firmware with DTR and MCAN timing disabled. Test remains a125 µs PDO /100 Hz disable-command probe, not full1 kHz application load or a worst-case guarantee.
