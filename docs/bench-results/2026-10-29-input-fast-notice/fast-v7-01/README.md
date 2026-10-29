# Atomic notice + late input refresh: fast-v7-01

64 disable probes, PDO 125 us, command interval 10 ms. PASS: no timeouts or
WKC misses. DTR1 v7: 64 complete records, zero flags. Trace enabled throughout;
this small single run is exploratory, not performance acceptance or hard RT.

| Mean (us) | Baseline ssc-v7-01 | Late refresh | Fast notice |
| --- | ---: | ---: | ---: |
| Host RTT | 827.045 | 842.747 | 774.036 |
| Capture to CAN snapshot | 72.717 | 71.070 | 51.346 |
| Publication to input mapping | 6.446 | 16.033 | 5.780 |
| ESC input write | 53.018 | 49.439 | 49.755 |
| Capture to ESC write end | 149.225 | 156.842 | 125.968 |
| CAN RX to ESC write end | 183.530 | 204.667 | 150.316 |
| Rx mapping to ESC write end | 512.996 | 522.790 | 462.900 |

Fast RTT P95 874.142 us, maximum 999.146 us. Absolute host and MCU clocks
are independent; only within-clock intervals are compared. Scheduling and
worker queue latency also vary, so changes cannot all be attributed to the hint.

Counters: refresh version1/checks4020/refreshes2/failures0; notice
version1/probes4020/pending2/unchanged4018. Probes equal pending + unchanged;
99.950% of eligible probes bypassed the full late-refresh service. Positive
hints are not unique motor reply counts. Regular expiry/service remains enabled.

Keep candidate pending trace-off ABBA comparison against baseline. Firmware
remains fast-v7-01. Saved matching ELF/HEX in firmware workspace
build/bench-host/fast-v7-01; all RAM exports used that ELF and resumed CPU.
Post-test master inactive, slave PREOP, ASPM restored. No reset during export.

Config versus late-v7-01 adds only fast notify; versus baseline adds late refresh
and fast notify. Coalescing off, CPU word ESC, event worker, DMA off. Source,
configs, hashes, disassembly, raw logs/CSV/binaries, decoded CSV and summary saved.
Full scripts/test_protocol.sh PASS: 504 cross-end layouts, four SSC modes,
three IO modes, trace, UART/MCAN and clock/ESC/DMA contracts. Actual target build
PASS: ROM176740 B, RAM247056 B. west flash --verify verified176740 bytes.
