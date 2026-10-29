# Multithread event-wakeup comparison

125 µs EtherCAT PDO cycle, 10 ms CAN3 ID3 disable-command interval, feedback ID0x13. All variants use multithreading, priority-0 CAN receive worker and priority-1 SSC, CPU 32-bit ESC access, no DMA or MCAN timing sidecar. Four performance rounds disable DTR tracing; optional scheduling counters remain enabled in both variants. Poll/event/event/poll, 1000 samples each, all PASS, no timeouts or WKC misses. Temporary ASPM settings restored after each round.

| Pooled 2000 samples | Poll 100 µs | Event only |
|---|---:|---:|
| Mean RTT, µs | 848.673 | 749.799 |
| P50 | 873.736 | 748.967 |
| P95 | 999.146 | 873.997 |
| P99 | 1124.008 | 874.515 |
| Maximum | 1249.409 | 999.638 |
| Above 1 ms | 41 | 0 |

Mean improves 98.874 µs (11.65%). Active empty worker wakes fall from 49953/49890 per poll run to 1 per event run; both event runs process 1000 changed batches with no timeouts. This supports removing the receive-worker polling timeout, while retaining the separate SSC expiry check. It does not measure CPU utilization or guarantee worst-case latency. This probe is 100 Hz, not full 1 kHz application traffic. The DM feedback pairing is sequential, not a nonce echo.

Raw CSV/logs, scheduling-counter RAM, actual configs and premeasurement build hashes are archived here. The original trace-enabled 64-sample rounds have a newly discovered instrumentation race: worker capture alone incorrectly allowed publication of an older SSC snapshot to be stamped as containing the feedback. The new snapshot→publication interval is nonmonotonic in 23 poll and 2 event records. Preserve these raw records, but do not use either trace round for precise publication-stage analysis; even monotonic records may select an old publication. This also limits earlier capture→publication trace conclusions. Whole RTT in host CSV and the four trace-disabled performance rounds are unaffected.

Fix: require the sequence-matched SSC snapshot before marking publication eligible. Add a regression simulating a worker capture before publication of an old snapshot. Two corrected trace rounds are pending.

Firmware event-wakeup implementation: afcaed5; original performance images include this implementation. Host decoder/harness: 0404ef3. Firmware default is promoted to event-only after these results; corrected trace instrumentation is built separately.
