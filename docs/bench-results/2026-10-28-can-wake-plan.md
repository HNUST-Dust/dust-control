# Multithread CAN receive wakeup experiment

Keep multithreading and CPU 32-bit ESC access. DMA remains disabled.

The corrected `mcanfixed` 64-sample trace baseline measured these mean durations (µs):

| Segment | Mean | Interpretation |
|---|---:|---|
| ESC read, 96 bytes | 16.334 | Includes scheduling interference |
| Core.Process | 26.121 | Protocol work |
| CAN submit → feedback callback | 300.981 | Bus/device/interrupt path; includes TX completion |
| Feedback callback → queue | 8.764 | Receive handling |
| Queue → worker dequeue | 27.986 | Scheduling wait |
| Dequeue → capture | 4.027 | Worker processing |
| Capture → publication | 59.974 | Scheduling plus main-loop work; previously not separated |
| ESC write, 400 bytes | 54.468 | Includes scheduling interference |

These segments overlap; do not sum them as an end-to-end budget. Independent NUC and MCU clocks cannot be subtracted directly. The trace-enabled baseline is not comparable to the trace-disabled ABBA aggregate.

First hypothesis: the priority-0 CAN worker's 100 µs timeout causes empty wakeups and preempts the priority-1 SSC loop. Test `CONFIG_DUST_ECAT_CAN_RX_POLL_US=100` versus `0` (event-only). SSC checks cached CAN snapshot expiry so removing the timeout retains the 20 ms stale-data rule. The default stays 100 until results support a change.

DTR1 v3 adds a sequence-matched SSC snapshot timestamp, splitting capture→publication into capture→SSC snapshot and SSC snapshot→publication. The decoder retains v1/v2 compatibility. Optional `dust_ecat_sched_stats` records active worker wakeups, timeouts, empty wakes, changed batches, and SSC expiry events; these counts do not measure CPU utilization.

Six rounds: poll/event with 64 traced commands each, followed by poll/event/event/poll with 1000 commands each and tracing off. Scheduling statistics stay enabled in both variants. Compare only the polling setting within each pair. Archive matching ELF/config hashes and export RAM before flashing the next image.

All rounds use a 125 µs PDO cycle and 10 ms command interval, only CAN3 ID 3 disable frames (`FF FF FF FF FF FF FF FD`), feedback ID 0x13. This is a 100 Hz probe, not the full 1 kHz application workload. The root helper is `tools/ethercat_can_wake.sh`; it waits for verified firmware gates and restores temporary ASPM settings through the child script.

Status: implementation and regression/build validation complete; physical comparison pending.
