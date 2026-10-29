# SM synchronized publication: first1ms functional run

64 PASS, no timeout or benchmark WKC misses. Rx96/Tx400, signature2699856283,
PDO1000us, command minimum interval10ms, FIFO40 CPU7 mlock1, trace/DMA off.
RTT mean2986.708us, median2869.644us, P953142.477us, max3151.007us.
This is functional verification, not a comparison against125us FreeRun or
proof of performance gain. Same-period FreeRun control remains required.

After master exited (Idle/inactive, LAB PREOP, no benchmark process), sequential
JTAG exports used saved build/bench-host/sm-owner-01/zephyr.elf. No reset;
CPU resumed after each export. Stats20B and two standard SSC structures68B.
Target uses OBJ_DWORD_ALIGN1 and no packing; decoded offsets are saved in
sync-summary.json and checked against target symbol sizes.

Mode evidence: output sync type1 (SM output); input sync type0x22 (SM2 input);
supported mask3 (FreeRun+SM). min cycle1,000,000ns. dust_pdi_stats version1,
notifications3356/services3356/waits9603/sync_cycles1678. Thus actual SM2
application processing occurred, not a silent fallback to FreeRun. Counts
include startup/idle, not64 response counts or per-command timings. Notice
roughly two notified/services per SM cycle; unrelated/stale AL notices are
filtered before PDI_Isr and must not be equated to two publications per cycle.
No CPU utilization estimate is inferred from these counters.

Saved output/input cycle-exceeded, missed-event, shift-short and sync-error
values are all0. Cycle-exceeded is directly relevant to an event arriving during
SSC handling;0 in this short run is encouraging. Missed-event/sync-error fields
are reset during StartOutputHandler and sync-error is recomputed, so these
post-run snapshots are not a general proof of no transient issues. Unmeasured
worker/IRQ timing and longer load behavior remain outside this functional run.

Host command send-call0.651..2.877us; response receive-call0.891..11.571us.
RTT spread should not be assigned wholly to SSC without response cycle/wake
instrumentation. No independent clocks subtracted. Raw CSV/log, host interval
summary, binary exports and decoded mode summary archived.

ASPM restored1/1/1. Next: same saved SM-capable firmware, fresh verified boot,
explicit FreeRun SDO0/0, same1ms/64 probes and host binary. Require sync_cycles0
for control. Then decide whether lowering configured cycle/minimum to125us
is justified, with actual SSC overrun/error evidence. DC/Sync0 remains disabled.
