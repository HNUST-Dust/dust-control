# IO and CAN worker elapsed overlap diagnosis

Same500us cycle, shift250us, CAN3 ID3 disable-only, Rx96/Tx400, same host.
No protection or priority changes. Existing stages0..7 preserved; ServiceIo
internals now separately timed as8checks/EnterActive,9CAN PumpCan,10UART loop.
Stage4 includes scope boundaries only after this subdivision.

New phase stats version2 is376B, littleendian:
6uint32 header(version2,services,stages11,max_work,worst_lateness,reserved),
11uint64 total at24,11uint64 worker_total at112,
11uint32 maximum at200,11uint32 worst at244,
11uint32 worker_worst at288,11uint32 worker_at_max at332.
Timeunit24MHz. Worker elapsed is cumulative priority0 CAN RX body time AFTER
CaptureGuard acquisition and BEFORE releasing it. It excludes mutex waiting,
but includes interrupts within that body. Owner snapshots worker counter and
cycle timer together under brief irq_lock, then attributes deltas by phase.
Worker body must not block (current drain/refresh calls are nonblocking).
Not a general scheduler/ISR tracer: residual is not pure CPU time, unmeasured
worker boundaries/IRQs/other threads remain. No direct ISR accounting claim.
Per-stage peak arrays worker_at_max show same-cycle worker overlap for that
stage peak; worst[]/worker_worst[] show same max-handler cycle. Sum(worst)
must equalmax_work. Do not sum separate maxima. Only Active entry/exit cycles.

Original active/sync/first protection stats remain compatible. Export phase,
active, sync, first protection and both SSC sync objects with matching saved
build/bench-host/dc-500us-io-01/zephyr.elf after masterinactive, before reset.

Target build, full native suite, actualDC SSC integration and phase accounting
contracts passed. Instrumentation overhead means results are diagnostic.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 64 dc-500us-io-01 250000

Result PASS64, zero timeouts/WKCmisses. Mean RTT1498.611us. Active1307cycles: mean116.989us, max238.292us. CANtx max71.167us; UARTtx max5.25us. Worker cumulative25946cycles,65wakes/64changes confirm instrumentation executes; measured Sync0 body overlap zero. Do not generalize this to absence of IRQ/preemption. Slowest cycle ProcessDispatch79.583us, CANtx67.75us, ESC49.292us. Next split CAN admission/preparation/driver call and driver RAM submit.
