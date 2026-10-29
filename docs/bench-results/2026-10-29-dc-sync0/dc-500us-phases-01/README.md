# 500 us DC Active phase diagnosis

Same500us cycle, nominal250us shift, Rx96/Tx400 and CAN3 ID3 disable-only
profile as dc-500us-01. Unchanged host binary and protection thresholds.
Only owner-side cycle-counter instrumentation added; no licensed SSC edits.
Timing is elapsed wall time, includes ISR/priority0 worker preemption.
Instrumentation has overhead; do not compare these RTTs as a pure optimization.

New dust_sync0_phase_stats is152B: sixuint32 header(version1,services,stages8,
max_work,worst_lateness,reserved), eightuint64 totals, eightuint32 per-stage
maxima, eightuint32 same-worst-cycle phase durations. Alllittleendian.
Stage order: SSCother, applicationTick, CANreceive, Process+DispatchIo,
ServiceIo, CaptureIo+Publish, APPL_InputMapping copy, ESCwrite wrapper.
Alltimes24MHz CPUcycle ticks. Only cycles Active at both entry and exit count.
Mean stage = total/services/24. Maxima can occur on different cycles; use
worst[] for decomposition of max_work, with sum(worst)==max_work.
SSCother includes SSC register/watchdog/error accounting and wrapper overhead.
It is not full PDI/SSC outer loop time. Prior dust_sync0_active_stats remains.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 64 dc-500us-phases-01 250000

After masterinactive dump dust_sync0_phase_stats,dust_sync0_active_stats,
dust_sync0_stats,dust_first_protection and SSC sync objects using saved
build/bench-host/dc-500us-phases-01/zephyr.elf before reset.
