# SM2 vs FreeRun at1ms — first paired functional experiment

Same saved sm-owner-01 firmware bytes (SM support compiled in), same host
binary, Rx96/Tx400/signature2699856283,1000us PDO,64 sequential CAN3
ID3 disable requests each, fresh verified flash before each run. Normal master
receive/process/send order, no receive-phase offset, trace/DMA/late-refresh/
fast-notice/coalescing off. CPU7 FIFO40 mlock1, ASPM disabled temporarily and
restored1/1/1. Each run PASS64, zero timeout/WKC miss.

| Mode | Mean us | Tool P50 us | P95 us | Maximum us |
| --- | ---: | ---: | ---: | ---: |
| SM2 | 2986.708 | 2869.644 | 3142.477 | 3151.007 |
| FreeRun | 1989.265 | 1993.606 | 2009.703 | 2013.957 |

Mean difference SM2-FreeRun997.442us, approximately one configured period.
The JSON comparison also includes the conventional even-n median; it differs
slightly from the benchmark's lower-rank P50. Do not mix definitions.

Mode proven using matching ELF RAM exports after master exit, without reset:
SM output1/input0x22, sync_cycles1678, notifications/services3356. FreeRun
output0/input0, sync_cycles/notifications/services/waits all0. Both expose
support mask3 and min cycle1ms. Saved standard error/cycle-exceeded counts0.
Counters include startup, not per-command timings. No silent mode fallback.

Mechanism supported by source review: gateway_poll captures/marks CAN dirty,
but its immediate extra PDO refresh is restricted to !bEscIntEnabled. In SM2
mode PDI processing publishes and writes on output events. A CAN reply arriving
between events waits for the next output event; that event's EtherCAT frame
has already sampled/been sampling the old input, so another later read may be
needed. This is consistent with the observed extra period; unsynchronized
MCU/host traces were NOT used to prove every wire/SM buffer transition.

Conclusion: SM2 functional path works, but this first short1ms pair shows a
clear RTT regression, not acceleration. Lower ESC write frequency/CPU load
is plausible but not directly measured. No hard RT claim;64 requests and one
run per mode do not establish loaded/worst-case timing. Do not extrapolate
exact effect size to125us or combine with historical125us measurements.

Decision: do not enable SM2 in production or blindly reduce the declared1ms
minimum and rerun125us. Restore validated A trace-off baseline after all data
exports. Experimental Kconfig remains defaultn, sources/results retained.
No commit/merge/push. Previous fixed receive-offset experiment remains separate.

If synchronization is pursued further, DC/Sync0 with deliberate publication
phase is a different experiment: the event should permit completing a coherent
snapshot before the target frame reads it. Current licensed generated SSC has
DC_SUPPORTED0 and missing DC source paths; regenerate proper DC-capable SSC,
wire owner-thread deferred Sync0 handling, verify supported AssignActivate and
clock mapping, bound execution/overruns and test against same-period controls.
Simply turning on a macro or reusing the SM2 latency result is insufficient.

Raw mode evidence, logs/CSVs and summaries reside in sm-owner-01 and
sm-freerun-01. No further active experiment starts during this collection turn.

Recovery completed: saved abba-off-A HEX flashed and verified173716 bytes.
Current board A, LAB PREOP, master Idle/inactive, ASPM1/1/1. Current mutable
build directory still contains SM image; use explicit saved HEX/ELF later.
