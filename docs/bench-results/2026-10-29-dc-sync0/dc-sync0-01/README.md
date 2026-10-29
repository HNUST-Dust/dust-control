# DC Sync0 lab integration

User generated source /Users/panpoming/Downloads/dust_hpm5e_dc imported into
external/ssc_dc with manifest. Only DC_SUPPORTED changes0->1 relative to the
previous configured ecat_def.h. Full newly generated DC implementation is used;
no hand-splicing licensed DC state machine and no mutation of external/ssc.
external/ssc_dc is ignored by git. No licensed source published or pushed.

CONFIG_DUST_ECAT_DC_SYNC defaultn, depends on the owner-thread SM infrastructure.
Independent build/dm_rtt_dc explicitly selects external/ssc_dc; CMake rejects
DC mode with FreeRun-generated SSC and DC-generated SSC without DC wiring.
Default ESI/SII bytes unchanged. DC build ESI adds Sync0-only mode, AssignActivate
0x0300 matching official HPM SDK ESI, CycleTimeSync0 factor1; no Sync1 mode.
Generated project min500us is overridden by existing gateway minimum1ms for
this first functional run. It does not advertise125us support. Rx96/Tx400 and
application identity/layout stay unchanged at1ms (signature2699856283).

Independent Sync0 hard IRQ reads status0x098e to acknowledge and increments
atomic pending count plus a semaphore wake. IRQ does not run SSC/Core or take
CAN/UART mutexes. Sync0 stays countable while owner works. Main services PDI
output mapping then Sync0 application/input mapping; ordering follows service
policy, not timestamped physical edge ordering. For the initial1ms/500us shift
these paths should be separated, but actual IRQ/service latency must be checked.
This is deferred synchronization, not zero-jitter hardware-latched application
execution. HPM GPR already prevents sync->PDI IRQ routing; additionally the DC
build masks Sync0/1 AL event bits in SSC event reads so full PDI_Isr cannot
service the same pulse again. Sync1 IRQ disabled and no Sync1 support advertised.

SSC's own DC watchdog remains enabled during startup. Extra software faults:
bit1 pending backlog>1, bit2 IRQ-to-owner latency>=configured period, bit4 owner
handler work>=period. A backlog is serviced once, not replayed as stale business
cycles. Sticky faults protect Core before the pending handler if business is
already Active; a work-overrun fault protects immediately after it returns.
Startup settling is not enforced by this extra policy; fault/maximum timing
fields reset once when Core enters Active, while SSC DC watchdog remains in
force throughout. Heavy inactive PHY scans are suppressed while DC is active;
normal link maintenance remains. No safety path, motor protection or frame
contract removed. Only CAN3 ID3 disable commands allowed by the bench.

Runtime symbol dust_sync0_stats is32B <8I>: version,notifications,services,
coalesced,fault,period_cycles,max_lateness,max_work. Timer24MHz; last3 timing
fields are cycles, not nanoseconds. Notifications/services/coalesced include
startup and stop; max timing/fault reset on business Arm and may include stop.
dust_pdi_stats remains20B. Export via saved matching ELF after master inactive,
before reset/flash. Need nonzero Sync0 services, types2/2, cycle1,000,000ns,
support mask7 and fault0; also inspect standard SSC exceeded/missed/sync status.
Do not infer actual phase alignment from host shift parameter alone.

Independent NUC dust_ethercat_dm_rtt_dc SHA256:
f41d7d1f1cb120dfa55e354c4f002aef28ea9e138ab0897821935776b5952931.
Normal receive/process/send order; no fixed receive offset. Select only LAB
slave as DC reference, configure0x0300/cycle1ms/shift500000ns/Sync1off; provide
steady-clock application time at the same cycle location; queue reference
clock sync every10cycles and slave clock compensation every cycle. DC adds
clock datagrams, so it is not identical bus traffic to historical FreeRun.
Initial clock phase and actual IRQ phase/latency need hardware verification.
No performance acceptance before functional and repeated phase/control tests.

Validation: full legacy native suite PASS; real generated DC SSC SDO/mapping
contracts PASS; Sync0 pending/latency/work/wrap fault contracts PASS; ESI DC mode
and default-byte invariance PASS; target build and NUC C++20 -O2 -Wall -Wextra
-Werror build PASS. Target ROM177384B/RAM211172B. Final saved ELF/HEX/config,
manifest, own-source patch, tests and logs archived. New functionality has not
yet passed a live DC test. No commit/merge/push.

First NUC command:
sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 1000 64 dc-sync0-01 500000

After completion read log/CSV and master state, then export dust_sync0_stats,
dust_pdi_stats, sSyncManOutPar/sSyncManInPar using saved
build/bench-host/dc-sync0-01/zephyr.elf. Check DC activation/period/phase registers
read-only where useful. Never halt an active master. Preserve failure logs and
restore A baseline if the run fails; do not alter watchdog or relax WKC merely
to enter OP. Do not infer performance benefit from64 samples or compare1ms
result directly to historical125us FreeRun.

DC image flashed via west flash --verify:177384 bytes verified. LAB PREOP
and master Idle/inactive confirmed after flash. Physical board now dc-sync0-01.
Await initial1ms64-probe500us-shift test; no live DC result yet.
