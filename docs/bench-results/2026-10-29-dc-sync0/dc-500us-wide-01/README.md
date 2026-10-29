# Batched 500us diagnosis, 512 probes

User requested more information per flash/test. Same500us, shift250us, host
binary, Rx96/Tx400 and CAN3 ID3 FF..FD only. No priority/protection changes.
This version simultaneously splits driver validation/admission/state/messageRAM/
request/release, CoreProcess vs Dispatch, Capture vs Encode, IOchecks/CAN/UART.
Support-driver hooks exist only underCONFIG_DUST_ECAT_DC_SYNC. GNUcleanup
restores owner phase on ALL driver return paths. Normal builds omit hooks.

Phase stats version3, stages22,728B, littleendian:
6uint32 header; total22uint64 at24; worker_total22uint64 at200;
maximum22uint32 at376; worst22uint32 at464; worker_worst22uint32 at552;
worker_at_max22uint32 at640. Stage order matches tools/ethercat_dc_profile_decode.py.
Old parent-stage fields now mostly scope overhead; leaf durations do not overlap.
RAM stage includes conversion/zeroing/messageRAM writing and chip RX workaround
if compiled (SDK defines workaround forHPM6200, not currentHPM5E31).
Means are per accepted Activecycle, not per send invocation. No claim CPUtime:
wall clocks include IRQs/instrumentation; measured worker overlap only knownbody.

New dust_sync0_slow_cycles is800B:12uint32 header(version1,services,kept<=4,
worker_overlap_cycles,histogram7,reserved), then4records each47uint32:
ordinal,work,lateness,phase22,worker22. Records retain top4 longest accepted
Activecycles (not sorted). Histogram ticks thresholds2400,3000,3600,4800,6000,
12000 =100,125,150,200,250,500us at24MHz; lowerinclusive bins.
Original sync/active/first protection formats unchanged. Full native and realDC
SSC tests passed; targeted tests verify accounting, top4 selection and histogram.
Decoder verified against archivedv1/v2 and syntheticv3+slow records.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 512 dc-500us-wide-01 250000

After masterinactive, dump phase_stats,slow_cycles,active_stats,sync_stats,
first_protection,bothSSC sync objects, worker_cycles and dust_ecat_sched_stats
with saved build/bench-host/dc-500us-wide-01/zephyr.elf before anyreset.
Then run tools/ethercat_dc_profile_decode.py ARCHIVE_DIR to save profile.json.
Compare all hypotheses from one run before adding instrumentation or optimizing.

Result FAIL at419/512 successful samples. Full fault snapshot preserved. CPU halted in arch_system_halt, reason1 (spuriousIRQ); PLIC save_irq0 and stack return after z_irq_spurious call establish empty-claim fatal path. Interrupted PC is plic_irq_enable_set_state. Exact cause of claim0 pending/mask race remains unproven. No software protection or Sync0 deadline fault recorded. CAN3 submitted/completed420, RX419; cannot attribute missing response to motor because CPU halted. No reset/reflash after capture. Priority now interrupt reliability, not lower cycle. See result.json, profile.json and raw RAM/CPU stack.
