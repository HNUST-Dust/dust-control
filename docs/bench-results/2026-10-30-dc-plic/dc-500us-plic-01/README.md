# PLIC empty-claim fix: 2000 probe validation

Same500us cycle, shift250us, CAN3 ID3 disable-only, Rx96/Tx400, sameNUC binary.
Only change vs wide01 is Zephyr PLIC empty-claim return plus DC32B counters.
No change to priority, SSC, CAN transmission, watchdogs or timing profile.
Prior wide01 fault already archived before flash. New saved matching ELF:
build/bench-host/dc-500us-plic-01/zephyr.elf.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 2000 dc-500us-plic-01 250000

After masterinactive dump dust_plic_claim_stats plus all wide01 symbols:
dust_sync0_phase_stats,dust_sync0_slow_cycles,dust_sync0_active_stats,
dust_sync0_stats,dust_first_protection,sSyncManOutPar,sSyncManInPar,
worker_cycles,dust_ecat_sched_stats. Profilev3/22stages and slow4 format unchanged.
Use tools/ethercat_dc_profile_decode.py after archiving. Claim counters layout
and interpretation in ETHERCAT_PLIC_EMPTY_CLAIM.md (copied guide).
Pass with empty>0 establishes recovery of this observed failure path; pass with
empty0 means it was not exercised. No real-time/reliability guarantee from one run.

Flash verified180708bytes. ELF dust_plic_claim_stats32B confirmed. Post-flashNUC masterIdle/inactive, physicalLinkDOWN, slaves0. Hardware test awaits link restoration; do not claim live validation.

Result: after cable restored, PASS2000, zero timeout/WKCmisses. MeanRTT1670.790us, median1495.448us, p951995.532us. PLIC370392valid, empty0, invalid0: branch not exercised in hardware, do not overclaim. Active41974cycles mean129.953us max276.625us;17cycles>=250us, none>=500us. Worker overlap1cycle/3.875us only; first protection normalStop. See result.json/profile.json/rawdumps. Main fixed costs ESC51.630us, CoreProcess23.385us, Capture15.873us, Encode8.540us perActivecycle. Means for driver stages are diluted by cycles with no send. No code/flash changes after this successful run.
