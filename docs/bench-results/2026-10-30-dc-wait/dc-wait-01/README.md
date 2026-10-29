# Sync0 pending wait attribution, B software retained

SameDC500us,250us nominalshift, CAN3 ID3 FF..FD only,RX0x13,Rx96/Tx400.
REFERENCE_DIGEST=n,PUBLISH_SKIP_OVERWRITE_CLEAR=y,DC_PROFILE=n.
NewDC_WAIT_PROFILE=y only. No service order, priority or safety threshold change.
Capture callback entry beforeESCack; firstqueuednotify afterack starts pending
clock. Measure semgive call and first owner phase checkpoint. Owner phases:
0Other,1SemaphoreWait,2PDI,3Sync0Owner,4PollBeforeSSC,5SSCMainLoop,
6PollAfterSSC,7LinkMaintenance,8InactivePHY,9Idle.
Phase at notification records where owner was, not necessarily CPU executing it.
No kernelready/switch tracing: checkpoints are not pure scheduler latency.

PLIC optional hook measures entry-to-exit elapsed intervals, clips to pending
notification window. Same clipping for CANworker body after mutex acquisition.
No nesting expected in current target. Workerbody includesIRQs, so worker and
IRQ overlap MUSTNOT be added or subtracted as disjoint CPU contributions.
No IRQ assertion timestamp/hardwareclock synchronization claim. Traceoverhead
is included; do not treat instrumentation effects as an optimization.

New dust_sync0_wait_stats is536B littleendian:
header8uint32(version1,services,phases10,max_late,max_ack,max_wake,
max_checkpoint,kept<=4). At32:6uint64 totals(late,ack,wake,checkpoint,worker,IRQ).
At80:10uint64 owner_total; at160:10uint32 origin_counts;
at200:7uint32 latehistogram; at228:reserveduint32;
at232:4records, each76B/19uint32:
ordinal,origin,late,irq_ack,wake_call,checkpoint,detach_cost,worker,IRQ,owner10.
24MHzticks. Latehistbins<10,10..25,25..50,50..100,100..200,200..250,>=250us.
For each record sum(owner)+detach_cost==late. Owner sums stop at pending detach;
detach_cost includes diagnostic copy and the remaining service prologue.
All samples require businessActive at entry and exit. NewIRQ afterdetach cannot
replace detachedrecord; top4 retains longest waits. Overall budget56B,active,
sync,firstprotection/PLIC stats preserved. No reset before collecting.

New 0002Zephyrpatch is optional hook only;0001emptyclaimfix retained. Updated
apply tool validates clean,partial,fullypatched source states in temporary trees,
refuses incompatible edits. CMake rejects WAITbuild missingthehook. PLIC actual
handler tests cover shell/edge/hook8configurations; unit tests cover attribution,
wrap,workerclipping,concurrentnotification,lifecycle,top4; budget modes/fullnative,
realSSC andtargetbuild passed. Decoder synthetic536B case verified.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 2000 dc-wait-01 250000

After masterinactive dump wait_stats,budget_stats,active_stats,sync_stats,
firstprotection,PLICclaims,bothSSCobjects,workerCycles/schedstats if symbolsavailable.
Use saved build/bench-host/dc-wait-01/zephyr.elf, archive logsCSV and run host
 tools/ethercat_dc_profile_decode.py ARCHIVE_DIR. It emits wait correlations plus
budget automatically. Compare longwait top4 and source frequencies before tuning.

Result PASS2000, RTTmean1731.470us. Active41944cycles, meanwait15.902us,max397.083us. Top4waits ALL originateLinkMaintenance:397.083,382.875,317.125,261.208us. Worst367.25us inlinkmaintenance +26.875usPDI; wakecall0.292us,IRQoverlap38.333us,worker0. Sourceupdate_link does extra2MDIOreads andprintk every20polls (~2s), a concrete unproven subsection candidate. Next preserveactual100mslinkchecks but omit periodicrawdump and retest. Activefault0, all-lifecyclefault2 at end; do not conflate teardown with acceptedActivecycles. Optionalworker_cycles symbol absent with detailprofileoff, skipped; allrequireddata archived.
