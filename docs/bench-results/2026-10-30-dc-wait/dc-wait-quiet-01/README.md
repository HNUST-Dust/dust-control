# Remove periodic PHY raw diagnostic work, retain true link detection

SameB software, DC500us, shift250us, CAN3 ID3 FF..FD only, Rx96/Tx400,
samehostbinary and wait/IRQ/budget instrumentation as dc-wait-01.
Onlyconfigdelta isnew CONFIG_DUST_ECAT_PHY_RAW_DIAGNOSTICS=n. InDC build
periodic extra2MDIOreads+printk every20polls are omitted. All actual100ms
linkchecks, latch-low doubleBMSR, port0BMCR speed/duplex validation,
hpm_esc_set_link_state and change-only messages retained. No priority,
service-order, watchdog or safety-policy change. NonDC samples retainlegacydump.
RawenabledDC,quietDC,nonDC native tests compile actualfunction with fakeMDIO;
verify healthy/latchlow/error/ffff/duplex/unconfigured ports and readcounts.
Wait attribution/lifecycle/deadline tests pass. Target built successfully.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 2000 dc-wait-quiet-01 250000

After masterinactive export dust_sync0_wait_stats,budget_stats,active_stats,
sync_stats,first_protection,dust_plic_claim_stats,bothSSC syncobjects, using
build/bench-host/dc-wait-quiet-01/zephyr.elf. No optional worker_cycles symbol
with detailedprofileoff. CopylogCSV; hostdecoder outputs wait+budget together.
Layouts identical to dc-wait-01 (wait536B,budget56B). Compare longestwaits,
LinkMaintenance phase, IRQoverlap and counts>=250us, not only RTTmean.
If long waits persist in real MDIO checks, evaluate split/asynchronous reads;
do not reduce linkmonitoring to hide a fault. No250/125us feasibility claim yet.

Result PASS2000, zero timeout/WKCmisses. Waitmax397.083 ->171.708us; >=250uswait4 ->0. Active41975cycles meanhandler97.661us,max257us (1cycle>=250). No activefault. RTTmean1775.518us, no RTTimprovement established. Remaining top4origins SSCMainLoop/PollBeforeSSC all proceed into98..120usLinkMaintenance whileSync0pending. Next addsafeowner-loop checkpoint before maintenance, return to existing PDI/Sync0 entry whenpending; preserve real linkmonitoring. No flash/code changes aftercapture.
