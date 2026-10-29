# Active FreeRun publication coalescing candidate

Board measurement completed: no duplicate writes were skipped. DTR1 v7 unchanged; only functional config difference
from saved ssc-v7-01 is CONFIG_DUST_ECAT_INPUT_WRITE_COALESCE=y. Candidate
retains CPU word writes, event worker, DMA off and earlier empty-cache change.

Full native regression passed (including 504 host/slave layout cases, both
SSC modes, IO, UART/MCAN, ESC and DMA failure contracts). Both SSC modes passed
again after limiting counters to Active. Target trace build passes; linked
PDO_InputMapping calls the adapter after APPL_InputMapping. Actual .config,
diff, firmware patch, policy document and image hashes saved here. Matching
ELF/HEX kept in firmware build/bench-host/coalesce-v7-01. New untracked policy
document is separately archived alongside the tracked-source patch.

After the 64-command disable-only bench, dump dust_ecat_trace and
 dust_ecat_input_write_stats using that saved ELF, after master exits. Stats
20 B `<5I>` = version, active requests, raw write attempts, skips, failures.
No performance improvement is claimed before board measurement.


## Result: negative for this load

Host PASS 64 commands, zero timeouts/WKC misses. Firmware trace 64/64 complete,
ordered, overflow zero. Active write counters: requests 3440, raw attempts 3440,
skips 0, failures 0. Counters balance exactly. Thus this run saved no writes.
Host mean RTT 830.960 us versus earlier v7 827.045 us; this short instrumented
comparison does not establish a performance regression or benefit. P95 is
998.602 us versus 874.902 us. No trace-off ABBA was run.

Candidate capture→snapshot mean 78.728 us versus v7 baseline 72.717 us;
matching ESC write mean 49.322 us versus 53.018 us. These are separate 64-probe
runs with instrumentation/layout/phase effects; do not assign changes to the
optimization when its skip count is zero.

## Zero-skip investigation

The saved ELF's private written_revision symbol was additionally dumped without
reset: 6770. That value only updates after an eligible synchronous write with
no sticky error, and freezes outside Active. It proves eligibility was reached
at least for the latest credited write, not every historical write. Trace
publications range 3334 (first command) to 6768 (last); the 3434 increment is
close to 3440 Active writes over the entire run, including the boundaries.

Source path: APPL_OutputMapping sets new_output; APPL_Application treats every
received RxPDO as a regular publication, even with no new physical CAN command.
The host continues RxPDO every 125 us while DM probes are 10 ms apart. This
explains why iterations can keep generating new publication versions and have
no identical version left to suppress. Counters do not separately classify
ineligible and new-version writes, so they cannot establish the reason for
every single request. There is no evidence here supporting duplicate-write
suppression as a latency improvement in this workload.

Keep feature default off. A next candidate should refresh feedback at the last
safe main-thread mapping boundary (bounded, preserving UART image/windows) so
a feedback received after an earlier snapshot can enter the impending write.
Do not suppress new acknowledgements/versions merely because motor command
payload repeats: RxPDO exchange/session/watchdog semantics still matter.
The original experiment remains available behind its switch for different loads.

All ASPM values restored to 1; master Idle / Active:no; LAB PREOP. RAM exports
used the saved matching candidate ELF and halt/read/resume without reset. No
motion frames sent. Restore saved ssc-v7-01 baseline after exports, with full
image verification; record final restoration status below.


Final restoration: west flash --hex-file build/bench-host/ssc-v7-01/zephyr.hex
--verify confirmed verified 176076 bytes. Board is now the saved ssc-v7-01
baseline, with coalescing off, earlier empty-cache optimization retained.
Use build/bench-host/ssc-v7-01/zephyr.elf for any subsequent baseline RAM export;
build/dm_rtt_event_trace still contains the candidate and is NOT its matching
ELF. Candidate and all raw evidence remain archived for review.
