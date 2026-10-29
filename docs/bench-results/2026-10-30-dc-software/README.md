# Software A/B with equal reduced profiling

A: original bitwiseDigest + original full-clear publisher.
B: bytewise256entry CRCtable + skip clearing header[0,60) and diagnostics,
which successfulPublish fully overwrites. Reserved header[60,64), CAN unused
slots, UART padding and invalidIMU remain cleared. UART preservation unchanged.
No expiry, protection, mapping, priority, CANprofile, host or PDO change.
Capture audit found expiry/mutex handoff significant; no capture optimization
was applied this round. Treat the two software changes as one measured bundle.

Both useDC500us,250us nominalshift, CAN3 ID3 FF..FD only, RX0x13, Rx96/Tx400.
Both PROFILE=n. Config diff verified only REFERENCE_DIGEST and
PUBLISH_SKIP_OVERWRITE_CLEAR. PLIC fix/counters retained. Detailed phase/worker
hooks removed from theseimages; cannot compare timing directly to wideprofile.
Saved build/bench-host/dc-sw-ab-a1 and dc-sw-ab-b1 each include matchingELF/HEX.

New dust_sync0_budget_stats is56B: uint32 version1,services; uint64 total_work;
uint32 max_work,max_lateness,histogram[7],reserved. OnlyActive entry+exit cycles.
24MHzticks; bins<100,100..125,125..150,150..200,200..250,250..500,>=500us.
Preserves original dust_sync0_active_stats,sync0stats,firstprotection/PLICstats.
No Sync0/PDI or outer work guarantee from handler timing alone.

CRC tested all0..4096lengths,4alignments,knownvector and bitmutations against
original. Full PDO byte equivalence across64layout combinations, realUART
windows,prefilleddirtybytes,CAN/IMU,regular/CAN-only refresh,protectedState.
Reference and optimized core contracts passed underASan/UBSan. Fullnative,
budget detailed/lite lifecycle/deadline/histogram tests and realSSC run.

First runA:
sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 2000 dc-sw-ab-a1 250000

After complete and masterinactive dump budget,active,sync,firstprotection,PLIC
and bothSSC syncobjects with savedAELF before flashingB. Then runB similarly.
Use tools/ethercat_dc_profile_decode.py (now supports budget-only data).
If effect/variation remains unclear, use another A/B pair before accepting gain.

First A/B pair complete: bothPASS2000, zero timeouts/WKCmisses. Handler mean111.508 ->96.097us (13.82% lower), observedmax242.208 ->199.667us; >=200us cycles24 ->0. RTTmean1559.464 ->1563.000us; no observed end-to-end gain. Max scheduling wait253.167 ->345.750us, so do not shorten directly. B remains flashed. Only one pair; results do not establish hard bounds or individual attribution between the two optimizations. Full comparison and rawdumps archived.
