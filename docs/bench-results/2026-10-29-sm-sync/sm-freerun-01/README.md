# Same-image FreeRun control result

Same SM-capable firmware bytes as sm-owner-01, fresh verified flash before run.
Host uses explicit1C32/1C33 sync type0/0; normal receive-then-send loop, no
receive offset.1000us PDO,64 sequential disable probes, Rx96/Tx400, same host
binary, FIFO40/CPU7/ASPM wrapper. This isolates selected SSC synchronization
mode within the same firmware; it is not the historical A image comparison.
Expected sync_cycles0; export counters and sync structures before another reset.
Matching ELF/HEX copied to firmware build/bench-host/sm-freerun-01.

User command:
sudo bash ~/dust-ecat-bench/ethercat_rtt_sm_sync_probe.sh 1000 64 sm-freerun-01 freerun

Same image flash verified174208 bytes. LAB PREOP/master inactive confirmed.
Await user-run control; physical board remains SM-capable, mode selected by
next host configuration. No control measurement yet.

Control completed:64 PASS, mean1989.265us. Exported sync types0/0 and all
PDI counters0. See ../RESULTS_1000US.md for paired result and next decision.
