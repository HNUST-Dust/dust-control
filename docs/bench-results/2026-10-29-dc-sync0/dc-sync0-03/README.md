# DC Sync0 03 — CAN3-only bench profile

Correct DC build CAN config to config/can_bench_dm_rtt.yaml (previous DC
builds accidentally used production can_devices.yaml). Only CAN3 ID3 may
transmit; protection on boot remains enabled and sends FF FF FF FF FF FF FF FD.
Only feedback ID0x13 is admitted. Other buses remain initialized to preserve
existing bench capability configuration but have no TX addresses or protection frames.
No timing, mapping, or protection threshold changes. First-protection diagnostic
from 02 remains enabled; use saved build/bench-host/dc-sync0-03/zephyr.elf.
Generator 13 tests passed; target built successfully.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 1000 64 dc-sync0-03 500000

Result: PASS, 64 probes, zero timeouts/WKC misses. Mean RTT 3423.161 us, median 3007.692 us, p95 4003.267 us. Sync0 1884/1884, no backlog or software deadline fault. First protection is APPL_StopOutputHandler at master teardown. See result.json and raw dumps. This validates functionality only; next isolate nominal Sync0 phase with identical 1 ms traffic and clean reset between runs. Do not compare against historical 125 us baseline as a controlled performance comparison.
