# DC Sync0 nominal 750 us
Same saved dc-sync0-03 CAN3 ID3 disable-only image and host binary.
Only nominal Sync0 shift changes; 1000 us PDO period, 64 probes.
Clean state via verified flash; no code or protection threshold changes.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 1000 64 dc-sync0-05-shift750 750000

Dump sync and protection statistics with saved build/bench-host/dc-sync0-05-shift750/zephyr.elf after master stops, before reset.
Repeat nominal 500 us afterward to check run-to-run variation.
