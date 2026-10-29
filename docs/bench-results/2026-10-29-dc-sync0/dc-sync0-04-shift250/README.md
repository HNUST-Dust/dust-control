# DC Sync0 phase: nominal 250 us

Use exactly the saved dc-sync0-03 firmware, with clean reboot via verified flash.
Only host DUST_RTT_DC_SHIFT_NS changes from 500000 to 250000. Same host binary,
1000 us cycle, 64 disable-only probes on CAN3 ID3, feedback ID0x13, Rx96/Tx400.
This is a functional phase comparison; nominal shift is not measured actual phase.
Counters include startup/teardown. No hard real-time or hardware phase claim.

Run:
`sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 1000 64 dc-sync0-04-shift250 250000`

After completion and master inactive, export dust_first_protection,
dust_sync0_stats, dust_pdi_stats and SSC sync objects using saved
build/bench-host/dc-sync0-04-shift250/zephyr.elf before reset.
Next phase is 750000 ns, also with a fresh reset, then consider repeating
500000 ns to assess run-to-run variation before attributing an effect.
