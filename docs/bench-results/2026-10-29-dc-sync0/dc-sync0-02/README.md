# DC Sync0 02: first protection diagnosis

Same host binary, 1000 us cycle and 500000 ns nominal shift as 01.
No protection threshold or dispatch policy changes.

DC-only `dust_first_protection` is 56 bytes, little endian 14 uint32:
version, valid, reason, previous business state, file pointer, source line,
last Tick now low/high, last Tick exchange_at low/high, operational,
watchdog_us, Tick caller file pointer, Tick caller line.
File pointers reference C strings in the saved ELF rodata. Tick fields describe
last Tick, which can precede a protection in an unrelated path. The first
protection remains latched through later lifecycle callbacks; reboot before rerun.
Do not interpret a last-Tick timestamp as the exact failure timestamp.

Next command:
`sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 1000 64 dc-sync0-02 500000`

After master stops, dump dust_first_protection and both sync statistics with
build/bench-host/dc-sync0-02/zephyr.elf before any reset.
