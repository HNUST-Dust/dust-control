# Correct ESC write phase coverage

Same cycle500us, shift250us, host binary, CAN3 disable-only profile.
Fix HW_EscWriteIsr route for DC to existing dust_gateway_esc_write_isr wrapper,
so stage7 contains actual PDO ESC write. Coalescing remains disabled; same raw
synchronous hardware write. No SSC licensed-source change or timing-policy change.
Phase format and Active filtering as phases01 (152B); saved matching ELF is
build/bench-host/dc-500us-phases-02/zephyr.elf. Previous phases01 ESCzero is a
measurement coverage gap, not a hardware-performance result.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 64 dc-500us-phases-02 250000

After masterinactive dump dust_sync0_phase_stats, dust_sync0_active_stats,
dust_sync0_stats, dust_first_protection, both SSC sync objects before reset.

Result: PASS64, zero timeouts/WKCmisses. ESCstage confirmed nonzero. Active1307 services, mean105.074us, max215.167us. ESCmean49.764us, slowest-cycle48.25us. Slowestcycle ServiceIo74.958us, ProcessDispatch37us, CapturePublish46us. Stages sum exactly5164cycles=max_work. Further decomposition should target ServiceIo and distinguish preemption before optimizing. RTTmean1498.358us is observational; do not attribute change to diagnostic patch.
