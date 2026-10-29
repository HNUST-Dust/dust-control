# DC 500 us functional test

User chose shorter cycle instead of further phase tuning. Firmware DC minimum
cycle lowered from generated 1 ms to 500 us, matching licensed DC SSC project.
Default non-DC descriptor remains unchanged. Host permits only500/1000us DC,
with shift leaving at least100us to cycle end. Use fixed half-period250us shift.
CAN config explicitly can_bench_dm_rtt.yaml: onlyCAN3 ID3 disable FF..FD, RX0x13.
Rx96/Tx400 unchanged, no watchdog threshold relaxation.

New dust_sync0_active_stats is24B `<6I`: version,services,coalesced,fault,
max_lateness,max_work. Only services with business Active at both entry and
exit are included; startup, Arm and Stop cycle excluded. Timing units24MHz.
This measures Sync0 handler only, not full SSC outer loop or end-to-end phase.
Original32B dust_sync0_stats remains unchanged and includes lifecycle activity.
Export both plus dust_first_protection and SSC sync objects after master idle.
Use saved build/bench-host/dc-500us-01/zephyr.elf.

Target build, deferredSync0 Active exclusion contract, ESI generation and
5-mode actualDC SSC integration passed. Host compiledO2 C++20 Wall Wextra Werror.

Run: sudo bash ~/dust-ecat-bench/ethercat_rtt_dc_probe.sh 500 64 dc-500us-01 250000

No125us feasibility claim until Active timings and overload behavior measured.

Result: PASS64 probes, zero timeout/WKC misses. RTT mean1560.582us, median1499.493us, p951999.751us. Active1310 services, max queue wait118.833us, max handler226.417us, zero backlog/deadline flags. First protection is normal Stop-frame handling. No125us support demonstrated. Next decompose Active handler and PDI/outer work before shorter cycle; max wait and work need not belong to same cycle. Raw dumps archived.
