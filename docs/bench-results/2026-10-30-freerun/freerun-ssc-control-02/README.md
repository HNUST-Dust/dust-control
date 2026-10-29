# Fresh-reset A control after two B passes

Use original saved freerun-return-01 HEX/ELF, SHA256 verified against baseline
archive, no rebuild. Original unusedAL read present. Same125us CAN3-only bench
and original master. Compare to A755.877us and B718.904/722.802us means.
All independent sessions need gateway reset/new boot context after Stop.
Pending: sudo bash ~/dust-ecat-bench/ethercat_rtt_aspm_probe.sh 125 2000 freerun-ssc-control-02

No performance promotion before this control. Export using saved matching A
ELF under firmware build/bench-host/freerun-ssc-control-02 after master exits.

Completed fresh-reset A/B/B/A control: all4valid sessions PASS2000 with no
timeouts/WKC misses. Means A755.877,B718.904,B722.802,A776.132us.
A pooled mean766.0045us vs B720.853us: observed -45.1515us (-5.89%).
This supports keeping B as the selected lab baseline; the optional setting
remains defaultn outside the explicit FreeRun experiment configuration.
P50/P95 nearly unchanged,999us tail recurs; no worst-case or direct45us
register-access saving claim. Sequential4sessions are not randomized loaded
validation. Failed no-reset02 had zero samples and is excluded.
Restore saved B firmware after exporting A control. Next investigate master
receive observation timing without introducing fixed receive offset.
