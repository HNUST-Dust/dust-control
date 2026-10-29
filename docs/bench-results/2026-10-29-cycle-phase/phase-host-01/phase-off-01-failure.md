# Invalid control run: phase-off-01

cycle_telemetry=0. FAIL: Benchmark deadline exceeded after35s, No RTT samples.
CSV contains header only. No quantitative overhead comparison is possible.
No OP/WKC/state startup counters were collected; current logs do not establish
whether AL startup, PDO validity, or business handshake stalled. Do not infer
zero commands sent merely from zero completed samples. After release master
Idle/inactive, LAB PREOP, AL status2/error0; these are post-release state,
not evidence of in-run state. ASPM restored1/1/1, no benchmark process.

Raw failure retained. Restore identical saved A firmware with fresh verified
flash while master inactive before retry phase-off-02. Do not change timing,
PDO layout or binary. Previous on/off pair incomplete; startup cause unresolved.

Recovery flash complete: identical A HEX, verified173716 bytes. LAB PREOP,
master inactive; await phase-off-02 with unchanged host binary. Source review
shows kStop calls Core::Protect; this is a possible lifecycle factor, not a
proven root cause of the no-sample run. Fresh boot avoids retained state.
