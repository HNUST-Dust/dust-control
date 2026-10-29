# receive-90-02: v2 successful run

PASS1000 commands, zero timeout/WKC miss.89809 cycle rows;1000 request joins
complete with no missing pending iteration or timestamp mismatch. A trace-off
firmware, Rx96/Tx400, CPU7 FIFO40 mlock1, same v2 binary as60-02. Normal startup
then observed Active triggers phase switch. No field layout or ESC clock change.

## Cadence and actual offsets

80927 phase send rows,80926 contiguous intervals. No planned125us-grid errors.
Actual send interval mean124.999972us, median125.000us, P95125.032us,
min114.387us/max135.630us. Actual send-entry lateness max10.698us. Therefore
sends follow an absolute grid but individual intervals are not exactly125us;
this is not a hard-real-time guarantee. Lab20us lateness abort did not fire.

Receive-enter minus preceding actual send stamp: mean90.039us, median90.039us,
P9590.070us, min79.451us/max97.071us. The offset is anchored to scheduled send
slot, so actual delayed sends shorten the receive margin. The dataset passed
these observed margins; this does not prove a universal79us safe lower bound.
Receive wake lateness max7.112us. Command separation mean10124.997727us;
median10125us, consistent with v1's90us phase run. Raw first transition is
included; startup receives preceding it are excluded from phase offset stats.

## Latency

| Run | Mean us | Median us | P95 us | P99 us | Max us |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original loop telemetry on01 | 786.362 | 749.0505 | 874.355 | 999.036 | 1124.434 |
| Original loop telemetry off02 | 792.241 | 749.125 | 874.420 | 999.036 | 1124.024 |
|90us v1 run01 | 744.876 | 716.050 | 841.084 | 966.033 | 966.451 |
|90us v2 run02 | 758.777 | 716.085 | 841.454 | 966.000 | 966.609 |

V2 nominal request/response iteration deltas5:35,6:615,7:323,8:27.90us runs
have very similar median/P95, but mean differs13.901us as band populations
change. Compared with original-on median, v2 shifts-32.9655us, consistent with
software observation moving from~124us to~90us after the preceding send.
This is an observation-latency candidate, NOT evidence ESC writes or CAN
response became faster. Exact NIC arrival remains unmeasured.

These are not matched same-version interleaved controls: original loops and
phase mode differ in command separation (original means~10093/10101us vs10125us),
send-slot handling, startup policy and telemetry scope. Do not assign every
mean difference to receive offset or accept a production effect size from
these runs. Similar medians across two phase tool versions are encouraging,
not proof of stability under other load/long runs.

## Decision and next work

Keep90us as experimental candidate.60-02 failed its first early WKC observation
at an actual slot-induced target~57.8us; do not proceed to30us or relax WKC.
The present90us code is diagnostic-only, not enabled in production transport.
No more flashing or benchmark started after collecting this result.

Before acceptance: use one host version and matched fixed command cadence,
legacy/control versus90us in interleaved repeated runs with fresh A startup,
compare actual sending jitter and WKC failures as well as RTT. Consider defining
receive offset relative to actual send while retaining absolute next-send grid,
with an explicit remaining-budget check; current code is slot-anchored. Test
load/tails and lifecycle restart behavior. Driver/hardware timestamps are only
needed if exact NIC or ESC sampling-phase attribution remains the objective.

Current board A, master Idle/inactive, LAB PREOP, no benchmark process; ASPM
restored1/1/1. Saved raw log/sample/cycle CSV, decoded requests and JSON summary.
