# Trace-off fast notice ABBA result

All four runs PASS: 1000 sequential CAN3 ID3 disable probes per run, zero
timeouts and zero benchmark WKC misses. 125 us PDO / 10 ms command interval;
CPU7, FIFO40, mlock enabled, identical ASPM wrapper. ASPM restored1/1/1.

A: late refresh off, fast notice off. B: both on. Same source otherwise.
Both: RTT trace off, event CAN worker on/event-only, CPU ESC, DMA off,
coalescing off. Config diff, firmware patches and SHA256 identities archived.
Each run started after a fresh verified flash of its saved image.

| Run | Mean us | Median us | P95 us | P99 us | Maximum us |
| --- | ---: | ---: | ---: | ---: | ---: |
| A1 | 784.611 | 749.049 | 874.335 | 998.869 | 999.611 |
| B1 | 789.351 | 749.141 | 874.229 | 998.669 | 999.468 |
| B2 | 772.925 | 749.006 | 874.184 | 874.778 | 999.138 |
| A2 | 787.091 | 749.043 | 874.369 | 998.969 | 999.448 |

Pooled descriptive means (2000 samples each): A 785.851 us, B 781.138 us;
B-A -4.713 us (-0.600%). Pooled samples are not independent
replicate runs. A run means differ by 2.480 us; B run means differ by16.426 us.
B1 is slower than either A run; B2 is faster. Median and P95 barely change.
This ABBA test does not reproduce the previous trace-on 6.4% improvement.
With only two runs per image, no stable small benefit or causal explanation
is established. Keep optional late-refresh/fast-notice disabled pending further
work; do not accept it as an end-to-end performance optimization.

Current physical board: A trace-off baseline, last flash verified173716 bytes.
Matching saved ELF/HEX: firmware build/bench-host/abba-off-A; B artifacts at
build/bench-host/abba-off-B. Current mutable build directory contains B, so
use explicit saved HEX/ELF for later operations. No further flash after A2.
Master Idle/inactive, LAB slave PREOP, no benchmark process after collection.
No hard RT claim. Raw CSV/logs and per-run RTT/wake lateness summaries saved.

Next investigation: reconsider how much work is done on every RxPDO and whether
full input publication can be reduced without breaking exchange/status/UART
semantics. Previous same-revision write coalescing skipped zero writes because
each new RxPDO generated a publication. Any new policy requires semantic review
and contract tests before board measurement; no implementation change here.
