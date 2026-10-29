# DC pre-maintenance checkpoint, CAN3-only

500us, 2000 samples, Sync0 shift250000ns; no fixed host receive offset.
PASS2000, zero timeouts/WKC misses/Active sync faults/coalescing.
Matching ELF/HEX retained at firmware build/bench-host/dc-maint-check-01.
Raw RAM exported without reset after the master exited; CPU resumed.

See result.json and profile.json for measured changes and limits. The maximum
wait decreased but the mean increased; RTT mean improvement is not established
by one run because cycle-phase distribution varies. Remaining worst waits
are dominated by an in-progress normal MDIO link poll. Next experiment splits
normal reads across owner-loop iterations, retaining actual link checks.
