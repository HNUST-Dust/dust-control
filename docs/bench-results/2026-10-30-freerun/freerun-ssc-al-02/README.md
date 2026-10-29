# Repeated run without gateway reset: startup failure

02 exited after35s, zero RTT samples;03 was not executed because loop breaks
on failure. Preserve raw log/CSV/RAM; dump without reset after master exited,
resume CPU. Master read-only state Idle/inactive, PREOP, linkUP. Cumulative
master lost-frame counters are not per-run loss evidence.

Successful prior run issues Stop -> Core::Protect(Error::State). Core does
not auto-rearm; configuration WriteObject requires Unconfigured, and Configure
also rejects any other state. Standard AL error acknowledgement does not
clear this guard (existing native test explicitly checks rejection). Repeated
independent bench sessions require an authorized gateway reset/new boot ID.
The supplied shell loop incorrectly omitted this precondition. No relaxation
of watchdog, fault, configuration-write or session validation is appropriate.

No RTT samples => no latency conclusion. Reset same image, then run03 alone.
