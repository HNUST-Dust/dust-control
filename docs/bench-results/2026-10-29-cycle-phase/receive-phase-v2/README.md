# Receive phase v2 — separate startup and observation experiment

Prior receive60-01 never reached OP (operational0/complete0, AL2, armed0,
commands0, boot0). This is not a60us runtime WKC result. Exact startup failure
cause unresolved. Preserve failure data in ../receive-phase-01.

V2 retains legacy receive-then-send sequencing throughout AL/SDO/business
startup. Latch phase_started only when an actual valid Active input image is
observed, then apply requested30/60/90us offset using the previous absolute
send grid. No startup phase rebasing after Active. Keep strict WKC,20us send
lateness, receive-target overrun,8ms timeout, sequential disable-only pairing.
No slave firmware change. Fresh identical A boot before retry.

Configured phase experiment now logs startup, idle and measured cycles within
the same preallocated35s bound. No-command rows have command_batch0; decoder
checks global ordering but excludes these from successful request joins. Per
request rows remain contiguous; the final stop row has no normal send. Legacy
startup rows have send_schedule_ns0. Review startup/first-active boundary
separately when checking planned send grid. Startup/idle records permit full
cadence analysis rather than only pending-request windows. No cyclic file IO.

Old binary retained; new binary dust_ethercat_dm_rtt_receive_phase_v2 SHA256:
860b1a8a38f7d2e5f62acb8e3758045af64bcf7d9073af1c83bf4f26d2a9843c.
Same wrapper filename now explicitly invokes v2. Log identifies
receive_phase_start=after_active. Five Python decoder tests PASS, including
batch0 startup/idle exclusion; NUC native contracts and C++20 -O2 -Wall
-Wextra -Werror build PASS; wrapper syntax and whitespace checks PASS.
These do not prove runtime60us feasibility. Previous90us must be repeated with
v2 for a matched comparison if v2 measurements succeed.

Next user command:
sudo bash ~/dust-ecat-bench/ethercat_rtt_receive_phase_probe.sh 125 1000 receive-60-02 60

Collect raw cycle/sample CSV and log into firmware build/bench-host/receive-phase-v2
then this folder. Do not compare zero/partial samples as an RTT improvement.
No driver reload, network interface change, PDO layout or ESC clock change.

Fresh A flash verified173716 bytes. LAB PREOP/master Idle inactive confirmed
before retry handoff. No live v2 result yet.
