# Host cycle telemetry experiment prepared

Board remains trace-off A (saved firmware abba-off-A). Rx96/Tx400,125us,
late-refresh and fast-notice disabled. No firmware flash or mapping change.
Original NUC dust_ethercat_dm_rtt and original ASPM wrapper retained.
New binary dust_ethercat_dm_rtt_phase; new wrapper ethercat_rtt_phase_probe.sh.
Binary SHA2561b835422d3a4ee624e231625e4a05a155644aec2a330390da97ff550d46e027f.

Telemetry defaults off in the binary; DUST_RTT_CYCLE_LOG enables it. Separate
wrapper fourth parameter on(default)/off selects logging with the same binary.
Before activation reserve at most35s/period+2 records. Only command and pending
receive iterations are stored; two additional host timestamps surround send
when enabled. No cyclic file IO/allocation; overflow throws rather than silently
dropping records. Write after stop send/end, or attempt partial dump on failure.
Partial failed runs must not be interpreted by the successful-run decoder.

Cycle CSV: iteration is host loop/exchange number; command_batch associates a
request. In the command-send iteration, receive fields describe the prior
image. Last row of final request omits normal send timestamps (zeros); stop
send is intentionally not a normal measurement send. previous_send_ns is the
previous iteration's existing last_pdo_send stamp, not NIC egress. publication,
feedback_batch and accepted are the observed TxPDO counters; they are not wire
timestamps. wake_lateness_us now records every logged iteration, including the
response (old sample CSV wake belongs to command iteration). No extra send or
receive call, no frame insertion, no PDO changes.

Build: NUC g++ -O2 -std=c++20 -Wall -Wextra -Werror, existing IgH/yaml-cpp.
C++20 is required by shared YAML validation. Native telemetry contracts PASS:
disabled mode no allocation/log, stable preallocation, overflow error, CSV
precision, final no-normal-send record, IO error. Python decoder4 tests PASS:
irregular intervals, missing cycle, wrong response timestamp, duplicate cycle.
Wrapper bash -n and git diff --check PASS. First live test phase-on-01 PASS; see RESULTS.md and raw/decoded CSV.

First user-run command on NUC:
sudo bash ~/dust-ecat-bench/ethercat_rtt_phase_probe.sh 125 1000 phase-on-01 on

Then collect sample CSV, cycle CSV and log; validate PASS/timeouts/WKC and decode:
python3 tools/ethercat_cycle_decode.py SAMPLES.csv CYCLES.csv DECODED.csv

Plan a same-binary off run and repeat/interleave on/off if overhead matters.
Compare actual iteration deltas and response wake/send/receive intervals.
This isolates host observation behavior only. It cannot prove an ESC window was
missed or that a particular publication was on the wire before a boundary.
Master Idle/inactive and LAB PREOP confirmed before handing off first test.
