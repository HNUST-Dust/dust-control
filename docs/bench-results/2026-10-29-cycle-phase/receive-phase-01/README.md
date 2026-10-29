# Controlled receive phase experiment prepared

Firmware A trace-off / Rx96 Tx400 /125us. No PDO field changes, no ESC clock
or slave scheduling changes. Fresh saved A flash before first test, matching
ELF/HEX build/bench-host/abba-off-A. Original host binary and previous telemetry
binary retained. New binary dust_ethercat_dm_rtt_receive_phase, SHA256:
fbcd9c4062e0cf681bbb9c8c20803a0ddad5d1fa28a81eca6fc6e43063dc8120.

Default without DUST_RTT_RECEIVE_OFFSET_US retains original receive-then-send
loop. Optional mode accepts exactly30/60/90us at125us period and requires
bounded cycle telemetry. Each iteration receives the previous iteration's
frame, processes it and prepares the next output, then waits for its ABSOLUTE
send slot. After normal send, wait until that slot+offset and begin the next
receive iteration. Absolute send slots advance125us; early receive does not
advance the next normal send. Before arming, a heavily late slot may be
rebased for startup; never rebase in armed measurement. No extra normal sends
or receive calls are added. Exception-only original WKC diagnostic polling is
retained; any incomplete armed WKC still fails rather than silently retrying.

Normal sends use the original command timeout/pairing, one in-flight CAN3 ID3
disable request,100Hz minimum command separation and stop frame on successful
completion. The phase shift changes when command preparation is eligible,
so verify actual command intervals as well as PDO cadence. At final successful
receive the stop frame may be sent immediately, outside normal send slots;
exclude it from cadence/performance metrics, as in the original tool.

Fail the phase experiment if armed send lateness >20us or send/processing has
already overrun the chosen receive target. This20us is an explicit lab abort
threshold, not a real-time guarantee; raw actual sends/slots must still be
examined.30us may be too early for the496B PDO exchange at100Mbps; failure there
would reject that phase setting, not justify relaxing WKC. Start90us, then60,
then30 only after assessing raw phase/cadence data. No offset accepted as safe
or faster before measurement. Actual receive offset must be calculated from
receive_enter minus previous actual send stamp; nominal offset is anchored to
scheduled send slot and differs by actual send lateness.

Cycle records append send_schedule_ns/send_lateness_us. These refer to the
normal send AFTER that row's receive, not the packet being received. The final
successful row has no normal send; previous_send_ns refers to the previous
actual last_pdo_send stamp. wake_lateness_us now refers to the scheduled receive
target, not the send boundary. Existing sample CSV schema retained; no NIC or
ESC arrival timestamp is implied. Decoder joins exact host timestamps and
iteration indices, not nearest-period rounding (invalid for shifted phase).

New startup failure summary prints OP/complete-cycle counts, last AL, armed,
command count and boot when total35s deadline expires. This improves diagnosis
of the previous no-sample failure without cyclic printing.

Validation: NUC native C++20 -O2 -Wall -Wextra -Werror build PASS. Updated
telemetry contracts PASS including strict offset/period validation, disabled
storage, preallocation, overflow, final row and file errors. Existing4 Python
cycle decoder tests PASS. Wrapper syntax and git diff --check PASS. These do
not replace hardware validation of schedule/WKC behavior.

First command on NUC:
sudo bash ~/dust-ecat-bench/ethercat_rtt_receive_phase_probe.sh 125 1000 receive-90-01 90

Collect raw sample/cycle CSV and log; confirm PASS first. Verify planned slots
advance125us per iteration, actual send lateness/intervals, actual receive
offsets, response index delta and shifted RTT distribution. Any failure is
archived without combining partial samples into successful statistics. Fresh
A boot before each later phase/control test. No driver reload/capture required.

Fresh A flash completed, verified173716 bytes; slave LAB PREOP, master
Idle/inactive before handing off receive90 test. No live phase result yet.
