# Cycle telemetry on/off comparison

Same new host binary, FIFO40 CPU7/125us/Rx96/Tx400, A trace-off firmware.
On01 and off02 each1000 PASS, zero timeout/WKC misses. Off01 failed startup
with zero samples and is archived separately, excluded from timing comparison.
Off02 succeeded after verified fresh A flash. No proof that boot reset alone
caused recovery; no in-run startup counters exist for the failed attempt.

| Mode | Mean us | Median us | P95 us | P99 us | Maximum us |
| --- | ---: | ---: | ---: | ---: | ---: |
| On01 | 786.362 | 749.0505 | 874.355 | 999.036 | 1124.434 |
| Off02 | 792.241 | 749.125 | 874.420 | 999.036 | 1124.024 |

On mean is5.879us lower, not higher. This is not negative overhead: one run
per mode, different startup history and changing band populations preclude
quantifying overhead. No obvious RTT inflation is observed. Median/P95/P99
are similar, consistent with using on01 telemetry for qualitative chronology.
Smaller per-cycle perturbation remains possible despite quantized host RTT.
No performance improvement is claimed for instrumentation or firmware.

Phase evidence retained: actual5..9 loop deltas, all logged wake max4.645us,
response receive max2.525us; no whole-cycle final-wake stall observed. New
feedback appears one software poll after the last stale image. Exact NIC
arrival and ESC sampling/commit phase remain unknown. Host-only receive-entry
logging cannot prove a missed ESC window or identify recoverable125us.

Next useful measurement: passive packet timing tied to the host clock, matching
EtherCAT datagram/publication to cycle records. Verify ec_igc capture support
first; a conventional tcpdump on a driver-owned EtherCAT NIC may not see traffic.
Use a short low-rate packet-capture capability probe before a full benchmark;
label software/hardware timestamps accurately. If unsupported, use IgH debug
capture/export or an external tap with an established clock mapping. Preserve
125us send cadence and current PDO layout. No new packet capture or benchmark
started in this collection turn.

Current board remains A, LAB PREOP/master Idle inactive, no benchmark process.
ASPM restored1/1/1. Raw CSV/log and structured summary archived.
