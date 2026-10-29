# Cycle phase review — 2026-10-29

Read-only/offline analysis; no flash, PDO layout change, firmware or master
scheduling change. Board remains trace-off A baseline. Reproduce with
`python3 docs/bench-results/2026-10-29-cycle-phase/analyze.py`.

## What the 125 us bands actually establish

All four trace-off runs use125us periods and1000 samples. The nominal band is
nearest(RTT/125), not a measured cycle index or inferred missed deadline.

| Run | ~624us (5 periods) | ~749us (6) | ~874us (7) | ~999us (8) |
| --- | ---: | ---: | ---: | ---: |
| A1 | 45 | 646 | 288 | 21 |
| B1 | 33 | 625 | 328 | 14 |
| B2 | 82 | 653 | 256 | 9 |
| A2 | 40 | 638 | 299 | 23 |

Mean residual RTT - nearest_period*125 is approximately -1.01 to-1.07us.
The identical B image changes its6/7-period mix between B1 and B2 without
materially changing RTT median or P95. This explains how a several-us mean
change can result from a changed band population rather than every transfer
becoming several-us faster. It does not establish why the population changed.

## Master chronology

Source tools/ethercat_dm_rtt.cpp main loop:
1. Wake at next cycle boundary; record wake lateness; advance deadline125us.
2. ecrt_master_receive; timestamp receive-return; ecrt_domain_process.
3. Copy TxPDO and parse ACK and feedback. RTT end is receive-return, not
   parsing completion, NIC ingress or the moment the ESC sampled its RAM.
4. Build any new command; update exchange_sequence every cycle; copy RxPDO.
5. Queue and send; command RTT start is last_pdo_send immediately before the
   command's send call. next_send=sent+10ms; next command still starts only
   on a cycle iteration, so there is no deliberate command phase sweep.
6. Sleep/spin until next boundary.

Thus command start and feedback observation are both tied to the same periodic
loop. Quantized RTT is expected even if underlying physical latency varies
continuously. The roughly-1us residual is consistent with different placement
of transmit and receive timestamps within their iterations, not a measured
negative physical delay. Wake_lateness_us in the CSV belongs to the COMMAND
iteration; it does not record wake lateness for the response iteration.

Crucially, three different events must be distinguished:
- outbound EtherCAT datagram samples ESC input RAM while traversing the slave;
- returned datagram reaches the NIC;
- application calls receive/process and observes it.
Moving only the application receive call can change observation delay without
changing which ESC publication was already sampled. It cannot retroactively
put a late SSC publication into that packet. A further send is a different
bus-traffic experiment and changes RxPDO rate/SSC workload.

## Device-side association, not synchronized phase measurement

Existing matched trace-on baseline64 records show:

| Nominal RTT band | n | MCU Rx mapping->CAN RX mean us | CAN RX->ESC end mean us | Rx mapping->ESC end mean us |
| --- | ---: | ---: | ---: | ---: |
| 6 | 28 | 324.98 | 171.22 | 496.21 |
| 7 | 33 | 328.04 | 186.26 | 514.30 |
| 8 | 2 | 352.19 | 259.90 | 612.08 |
| 9 | 1 | 456.50 | 285.29 | 741.79 |

Groups6 and7 differ by roughly18us in the measured device pipeline while
host RTT bands differ by125us. This is consistent with periodic gating
amplifying a modest timing shift. It is not proof of a missed ESC sampling
window: command transport before Rx mapping, return transport, FreeRun phase,
software observation and other scheduling can also differ. Within-group
variation is substantial;8/9-band groups are too small for broad conclusions.
Trace-on images are not timings of the trace-off ABBA runs.

No host timestamp was subtracted from an MCU timestamp. MCU-derived intervals
use their24MHz clock; host intervals use the host monotonic clock. Absolute
MCU cycles modulo125us do NOT identify phase against the master: clock origins
are unrelated and clocks may drift. ESC write completion is also not a packet
sampling timestamp; logical/physical buffered SyncManager visibility matters.

## Next controlled measurement

First add bounded optional host-only cycle telemetry, default off, with logs
written after the run: command and response iteration indices, response wake
lateness, each pending iteration's send enter/return and receive enter/return,
WKC, observed publication/CAN batch and accepted command sequence. Include the
last stale-input iteration before the matching feedback. Preallocate storage;
fail clearly on overflow, preserve the old pairing/fault/timeout checks. Compare
instrumentation-on/off overhead; do not make the heavy DTR trace a requirement.
This separates a lost application observation cycle from a slower MCU pipeline
and makes the nominal band count an actual iteration difference. It still
cannot independently timestamp the NIC or ESC sampling event.

Next use passive packet timestamps tied to the host clock (verify ingress/
egress timestamp type and kernel/NIC availability) to bound send-to-wire,
wire-return and return-to-application intervals. A packet capture's software
timestamp is not hardware wire time. Match packets/publication identifiers;
keep master125us send cadence and full400B layout unchanged.

For direct ESC-window evidence, require a justified clock mapping (DC/common
reference or external synchronous observation) between publication commit and
frame traversal. No causal missed-window claim from two unsynchronized clocks.
Then sweep publication/command timing relative to the actual send schedule in
a separate bounded lab mode, not merely delay the receive call. Keep stop,
watchdog and regular service intact; compare repeated phase settings in
interleaved order. No DC-mode switch or scheduling modification made here.

Decision: investigate phase gating further; quantization is confirmed and
modest MCU pipeline shifts correlate with whole-cycle host changes. The number
of preventable missed windows, attainable gain and best phase are still
unmeasured. Do not infer one full125us is routinely recoverable.
