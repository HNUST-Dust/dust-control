# phase-on-01 results

PASS1000 commands, zero timeouts/WKC misses.7299 cycle records joined exactly;
no missing/duplicate pending iteration, all response timestamps match sample
CSV. Firmware unchanged A trace-off, Rx96/Tx400,125us. FIFO40 CPU7 mlock1.
ASPM restored by wrapper. Master Idle/inactive, LAB PREOP after run.

RTT mean786.362us, median749.050us, P95874.355us, max1124.434us.
Actual response-minus-command iteration counts:5:47,6:629,7:303,8:20,9:1.
These are measured loop indices, not rounded RTT guesses.

Response wake lateness mean0.059us, P950.078us, max1.123us. Receive call
mean0.875us, P951.399us, max2.525us. Last stale poll to response poll
mean124.997us, P95125.031us, max126.351us. The one9-cycle response has
wake lateness0.043us and receive call1.178us, so its extra latency is not
explained by a delayed final response wake. See full summary for all pending
iteration wake maximum and per-band distributions.

Previous-send stamp to response receive-return mean124.088us. This includes
send/NIC/frame transport/intentional periodic polling; it is NOT NIC residence
time. No passive packet timestamps exist in this run. Do not claim that the
packet was already waiting in the NIC for124us, or infer a recoverable125us.

Last stale versus response feedback batch advances by1 in all1000 requests.
Publication advances by1 for916 and2 for84. This shows fresh input versions
at the next observation, not physical ESC commit/packet sampling timestamps.

Compared with prior A trace-off means784.611/787.091us, this run mean786.362us
is in the same range, but inter-run similarity alone does not validate zero
telemetry overhead. Next measure same new binary with logging off; preserve
firmware, mapping, scheduler and send cadence. No firmware flash needed.

Conclusion: cycle quantization confirmed; no whole-cycle final-wake stall.
ESC visibility/frame traversal versus NIC arrival/application observation
remains unresolved. To separate them, need passive host-clock packet timestamps
and exact packet/publication matching; software timestamps must be labeled as
such. Direct ESC-window phase still requires a justified shared clock mapping.
