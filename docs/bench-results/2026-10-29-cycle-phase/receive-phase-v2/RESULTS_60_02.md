# receive-60-02: runtime receive deadline failed

Unlike60-01, startup reached AL8/Active. First command batch1 was sent; no RTT
sample completed.8822 logged rows,8821 batch0 startup/idle and one batch1 row.
Only one normal phase send was recorded before failure. Do not run the
successful-run decoder or treat partial/no samples as latency results.

Log: initial_wc0, eventual_wc3, first_poll_after_send58.837us,
final_poll_after_send61.550us. Existing diagnostic re-polls without retrying
commands; it keeps the run failed. AL8/WC3 in the final exception describes
state AFTER diagnostic recovery; it does not make initial receive valid.

First phase send scheduled2047999423611ns, actual send-enter2047999425819ns:
2.208us late. Its nominal slot+60us receive target was thus57.792us after
actual send-enter. The first_poll_after_send diagnostic timestamp is taken
after the failed receive/process checks, not at exact poll entry. This mode
is slot-anchored, not precisely actual-send+60us. The result therefore does
not prove actual-send+60us is invariably too early; first-transition lateness
and sparse sampling matter. Full domain data was observed by diagnostic end,
not an independently measured hardware arrival timestamp.

The failure concerns completion of the whole PDO datagram/WKC, not whether the
new CAN feedback had been published by ESC: the request had only just been
sent and its CAN round trip was still pending. This is a transport/observation
budget constraint on placing the receive call, not evidence of a missed SSC
publication window or an ESC bandwidth limit.

Do not weaken WKC, adopt diagnostic polling as a successful60us result, or
proceed blindly to30us. Reconfirm90us with the SAME v2 binary and startup
policy before drawing matched comparisons. An exact actual-send-anchored60us
experiment could be designed later, with absolute next-send slots unchanged,
if this narrow boundary is worth pursuing. No such change made here.

After failure master Idle/inactive, LAB PREOP, no benchmark process; ASPM
restored1/1/1. Original raw log/cycle/sample files retained beside this report.
Next: fresh identical A image, receive-90-02 using unchanged v2 host binary.

Recovery A flash verified173716 bytes; LAB PREOP/master inactive confirmed.
Await receive-90-02 with same v2 binary; no30us run planned at present.
