# receive-90-01 result

1000 PASS, zero timeouts/WKC misses;7231 cycle rows exactly joined to samples.
Trace-off A / Rx96 Tx400 /125us, same dedicated CAN3 ID3 disable contract.
Master Idle/inactive, LAB PREOP after test; ASPM restored1/1/1.

Planned send grid: no violations across all logged send slots (including gaps
between request windows). Contiguous actual send intervals n6230, mean124.997us,
min123.524us/max126.449us. Actual send-entry minus planned slot max1.554us.
Receive-entry minus previous actual send stamp mean90.030us, P9590.063us,
max90.812us. This supports unchanged125us cadence within recorded windows;
idle/unrecorded sends and physical NIC egress are not directly measured.

RTT mean744.876us, median716.050us, P95841.084us, max966.451us.
Actual loop deltas5:68,6:652,7:261,8:19. Do not round shifted RTT to125us
multiples. For this scheduling mode a k-iteration response occurs roughly
(k-1)*125+90us after command send, plus receive-call and timestamp offsets.

Prior original receive phase: on01 mean786.362us, median749.0505us,
P95874.355us. New median shifts-33.0005us, P95 shifts-33.271us, consistent
with software observation moving from about124us after previous send to90us.
Mean differs-41.486us, but population across cycle bands also changes; not all
of that difference is a proven fixed per-request improvement. One run per
setting does not establish repeatable effect size or hardware timing changes.

Confound: command interval is now consistently10125us (mean10124.999992us),
versus prior telemetry-on mean10092.968us and telemetry-off mean10101.099us.
Command eligibility was checked during earlier receive processing, so it can
wait one more125us send slot. Still at least10ms and strictly sequential, but
not exactly the same command cadence.90/60/30 phase settings must report this
and use matched-cadence controls before an acceptance claim. No PDO rate change.

Conclusion: earlier software observation is feasible at90us under these test
conditions and likely contributes the median shift. No proof that ESC commit
advanced, frame arrived at a particular hardware time, or full125us is
recoverable. Next60us with same binary/fresh A boot; fail rather than relax
WKC or shift normal sends if60us is too early. Keep raw results separate.

Next60us preparation: identical A flash verified173716 bytes; LAB PREOP,
master Idle/inactive. Host binary unchanged. Await receive-60-01.
