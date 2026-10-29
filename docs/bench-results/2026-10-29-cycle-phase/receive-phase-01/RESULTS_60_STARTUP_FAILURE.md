# receive-60-01: startup failure, not an RTT result

35s deadline, operational_cycles0, complete_cycles0, last_AL2, armed0,
commands0, boot0. Both CSVs header only. No command sent, no RTT samples.
ASPM restored1/1/1; after test master Idle/inactive, LAB PREOP.
The failed mode applied60us to configuration/startup as well as measurement;
startup has different datagram/SDO timing. No causal root proven. This is not
a measured60us OP/WKC failure nor evidence about ESC commit phase.

Revise tool to retain original receive-then-send startup and transition only
on observed Active. Record configured-phase mode's startup/idle cycles too,
label no-command rows batch0, and ignore batch0 only when decoding successful
RTT samples. Keep strict OP/WKC/send deadline checks in actual measurement.
Rebuild and fresh A boot for receive-60-02.90us must eventually be repeated
with the same revised binary for comparable startup conditions.
