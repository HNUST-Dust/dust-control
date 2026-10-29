# TxPDO 400-byte audit

Offline audit only. Board remains ABBA A trace-off baseline; no flash or live
mapping changes performed. Rebuilt current dust_ethercat_layout target because
existing binary was stale and only accepted 1000 us. Actual 125 us configuration
reproduced Rx96/Tx400 signature3858229428, matching bench logs.

| Current TxPDO region | Offset | Bytes |
| --- | ---: | ---: |
| Header | 0 | 64 |
| CAN3 batch header | 64 | 16 |
| CAN3 four feedback slots | 80 | 96 |
| Diagnostics | 176 | 224 |
| Total | | 400 |

Each CAN feedback slot is24 bytes: id4, bus sequence4, capture timestamp4,
length1, flags1, reserved2, data8. Disabled CAN0..2, UART and IMU contribute
zero bytes. Fixed diagnostics224 = common32 + four CAN controllers128 + six
UART error blocks48 + status reply16. Diagnostics are always appended by both
host and firmware layout builders, even when corresponding ports are disabled.
They are structured protocol fields, not trailing alignment padding.

The dedicated bench config requests four CAN3 RX slots although it issues one
sequential CAN3 ID3 disable request at a time and selects feedback ID0x13.
The bench accepts any CAN3 RX slot count >=1. Changing only rx_slots[3] to1
produces Rx96/Tx328 signature2191791339: 72 bytes /18% less TxPDO, with no ABI
format change or firmware rebuild required. Existing configuration/signature,
SDO and dynamic mapping must be applied normally; never truncate the old400B
mapping or reuse its signature. Candidate config and full layout JSON saved.
This reduces available burst capacity: other unsolicited frames or multiple
responses before snapshot consumption can evict older frames and increment
drop counters. One slot is a dedicated sequential bench candidate, not a
production multi-motor recommendation. Actual traffic/drop behavior needs board
measurement; offline compilation does not prove one-slot retention adequate.

At a hypothetical linear50 us/400B write cost, saving72B would save about9 us
per full write; this is a scaling estimate, not a measurement or an RTT promise.
Ethernet payload serialization saves5.76 us for72B at100Mbps, before framing,
padding, scheduling and cycle effects; not additive to an established RTT gain.

Further compression is a protocol project. Header64 + CAN header16 + one
feedback24 =104B excluding diagnostics, versus current328B minimum with one
CAN slot and full diagnostics. Removing224B wholesale would also remove fault,
status-query and diagnostic information. Host session.cpp copies224B and reads
status replies at+208; firmware Core::Publish writes the same layout. Current
builders unconditionally include diagnostics. A compact/optional diagnostics
profile must update both ends, mapping/signature/version compatibility and
status/fault handling. A theoretical104B profile is not currently supported.

Sources: host src/hardware/ethercat/pdo_config.cpp TxHeader/CanFeedback/
Diagnostics/BuildPdoLayout; host session.cpp diagnostic copy/status reply;
firmware src/ethercat/layout.cpp Header/Can/Diagnostics/Image;
firmware core.cpp Publish; io_backend.cpp DrainCanReceive bounded eviction;
tools/ethercat_dm_rtt.cpp config checks and sequential feedback matching.

Recommended next experiment: same A trace-off firmware, compare existing400B
and candidate328B YAML under identical125us/1000-probe ABBA conditions, checking
WKC, timeout and CAN drop counters before accepting any gain. Keep the optional
late-refresh/fast-notice features off to isolate mapping size.
