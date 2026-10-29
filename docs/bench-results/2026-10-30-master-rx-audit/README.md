# Native master RX timestamp feasibility audit

2026-10-30. Read-only inspection. Firmware remains the selected FreeRun B image,
reset after restoring it; no benchmark, reset, driver replacement, timestamp
filter change or receive-phase modification during this audit.

Installed: kernel5.15.0-1114-realtime; native ec_igc I225-LM PCI0000:58:00.0.
Module SHA256 c20c57c959512a154c06cd2ba39e7528685250fc3834044d8119c9848b395945,
identical to earlier capability audit. Master Idle/inactive, linkUP. No normal
netdev for this PCI device (only lo/wlo1); /dev/ptp0 exists, root-only.
No matching igc source found under /home/wbr-infantry or /opt. Upstream v5.15
source is a reference, not proof of the exact installed build's source revision.

## Evidence

1. Existing user timestamps measure receive call/processing and observation,
not physical NIC arrival. Native ec_poll synchronously cleans RX descriptors
when IgH polls. Extra software timestamps in ecdev_receive observe the poll,
not the earlier DMA completion. Debug capture cannot resolve this distinction.
2. Installed symbols include igc_ptp_rx_pktstamp, igc_ptp_set_ts_config and
igc_ptp_get_ts_config; PHC existence does not establish enabled EtherCAT stamps.
3. Installed igc_clean_rx_irq disassembly is archived. At3a52 it checks TSIP;
a true branch3c39 strips16 bytes from a size register and calls the timestamp
parser at3c57. The native EtherCAT branch at3a73 loads ecdev; at3a87..3aaa it
reconstructs the raw page+page_offset pointer, reads the original descriptor
length at3a9a and calls ecdev_receive at3aad. It does not apply the16-byte packet
offset/adjusted length in those arguments. This is a concrete compatibility
concern if timestamps are enabled; no runtime timestamp test was performed.
4. ecdev_receive API carries only device/data/size, with no per-frame timestamp.
Even a correctly stripped hardware-stamped packet would lose its timestamp
before reaching userspace unless a separate diagnostic path exports it.
5. Upstream Linuxv5.15 igc_ptp.c supports HWTSTAMP_FILTER_ALL. RX timestamps use
a16-byte prefix with timer1 and timer0 seconds/nanoseconds; the parser uses
timer0 and applies link-speed RX latency adjustment. Do not assume installed
native EtherCAT support merely because the upstream normal-netdev path has it.

## Proposed measurement, before changing scheduling

Use an isolated diagnostic driver build against source matched to the installed
IgH/kernel modules, with an off-by-default option and saved rollback modules.
First handle stamped prefix/length correctly in the native ecdev path. Preserve
normal frame parsing and all WKC checks. Export bounded in-memory records after
measurement, with no cyclic printk, allocations or packet-copy capture.
Record per matched EtherCAT datagram/frame: hardware RX stamp, host monotonic
poll entry, ecdev delivery, application receive/process/consume, and frame or
datagram identifier plus TxPDO publication and feedback generation. Define how
multiple datagrams/frames map to one response, including index reuse/timeouts.
Calibrate PHC(timer0) against the host monotonic clock with bracketed samples;
report conversion uncertainty/drift, lost stamps and ring overflow. Never
subtract PHC time directly from host CLOCK_MONOTONIC.

Then distinguish:
- NIC hardware receive -> driver polling: possible observation waiting.
- polling -> userspace consume: driver/master/application work.
- no fresh feedback in the already-returned frame: remaining slave/CAN path.

Keep125us normal send cadence and one receive/process/send sequence unchanged.
Validate instrumentation-off equivalence before comparing stamped vs unstamped
runs. Each independent CAN3-only2000-sample session requires gateway reset
because Stop leaves Core Protected. This route needs kernel-driver adaptation;
there is no valid pure-userspace switch that supplies the missing measurement.
No new sudo benchmark command is ready from this read-only audit.

Sources:
- https://docs.etherlab.org/ethercat/1.7/doxygen/group__DeviceInterface.html
- https://github.com/torvalds/linux/blob/v5.15/drivers/net/ethernet/intel/igc/igc_main.c
- https://github.com/torvalds/linux/blob/v5.15/drivers/net/ethernet/intel/igc/igc_ptp.c
- https://docs.kernel.org/networking/timestamping.html
