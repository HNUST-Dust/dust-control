# EtherCAT packet capture capability — 2026-10-29

Read-only inspection, no driver reload/rebuild, debug enable, packet capture,
firmware flash or new active benchmark. Master remains Idle/inactive.

Installed IgH1.7.0 unknown, kernel5.15.0-1114-realtime, native ec_igc I225-LM
PCI0000:58:00.0 MAC54:b2:03:9e:f2:fa. `ip -br link` lists only lo and wlo1;
no regular dedicated Ethernet interface and no ecdbg interface. `ethercat
--help` has no pcap command; `ethercat pcap --help` returns unknown command.
`nm ec_master.ko` exposes ec_device_poll but no ec_debug/debug_ring symbols.
Together these observations indicate the current build has no accessible debug
capture interface. Increasing debug_level is not a substitute for enabling a
compile-time debug interface and would add logging overhead.

Actual installed ec_igc ec_poll disassembly calls igc_clean_tx_irq and
igc_clean_rx_irq synchronously. IgH official1.7 ecrt_master_receive source
calls ec_device_poll, which calls the driver's poll callback. A receive-side
software mirror in this path observes driver poll/delivery time, not necessarily
the instant a frame arrived at NIC hardware. Therefore even adding debug-if
would not by itself resolve NIC residence time, and its packet copies and
allocations would perturb the experiment.

PTP clock ptp0 exists under the dedicated PCI device, clock_name54b2039ef2fa,
/dev/ptp0 root-only. igc contains PTP helper symbols. This establishes PHC/
driver code presence only: it does NOT establish that EtherCAT receive frames
are hardware stamped, that native ecdev_receive exports those stamps, that
all EtherCAT packet types can be timestamped, or a PHC-to-host-monotonic mapping.
Do not configure timestamp filters or write NIC registers from this evidence.

Module SHA256:
- ec_master:8de75d30018d05eab1612bd298cd7433e9b7d8a69c1504c6fe542385b1b59b7d
- ec_igc:c20c57c959512a154c06cd2ba39e7528685250fc3834044d8119c9848b395945

## Practical next choices

1. Minimal host-only diagnostic: independently schedule receive/process after
   each normal125us send at controlled offsets (for example30/60/90us), while
   preserving send cadence and exactly one normal PDO send. This is an active
   observation-phase experiment, NOT passive capture. It can bound when a frame
   is available to software and test avoidable observation waiting, but cannot
   yield exact hardware arrival/ESC phase. Verify IgH domain processing/WKC,
   command construction deadline, timeout/pairing and instrumentation overhead
   before implementation. Do not simply delay receive and let send drift.
2. Exact arrival route: establish native ec_igc RX hardware timestamp support
   and per-frame export plus clock mapping, or use a suitable external tap with
   synchronized timestamps. This is a separate driver/instrumentation project.
3. Debug-if rebuild could expose packet content/ordering, but changes the
   installed driver setup and cannot alone answer the hardware timing question.
   No production driver change is justified merely to get software timestamps.

Recommended sequence: controlled receive-phase experiment first; only pursue
hardware timestamps if exact wire/ESC phase is still needed. Current RTT bands
and wake evidence remain valid but no missed ESC window has been measured.

Primary references:
- IgH debug interface documentation (native drivers do not register normal
  network interfaces; debug-if is a build option):
  https://docs.etherlab.org/ethercat/1.5/pdf/ethercat_doc.pdf section7.5
- Actual1.7 receive path:
  https://docs.etherlab.org/ethercat/1.7/doxygen/master_8c_source.html
  ecrt_master_receive / ec_device_poll
  https://docs.etherlab.org/ethercat/1.7/doxygen/device_8c_source.html
- Build switches:
  https://docs.etherlab.org/ethercat/1.6/pdf/ethercat_doc.pdf section9

No sudo test is ready or required for this read-only capability result.
