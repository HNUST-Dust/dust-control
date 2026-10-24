# Generic dust-adapter USB v4 integration

Paired firmware: HNUST-Dust/dust-adapter, branch feature/generic-usb-adapter.
Host USB messages live in src/hardware/usb/protocol/. usb_protocol.hpp,
usb_protocol_codec.hpp and usb_session_policy.hpp are byte-for-byte copies from
MCU commit 28fa998. Named messages use explicit Encode/Decode; packet buffers are
UsbOutRequest and UsbInPacket, not memory images of structs. The old client.hpp
and Packet/Request/Record API have been removed.

MCU only encodes reports. usb_report_decoder.hpp supplies host-side Decode for
Ready, Result, CAN, UART and IMU reports; it does not alter the shared files.
MotorEncoder returns transport-independent CAN frames in W/B/D order. UsbLink
builds CanTransmitRequest and owns session and sequence assignment.
The small bridge_protocol.hpp wrapper holds application aliases
and the host lease limit only. Check shared sources before coordinated updates:

```bash
python3 tools/check_usb_protocol.py /path/to/dust-adapter
```

## Responsibility split

The MCU compiles its CAN device IDs and default protection bytes from a configuration file,
but does not decode motor payloads or run robot control logic. The controller returns
one target per motor per cycle. MotorEncoder converts those targets directly to CAN
frames in left W/B/D, right W/B/D order; UsbTransmitter schedules the latest batch
and UsbLink encodes the USB request.
UsbLink validates and dispatches reports directly to independent
device decoders. MotorFeedbackDecoder routes standard-data CAN IDs using the
startup configuration; unrelated IDs/extended/RTR records do not enter motor decoding.
SBUS/HI91 framing, transforms, freshness and explicit user output-enable remain
unchanged in the host.

USB Open preposts four IN transfers, checks firmware protection capability and
required CAN channels, then selects firmware defaults with a lease of at most 20 ms.
It waits for acknowledgement before explicitly arming and awaiting acknowledgement.
Adapter Arm is not motor Enter. Failed configuration/arm or a data rejection fails
the link. No automatic rearm occurs.
Business threads submit encoded CAN batches or bounded UART messages to a single
USB transmit thread. Each registered CAN producer owns complete (bus, command_id)
groups and has its own deadline. Any expired CAN producer stops the entire session;
other traffic cannot mask its failure. See [architecture](ARCHITECTURE.md).

All OUT transfers are 128 bytes, including Hello and Stop. CAN IN carries 1..6
24-byte records, for an actual length of 16 + 24*N (40..160 bytes). Other IN
reports are 64 bytes; preposted IN buffers are 160 bytes. CAN command records
remain 16 bytes with up to six per request, without motor slot metadata. No
periodic success ACK. Older wire versions are rejected without fallback.

## Offline validation

```bash
cmake -S . -B build -DBUILD_TESTING=ON \
  -DDUST_ENABLE_FOXGLOVE=OFF -DDUST_BUILD_MPC=OFF -DDUST_BUILD_TOOLS=OFF
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

The USB mock checks safety payloads and refuses business commands before explicit
Arm. It covers safety/arm/data rejection, old firmware, preposted IN, reconnect,
and normal sensor/control lifecycle. Native shared wire contracts cover bounds,
replay, lease expiry and sequence/timestamp wrap. These are not hardware tests.

Deploy the two feature branches together only after stop-only, physical disconnect,
UART TX loopback and aggregate-load/latency acceptance. Do not infer new CAN RTT or
physical stop guarantees from old firmware results. Nothing is auto-flashed or
started on the robot by this migration.

## MCU-owned configured CAN protection

The firmware now compiles config/can_devices.yaml into per-bus feedback-ID filters,
command-ID allowlists and default protection frames. It applies these on disconnect,
lease expiry, explicit Stop and (by default) MCU startup, without needing a host upload.
It still does not decode motor payloads or execute the chassis controller.

Ready advertises firmware-protection capability, count, default lease and a CRC32
signature. The host requires protection capability and selects the firmware table
with ConfigureSafety count 0; it does not compare protection-table signatures or
keep a local copy of the MCU device profile. The host may choose a shorter lease,
not a longer one. Host motor addresses come from motors_params.yaml (or built-in
defaults); firmware independently enforces its CAN allowlist. Topology changes must
be reflected in host motor parameters and MCU configuration. Rejected CAN requests
fail the host link. Protection bytes remain entirely owned by the MCU.

## USB identity and board selection (2026-10-23)

Production discovery enumerates VID `0x34b7`, requires manufacturer `Dust` and
product `dust-hpm6750`, then validates high speed, vendor interface 0 / alternate 0
and Bulk endpoints OUT `0x01`, IN `0x81`, MPS 512. The USB v4 Hello/Ready,
protection capability and Arm checks still apply. No wire formats changed.

PID is CRC16-CCITT-FALSE of the full MCU UID, including possible values zero and
`0xffff`. PID collisions are possible; PID is not a unique board identifier.
The serial is the complete uppercase UID hex string (32 characters on HPM6750).
Select a board with `--usb-serial FULL_UID` (exact, case-sensitive; no prefixes).
`--usb-product NAME` supports other boards. Optional `--usb-pid 0x1234` is an
additional exact filter; by default all PIDs are eligible. Multiple matches fail
and print their identities rather than selecting the first. Selected device logs
include VID, PID, manufacturer, product and full serial. Unreadable candidates
fail discovery with bus/address and libusb errors; selection does not silently
ignore a potentially matching inaccessible board.

Old PID `0xffff` remains eligible only with matching identity, interface and v4
protocol. No legacy protocol fallback is provided. The standalone historical
stress/echo programs under tests/benchmarks use their own firmware protocols and
fixed identities; they are not USB v4 acceptance tools.

### Linux permissions

Run the controller with sudo for USB access. For boot startup, run it as a root
systemd service; ExecStart invokes the executable directly without sudo.

Offline identity regression covers dynamic/zero/old PID, colliding PIDs with
separate serials, full serial selection, multiple/absent candidates, permission
and string failures, product selection and vendor-interface rejection. Hardware
validation remains pending: flash boards, inspect descriptors, verify stable PID
and serial across reboot, select each of at least two boards, and verify USB
access and the full session under the actual NUC service account. Any Arm/motor-output validation
must follow the existing hardware acceptance authorization.
