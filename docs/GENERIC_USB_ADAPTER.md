# Generic dust-adapter v2 integration

Paired firmware: HNUST-Dust/dust-adapter, branch feature/generic-usb-adapter.
Canonical wire header is include/dust_adapter/protocol.hpp; client packet builders
are include/dust_adapter/client.hpp, copied verbatim from the firmware repository.
The small wbr_bridge wrapper holds robot application aliases/helpers only.

## Responsibility split

The MCU compiles its CAN device IDs and default protection bytes from a configuration file,
but does not decode motor payloads or run robot control logic. MotorCommands coalesces per-motor
writes within a host control cycle, then emits CAN records in left W/B/D, right
W/B/D order. SensorDecoder routes raw standard-data CAN IDs using the startup
configuration; unrelated IDs/extended/RTR records do not enter motor decoders.
SBUS/HI91 framing, transforms, freshness and explicit user output-enable remain
unchanged in the host.

USB Open preposts four IN transfers, negotiates v2 capabilities/channels, verifies
the MCU protection signature and selects its defaults with a lease of at most20ms,
waits for acknowledgement,
then explicitly arms and waits for acknowledgement. The host expects wheel protection payloads to be zero
current and joint protection payloads to be DM Exit, and refuses a mismatching MCU profile. Adapter Arm is not motor Enter. Failed
configuration/arm or a data rejection fails the link. No automatic rearm occurs.
The control loop remains the sole business command producer; its failure must not
be concealed by an independent heartbeat thread.

All OUT transfers are now128 bytes, including Hello and Stop. IN remains64 bytes.
CAN IN records are24 bytes, with up to2 per transfer. All commands are raw16-byte
records, max6 per transfer, without motor slot metadata. No periodic success ACK.
Version1 is rejected, rather than guessed or silently accepted.

## Offline validation

```bash
cmake -S . -B build/generic-adapter -G Ninja -DBUILD_TESTING=ON \
  -DDUST_ENABLE_FOXGLOVE=OFF -DDUST_BUILD_MPC=OFF -DDUST_BUILD_TOOLS=OFF
cmake --build build/generic-adapter --parallel 2
ctest --test-dir build/generic-adapter --output-on-failure
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
signature over the serialized protection table. The host compares this with the same YAML-generated device profile before Arm,
and checks that runtime motor command/feedback IDs are declared in that profile. An absent or
mismatched profile fails Open. ConfigureSafety count0 selects the firmware table;
it cannot delete protection. The host may choose a shorter lease, not a longer one.
Both host and firmware must be updated when the CAN topology changes. The signature
checks protection-table identity, not motor execution, firmware authenticity or the
entire feedback routing configuration.

The configuration file and generator are shared in format/content with dust-adapter,
including the default protection payloads. UsbLink no longer reconstructs protection
bytes in C++; both builds can explicitly point to the same YAML with
`DUST_ADAPTER_CAN_CONFIG`. Configuration remains compile-time and outside RT loops.
