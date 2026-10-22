# Generic dust-adapter v2 integration

Paired firmware: HNUST-Dust/dust-adapter, branch feature/generic-usb-adapter.
Canonical wire header is include/dust_adapter/protocol.hpp; client packet builders
are include/dust_adapter/client.hpp, copied verbatim from the firmware repository.
The small wbr_bridge wrapper holds robot application aliases/helpers only.

## Responsibility split

The MCU has no motor map or robot stop bytes. MotorCommands coalesces per-motor
writes within a host control cycle, then emits CAN records in left W/B/D, right
W/B/D order. SensorDecoder routes raw standard-data CAN IDs using the startup
configuration; unrelated IDs/extended/RTR records do not enter motor decoders.
SBUS/HI91 framing, transforms, freshness and explicit user output-enable remain
unchanged in the host.

USB Open preposts four IN transfers, negotiates v2 capabilities/channels, registers
the application lease (20ms) and six raw stop actions, waits for acknowledgement,
then explicitly arms and waits for acknowledgement. Wheel stop payloads are zero
current; joint stop payloads are DM Exit. Adapter Arm is not motor Enter. Failed
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
