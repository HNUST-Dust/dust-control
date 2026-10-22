# wbr_infantry USB release package

Target: Linux x86_64, built on Ubuntu 22.04. This is the host USB controller,
not MCU firmware. Install runtime dependencies on the target host:

```bash
sudo apt-get install libusb-1.0-0 libyaml-cpp0.7 libstdc++6
```

Verify the downloaded archive with `sha256sum -c SHA256SUMS`, then extract it.
`BUILD_INFO.txt` identifies the robot, transport, version and exact source commit.

```bash
# Offline check: no USB device or motors opened.
./bin/chassis_controller --config share/dust-control/config --simulate --no-rt --seconds 1
# Real USB input, default actuator output disabled:
sudo ./bin/chassis_controller --config share/dust-control/config --foxglove
```

Only the onsite operator should add `--enable-output` after setup is ready.
Startup disabled-pose confirmation and all existing safety gates remain required.
For realtime operation set the host's locked-memory limit appropriately (for
example `sudo bash -c 'ulimit -l unlimited; exec ./bin/chassis_controller --config share/dust-control/config --foxglove'`).
Foxglove listens on loopback 8765; connect via SSH forwarding when remote.
The importable Plot layout is `share/dust-control/foxglove/debug-layout.json`.

This package does not replace MCU firmware or change gains/signs/calibration.
MPC and EtherCAT development applications are excluded. Refactored/released
builds still need timing verification on the actual robot host.
