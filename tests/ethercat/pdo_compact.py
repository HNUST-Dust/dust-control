"""Current board wiring uses only CAN2/3, UART2/5, and onboard IMU."""
import json
import subprocess
import sys
# The production profile is generated from config/pdo_schema.yaml; the layout
# tool exposes it via --production (no separate PDO config file).
result = subprocess.run([sys.argv[1], "--production", "--json"], text=True, capture_output=True, check=True)
layout = json.loads(result.stdout)["adapters"]["base"]
assert layout["rx"]["bytes"] == 131
assert layout["tx"]["bytes"] == 410
names = {entry["name"] for direction in ("rx", "tx") for mapping in layout[direction]["mappings"] for entry in mapping["entries"]}
assert "can2.batch_sequence" in names and "can3.batch_sequence" in names
assert "can0.batch_sequence" not in names and "can1.batch_sequence" not in names
assert "uart2.block_sequence" in names and "uart5.block_sequence" in names
for port in (1, 3, 4, 6):
    assert f"uart{port}.block_sequence" not in names
assert "imu.sample_sequence" in names
