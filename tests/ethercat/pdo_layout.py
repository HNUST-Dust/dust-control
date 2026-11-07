"""Validate DTG1 mappings using the actual application configuration loader."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml

exe, source = sys.argv[1:]
baseline = yaml.safe_load(Path(source).read_text())
def run(document, valid=True):
    with tempfile.TemporaryDirectory() as directory:
        config = Path(directory) / "dust_control.yaml"
        config.write_text(yaml.safe_dump(document))
        result = subprocess.run([exe, str(config), "--json"], text=True, capture_output=True)
        assert result.returncode == (0 if valid else 1), result.stderr
        return json.loads(result.stdout) if valid else None
layout = run(baseline)
assert not layout["hardware_accessed"] and layout["cycle_time_us"] == 125
base = layout["adapters"]["base"]
assert base["layout_signature"] == 0x44544731
for side, size, obj, widths in (("rx", 60, 0x2050, [80]*6), ("tx", 180, 0x3051, [80]*6+[240]*4)):
    image = base[side]
    assert image["bytes"] == size and len(image["mappings"]) == 1
    entries = image["mappings"][0]["entries"]
    assert [entry["bit_length"] for entry in entries] == widths
    offset = 0
    for index, entry in enumerate(entries, 1):
        assert entry["object_index"] == obj and entry["subindex"] == index
        assert entry["offset"] == offset
        offset += entry["bit_length"] // 8
for period in (0, 250, 500, 1000):
    changed = copy.deepcopy(baseline)
    changed["ethercat"]["cycle_time_us"] = period
    run(changed, valid=False)
multiple = copy.deepcopy(baseline)
for index in (1, 2):
    board = copy.deepcopy(multiple["adapters"]["base"])
    board["position"] = index
    multiple["adapters"][f"board{index}"] = board
assert run(multiple)["total_tx_bytes"] == 540
multiple["adapters"]["board2"]["position"] = 1
run(multiple, valid=False)
limited = copy.deepcopy(baseline)
limited["ethercat"]["limits"]["max_txpdo_bytes"] = 179
run(limited, valid=False)
print("DTG1 mappings, fixed cycle, aggregate size and address validation passed")
