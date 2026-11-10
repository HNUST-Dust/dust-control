"""Check deterministic schema generation, propagation and invalid inputs."""
import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml

root = Path(sys.argv[1])
baseline = yaml.safe_load((root / "config/pdo_schema.yaml").read_text())
with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    schema = directory / "schema.yaml"
    output = directory / "schema.hpp"
    def generate(document, valid=True):
        schema.write_text(yaml.safe_dump(document))
        result = subprocess.run([sys.executable, str(root / "tools/generate_pdo_schema.py"),
            "--schema", str(schema), "--output", str(output)], capture_output=True, text=True)
        assert (result.returncode == 0) == valid, result.stderr
        return output.read_text() if valid else None
    first = generate(baseline)
    assert generate(baseline) == first
    changed = copy.deepcopy(baseline)
    changed["signature"] += 1
    assert generate(changed) != first
    for mutate in (
        lambda d: d.update(protocol="generic"),
        lambda d: d["tx"]["entries"].__setitem__(0, 7),
        lambda d: d["tx"]["entries"].pop(),
        lambda d: d["regions"]["hi91"].update(offset=170),
        lambda d: d["routes"].__setitem__(5, d["routes"][0]),
        lambda d: d["routes"][0].update(bus=0),
        lambda d: d["startup_values"].__setitem__(2, 3),
        lambda d: d["startup_values"].__setitem__(4, 0),
        lambda d: d.update(unrecognized=1),
    ):
        changed = copy.deepcopy(baseline)
        mutate(changed)
        generate(changed, valid=False)
print("Schema determinism, propagation and rejection checks passed")
