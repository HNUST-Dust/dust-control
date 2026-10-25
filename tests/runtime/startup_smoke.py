"""Execution settings come only from YAML, before any hardware is opened."""
from pathlib import Path
import subprocess
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from startup_support import configuration, node_command, run_for

binary = sys.argv[1]
with configuration() as config:
    result = run_for(node_command(binary, config))
    assert result.returncode == 0, (result.stdout, result.stderr)
    assert "offline" in result.stdout and "output=disabled" in result.stdout
    assert "Stopped;" in result.stdout
    document = yaml.safe_load((config / "robot.yaml").read_text())
    for execution in (
        {"simulate": True, "realtime": False, "enable_output": True},
        {"simulate": True, "realtime": False},
        {"simulate": True, "realtime": False, "enable_output": False, "run_seconds": 1},
        {"simulate": "invalid", "realtime": False, "enable_output": False},
    ):
        document["execution"] = execution
        (config / "robot.yaml").write_text(yaml.safe_dump(document))
        result = subprocess.run(
            node_command(binary, config), capture_output=True, text=True, timeout=5
        )
        assert result.returncode == 1, result
        assert "Chassis: 1 kHz" not in result.stdout
print("YAML execution, safe simulation and early validation passed")
