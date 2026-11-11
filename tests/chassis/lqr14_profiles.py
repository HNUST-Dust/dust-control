"""验证调试配置输出门、隔离性及完整应用参数校验。"""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/"tools"))
from prepare_lqr14_config import prepare

probe = sys.argv[1]
with tempfile.TemporaryDirectory() as directory:
    directory = Path(directory)
    original = (ROOT/"config/dust_control.yaml").read_bytes()
    for mode in ("observe", "active", "deploy"):
        destination = directory/mode
        app, chassis = prepare(ROOT/"config", destination, mode)
        assert app["execution"]["enable_output"] == (mode != "observe")
        assert chassis["balance"]["max_body_speed_mps"] == (1.6 if mode == "deploy" else .3)
        assert chassis["balance"]["max_turn_wheel_speed_mps"] == (2.7 if mode == "deploy" else .215)
        subprocess.run([probe, str(destination)], check=True, capture_output=True)
        try:
            prepare(ROOT/"config", destination, mode)
        except ValueError:
            pass
        else:
            raise AssertionError("nonempty destination overwritten")
    prepare(ROOT/"config", directory/"simulate", "observe", simulate=True)
    subprocess.run([probe, str(directory/"simulate")], check=True, capture_output=True)
    try:
        prepare(ROOT/"config", directory/"invalid", "active", non_realtime=True)
    except ValueError:
        pass
    else:
        raise AssertionError("non-realtime active profile accepted")
    assert original == (ROOT/"config/dust_control.yaml").read_bytes()
print("LQR14 commissioning profile isolation and output gates passed")
