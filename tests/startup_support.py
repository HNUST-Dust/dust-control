"""Isolated, output-disabled configuration for ROS performance tools."""
from contextlib import contextmanager
from pathlib import Path
import shutil
import tempfile
import yaml


@contextmanager
def configuration(*, simulate=True, realtime=False):
    if not simulate:
        raise ValueError("Performance tools only support offline input")
    with tempfile.TemporaryDirectory(prefix="dust-config-") as directory:
        path = Path(directory)
        for source in (Path(__file__).resolve().parents[1] / "config").glob("*.yaml"):
            shutil.copy(source, path / source.name)
        document = yaml.safe_load((path / "dust_control.yaml").read_text())
        document["execution"] = {"simulate": True, "realtime": realtime, "enable_output": False}
        if not realtime:
            for module in document["modules"].values():
                module["thread"] = {"cpu": None, "priority": 0}
        (path / "dust_control.yaml").write_text(yaml.safe_dump(document))
        yield path
