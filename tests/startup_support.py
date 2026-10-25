"""Isolated deployment files and signal-driven lifecycle checks for host tests."""
from contextlib import contextmanager
from pathlib import Path
import os
import shutil
import signal
import subprocess
import tempfile

import yaml


@contextmanager
def configuration(*, simulate=True, realtime=False):
    with tempfile.TemporaryDirectory(prefix="dust-config-") as directory:
        path = Path(directory)
        for source in (Path(__file__).resolve().parents[1] / "config").glob("*.yaml"):
            shutil.copy(source, path / source.name)
        document = yaml.safe_load((path / "robot.yaml").read_text())
        document["execution"] = {
            "simulate": simulate,
            "realtime": realtime,
            "enable_output": False,
        }
        (path / "robot.yaml").write_text(yaml.safe_dump(document))
        yield path


def run_for(command, *, seconds=0.3, env=None, timeout=8):
    """Stop a running host through SIGINT; preserve startup/fault exit codes."""
    process = subprocess.Popen(
        command,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        start_new_session=True,
    )
    try:
        try:
            stdout, stderr = process.communicate(timeout=seconds)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGINT)
            stdout, stderr = process.communicate(timeout=timeout)
        return subprocess.CompletedProcess(command, process.returncode, stdout, stderr)
    finally:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate()


def node_command(binary, config):
    return [str(binary), "--ros-args", "-p", "config_directory:=" + str(config)]
