"""Real ROS node in offline mode; never acquire an EtherCAT master."""
import os
import pathlib
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import yaml

node, source = sys.argv[1:]
with tempfile.TemporaryDirectory() as temporary:
    directory = pathlib.Path(temporary)
    for path in pathlib.Path(source).glob('*.yaml'):
        shutil.copy(path, directory / path.name)
    config = directory / 'dust_control.yaml'
    app = yaml.safe_load(config.read_text())
    app['execution'] = {'simulate': True, 'realtime': False, 'enable_output': False}
    for module in app['modules'].values():
        module['thread'] = {'cpu': None, 'priority': 0}
    config.write_text(yaml.safe_dump(app))
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='174', ROS_LOG_DIR=temporary)
    process = subprocess.Popen([node, '--ros-args', '-p', f'config_directory:={directory}'],
                               stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, env=env)
    try:
        time.sleep(2)
        assert process.poll() is None, process.communicate()[0]
        process.send_signal(signal.SIGINT)
        output, _ = process.communicate(timeout=10)
        assert process.returncode == 0, output
        assert 'offline' in output, output
    finally:
        if process.poll() is None:
            process.kill()
            process.communicate()
print('ROS node loads EtherCAT schema in offline mode and shuts down cleanly')
