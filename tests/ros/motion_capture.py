"""Live ROS full-sample recording and analysis, offline transport and output disabled."""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import yaml

node, root, build = map(Path, sys.argv[1:])
with tempfile.TemporaryDirectory() as temporary:
    directory = Path(temporary)
    config = directory/'config'
    config.mkdir()
    for path in (root/'config').glob('*.yaml'):
        shutil.copy(path, config/path.name)
    app_path = config/'dust_control.yaml'
    app = yaml.safe_load(app_path.read_text())
    app['execution'] = {'simulate': True, 'realtime': False, 'enable_output': False}
    for module in app['modules'].values():
        module['thread'] = {'cpu': None, 'priority': 0}
    app_path.write_text(yaml.safe_dump(app))
    env = dict(os.environ, ROS_LOCALHOST_ONLY='1', ROS_DOMAIN_ID='176', ROS_LOG_DIR=str(directory/'ros_logs'))
    env['LD_LIBRARY_PATH'] = str(build)+os.pathsep+env.get('LD_LIBRARY_PATH', '')
    env['PYTHONPATH'] = str(build/'rosidl_generator_py')+os.pathsep+env.get('PYTHONPATH', '')
    logfile = (directory/'node.log').open('w')
    process = subprocess.Popen([str(node), '--ros-args', '-p', f'config_directory:={config}'],
                               env=env, stdout=logfile, stderr=subprocess.STDOUT)
    try:
        capture = directory/'offline.jsonl'
        result = subprocess.run([sys.executable, str(root/'tools/capture_motion_diagnostics.py'),
            '--output', str(capture), '--config-directory', str(config),
            '--scenario', 'offline_invalid_samples', '--condition', 'unlabeled',
            '--duration', '.8', '--wait', '12'], env=env, text=True, capture_output=True, timeout=18)
        assert result.returncode == 0, (result.stdout, result.stderr, (directory/'node.log').read_text())
        rows = [json.loads(line) for line in capture.read_text().splitlines()]
        assert len(rows) > 10, len(rows)
        assert all(row['version'] == 9 for row in rows)
        assert all(not row['motion_estimator_valid'] for row in rows) # invalid states must be logged
        assert all('imu_sample_monotonic_us' in row for row in rows)
        assert all('motion_reliability_parameters' in row for row in rows)
        assert all(row['wheel_speed_pair_residual_mps'] == [None, None] for row in rows)
        assert all(a['sequence'] < b['sequence'] for a, b in zip(rows, rows[1:]))
        summary = json.loads(capture.with_suffix('.summary.json').read_text())
        assert summary['written_samples'] == len(rows) and summary['error'] is None
        report_dir = directory/'report'
        analyzed = subprocess.run([sys.executable, str(root/'tools/analyze_motion_diagnostics.py'),
            str(capture), '--output-dir', str(report_dir)], text=True, capture_output=True)
        assert analyzed.returncode == 0, (analyzed.stdout, analyzed.stderr)
        report = json.loads((report_dir/'threshold_report.json').read_text())
        assert report['invalid_estimator_samples'] == len(rows)
        assert (report_dir/'events.csv').is_file()
        print(f'Captured and analyzed {len(rows)} offline samples, including invalid estimator states.')
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        logfile.close()
