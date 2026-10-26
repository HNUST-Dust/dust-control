"""Application schema 3 and PDO configuration fail before any bus access."""
import copy
import pathlib
import shutil
import subprocess
import sys
import tempfile
import yaml

exe, source = sys.argv[1:]
with tempfile.TemporaryDirectory() as temporary:
    directory = pathlib.Path(temporary)
    for path in pathlib.Path(source).glob('*.yaml'):
        shutil.copy(path, directory / path.name)
    baseline = {path.name: yaml.safe_load(path.read_text()) for path in directory.glob('*.yaml')}

    def run(changes=None, valid=False):
        docs = copy.deepcopy(baseline)
        if changes:
            changes(docs)
        for name, content in docs.items():
            (directory / name).write_text(yaml.safe_dump(content, sort_keys=False))
        result = subprocess.run([exe, str(directory)], text=True, capture_output=True)
        assert result.returncode == (0 if valid else 1), (result.stdout, result.stderr)

    run(valid=True)
    run(lambda d: d['dust_control.yaml'].__setitem__('schema_version', 2))
    run(lambda d: d['dust_control.yaml']['adapters']['base'].__setitem__('serial', 'old-usb'))
    run(lambda d: d['dust_control.yaml']['ethercat'].__setitem__('pdo_file', 'missing.yaml'))
    run(lambda d: d['dust_control.yaml']['ethercat']['thread'].__setitem__('priority', 100))
    run(lambda d: d['dust_control.yaml']['modules']['chassis'].__setitem__('sensor_adapter', 'absent'))
    run(lambda d: d['dust_control.yaml']['adapters']['base']['uart_sensors'].__setitem__('hi91', 1))
    run(lambda d: d['ethercat_pdo.yaml']['adapters']['base']['pdo']['uart'][2].__setitem__('rx_bytes', 0))
    run(lambda d: d['ethercat_pdo.yaml']['ethercat'].__setitem__('cycle_time_us', 500))
    run(lambda d: d['ethercat_pdo.yaml']['adapters'].__setitem__('extra', copy.deepcopy(d['ethercat_pdo.yaml']['adapters']['base'])))
    run(lambda d: d['motors_params.yaml']['motors'][0].__setitem__('adapter', 'absent'))
    run(lambda d: d['motors_params.yaml']['motors'][0].__setitem__('freshness_deadline_us', 0))
    run(lambda d: d['sensors_params.yaml']['hi91'].__setitem__('freshness_deadline_us', 0))
    run(lambda d: d['chassis_params.yaml'].__setitem__('unknown', 1))
    run(lambda d: d['dust_control.yaml']['execution'].update(simulate=True, enable_output=True))
    run(lambda d: d['dust_control.yaml']['execution'].update(simulate=True, realtime=False), valid=True)
print('Application/PDO schema, bindings, missing files and legacy USB rejection passed')
