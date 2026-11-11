"""Application schema 4 and PDO configuration fail before any bus access."""
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
    run(lambda d: d['dust_control.yaml']['ethercat'].__setitem__('master_index', 256))
    run(lambda d: d['dust_control.yaml']['ethercat']['thread'].__setitem__('priority', 100))
    run(lambda d: d['dust_control.yaml']['modules']['chassis'].__setitem__('sensor_adapter', 'absent'))
    run(lambda d: d['dust_control.yaml']['adapters']['base'].__setitem__('protocol', 'generic'))
    run(lambda d: d['dust_control.yaml']['adapters']['base'].__setitem__('strict_hi91_crc', True))
    run(lambda d: d['dust_control.yaml']['ethercat'].__setitem__('cycle_time_us', 1000))
    run(lambda d: d['dust_control.yaml'].__setitem__('schema_version', 3))
    run(lambda d: d['dust_control.yaml']['adapters']['base'].pop('vendor_id'))
    run(lambda d: d['motors_params.yaml']['motors'][0].__setitem__('adapter', 'absent'))
    run(lambda d: d['motors_params.yaml']['motors'][0].__setitem__('freshness_deadline_us', 0))
    run(lambda d: d['sensors_params.yaml']['hi91'].__setitem__('freshness_deadline_us', 0))
    run(lambda d: d['chassis_params.yaml'].__setitem__('unknown', 1))
    # 老 schema 2 配置可以没有可选 LQR 块；显式接入必须校验模型范围。
    run(lambda d: d['chassis_params.yaml']['balance'].pop('lqr'), valid=True)
    run(lambda d: d['chassis_params.yaml']['balance']['lqr'].__setitem__('enabled', True))
    def lqr(d):
        p = d['chassis_params.yaml']
        p['balance']['lqr']['roll_sign'] = 1.0
        p['balance']['min_leg_length_m'] = .16
        p['balance']['max_leg_length_m'] = .30
        p['stool']['target_leg_length_m'] = .20
    run(lqr, valid=True)
    def wrong_mass(d):
        lqr(d)
        d['chassis_params.yaml']['mechanics']['robot_mass_kg'] = 13
    run(wrong_mass)
    run(lambda d: d['chassis_params.yaml']['balance']['lqr'].__setitem__('roll_sign', 0))
    run(lambda d: d['chassis_params.yaml']['balance']['lqr'].__setitem__('extra', 1))
    run(lambda d: d['dust_control.yaml']['execution'].update(simulate=True, enable_output=True))
    run(lambda d: d['dust_control.yaml']['execution'].update(simulate=True, realtime=False), valid=True)
print('Application/PDO schema, bindings, missing files and retired configuration rejection passed')
