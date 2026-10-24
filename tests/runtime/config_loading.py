import copy
from pathlib import Path
import subprocess
import sys
import tempfile
import yaml

fixture,controller,config_dir=sys.argv[1:]
config_dir=Path(config_dir)
motors=yaml.safe_load((config_dir/'motors_params.yaml').read_text())
sensors=yaml.safe_load((config_dir/'sensors_params.yaml').read_text())
def write(folder,m,s):
    folder.mkdir(exist_ok=True)
    (folder/'motors_params.yaml').write_text(yaml.safe_dump(m))
    (folder/'sensors_params.yaml').write_text(yaml.safe_dump(s))
def run(*args,ok=True):
    result=subprocess.run(args,capture_output=True,text=True,timeout=6)
    assert (result.returncode==0)==ok,(args,result.stdout,result.stderr)
    return result
with tempfile.TemporaryDirectory() as temp:
    root=Path(temp)
    valid=root/'valid'
    motors['left_wheel'].update(command_slot=1,feedback_id=0x202,freshness_deadline_us=1000)
    sensors['hi91']['freshness_deadline_us']=1000
    sensors['remote']['freshness_deadline_us']=1501
    write(valid,motors,sensors)
    run(fixture,str(valid))
    for case in ('missing','unknown','zero','overflow','negative','duplicate','bus','slot','sensor','version'):
        m,s=copy.deepcopy(motors),copy.deepcopy(sensors)
        if case=='missing': del m['left_wheel']['bus']
        elif case=='unknown': m['left_wheel']['typo']=1
        elif case=='zero': m['left_wheel']['freshness_deadline_us']=0
        elif case=='overflow': m['left_wheel']['freshness_deadline_us']=2**40
        elif case=='negative': s['remote']['freshness_deadline_us']=-1
        elif case=='duplicate': m['right_wheel'].update(bus=3,command_slot=1,feedback_id=0x202)
        elif case=='bus': m['left_wheel']['bus']=1
        elif case=='slot': m['left_wheel']['command_slot']=4
        elif case=='sensor': s['extra']={}
        elif case=='version': s['schema_version']=2
        bad=root/case
        write(bad,m,s)
        result=run(controller,'--config',str(bad),'--simulate','--no-rt','--seconds','.05',ok=False)
        assert 'chassis_controller: 1 kHz' not in result.stdout
        run(fixture,str(valid),str(bad))
    run(controller,'--config',str(config_dir),'--simulate','--no-rt','--seconds','.1')
    missing=root/'absent'
    run(controller,'--config',str(missing),'--simulate','--no-rt',ok=False)
print('Module configuration loading, strict validation, atomic startup failure and instance isolation passed')
