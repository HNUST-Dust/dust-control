"""No SDK dependency: real worker serialization, units and freshness contracts."""
import json
import socket
import subprocess
import sys
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(('127.0.0.1', 0))
    sock.settimeout(2)
    process = subprocess.Popen([sys.argv[1], str(sock.getsockname()[1])])
    samples = []
    try:
        while not samples or samples[-1]['sequence'] < 29:
            samples.append(json.loads(sock.recv(16384)))
        assert process.wait(timeout=3) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait()
    fresh = [m for m in samples if m['sequence'] < 16]
    stale = [m for m in samples if m['sequence'] >= 16]
    assert fresh and stale
    for msg in fresh:
        assert msg['pitch_deg'] == 12.5 and msg['roll_deg'] == -3.25
        assert msg['yaw_deg'] == 70
        assert msg['left_length_m'] == .25 and msg['left_height_m'] == .23
        assert msg['right_length_m'] == .26 and msg['right_height_m'] == .24
    for msg in stale:
        assert msg['pitch_deg'] is None and msg['yaw_deg'] is None
        assert msg['left_length_m'] is None and not msg['left_leg_fresh']
        assert msg['right_length_m'] == .26 and msg['right_leg_fresh']
print('Telemetry UDP values, height sign, per-leg freshness and null gaps passed')

# Exercise the production main-loop hook with no hardware, including the
# default-disabled policy. A missing bridge must never become a control error.
with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
    sock.bind(('127.0.0.1', 0))
    sock.settimeout(2)
    result = subprocess.run([sys.argv[2], '--simulate', '--no-rt', '--seconds', '.2',
                             '--telemetry-port', str(sock.getsockname()[1])],
                            capture_output=True, text=True, timeout=4)
    assert result.returncode == 0, result.stderr
    msg = json.loads(sock.recv(16384))
    assert msg['simulated'] and msg['state'] == 0
    assert msg['pitch_deg'] is None and msg['left_length_m'] is None
    assert 'output=disabled' in result.stdout
result = subprocess.run([sys.argv[2], '--simulate', '--no-rt', '--seconds', '.1',
                         '--telemetry-port', '1'],
                        capture_output=True, text=True, timeout=4)
assert result.returncode == 0, result.stderr
print('Production telemetry opt-in, invalid gaps and absent receiver passed')
