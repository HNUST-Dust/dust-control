"""Native C++ SDK WebSocket, freshness, scheduling and startup failure checks."""
import argparse
import asyncio
import json
from pathlib import Path
import socket
import subprocess
from websockets.asyncio.client import connect

ROOT = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--controller', type=Path, default=ROOT / 'build/development/chassis_controller')
parser.add_argument('--fixture', type=Path, default=ROOT / 'build/development/tests/telemetry_contracts')
args = parser.parse_args()


def free_port():
    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


async def subscribe(port, process):
    for _ in range(100):
        if process.poll() is not None:
            raise RuntimeError(process.communicate())
        try:
            ws = await connect(f'ws://127.0.0.1:{port}', subprotocols=['foxglove.sdk.v1'])
            break
        except OSError:
            await asyncio.sleep(.01)
    else:
        raise RuntimeError('Native server startup timed out')
    while True:
        msg = json.loads(await asyncio.wait_for(ws.recv(), 3))
        if msg['op'] == 'serverInfo':
            assert 'clientPublish' not in msg.get('capabilities', [])
        if msg['op'] == 'advertise':
            channel = next(c for c in msg['channels'] if c['topic'] == '/wbr/chassis')
            assert channel['schemaEncoding'] == 'jsonschema'
            await ws.send(json.dumps({'op': 'subscribe', 'subscriptions': [{'id': 1, 'channelId': channel['id']}]}))
            return ws


async def receive(ws):
    while True:
        data = await asyncio.wait_for(ws.recv(), 3)
        if isinstance(data, bytes) and data[0] == 1:
            return json.loads(data[13:])


async def check():
    port = free_port()
    fixture = subprocess.Popen([str(args.fixture), str(port), 'native'],
                               stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        ws = await subscribe(port, fixture)
        try:
            fresh = stale = disconnected = False
            while not disconnected:
                msg = await receive(ws)
                if msg['imu_fresh']:
                    assert msg['pitch_deg'] == 12.5 and msg['left_height_m'] == .23
                    fresh = True
                elif msg['source_connected']:
                    assert msg['left_length_m'] is None and msg['right_length_m'] == .26
                    stale = True
                else:
                    assert msg['left_length_m'] is None and msg['right_length_m'] is None
                    disconnected = True
            assert fresh and stale
        finally:
            await ws.close()
        assert fixture.wait(timeout=3) == 0
    finally:
        if fixture.poll() is None:
            fixture.terminate()
        fixture.communicate(timeout=3)

    port = free_port()
    proc = subprocess.Popen([str(args.controller), '--simulate', '--no-rt',
                             '--seconds', '2', '--foxglove-port', str(port)],
                            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    try:
        ws = await subscribe(port, proc)
        try:
            msg = await receive(ws)
            assert msg['simulated'] and msg['source_connected']
            assert msg['pitch_deg'] is None and msg['state'] == 0
            # SDK background threads inherit the worker's non-RT CPU affinity.
            tasks = list(Path(f'/proc/{proc.pid}/task').iterdir())
            assert len(tasks) > 2
            for task in tasks:
                if int(task.name) == proc.pid:
                    continue
                status = (task / 'status').read_text()
                cpulist = next(line.split(':', 1)[1].strip() for line in status.splitlines()
                               if line.startswith('Cpus_allowed_list:'))
                allowed = set()
                for part in cpulist.split(','):
                    ends = part.split('-')
                    allowed.update(range(int(ends[0]), int(ends[-1]) + 1))
                assert 3 not in allowed, (task.name, cpulist)
                assert __import__('os').sched_getscheduler(int(task.name)) == 0
        finally:
            await ws.close()
        proc.terminate()
        stdout, stderr = proc.communicate(timeout=3)
        assert proc.returncode == 0 and 'output=disabled' in stdout, (stdout, stderr)
    finally:
        if proc.poll() is None:
            proc.kill()
            proc.wait()

    with socket.socket() as sock:
        sock.bind(('127.0.0.1', 0))
        sock.listen()
        proc = subprocess.run([str(args.controller), '--simulate', '--no-rt',
                               '--foxglove-port', str(sock.getsockname()[1])],
                              capture_output=True, text=True, timeout=4)
        assert proc.returncode == 1 and 'Foxglove port' in proc.stderr
        assert 'chassis_controller: 1 kHz' not in proc.stdout
    print('Native WebSocket values/freshness, main hook, SDK CPU/scheduler isolation and port conflict passed')


asyncio.run(check())
