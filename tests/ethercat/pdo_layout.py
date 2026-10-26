"""Verify the offline compiler against independent sizes and invalid inputs."""
import copy
import json
import pathlib
import subprocess
import sys
import tempfile

import yaml

exe, config_file = sys.argv[1:]
base = yaml.safe_load(pathlib.Path(config_file).read_text())

with tempfile.TemporaryDirectory() as directory:
    config = pathlib.Path(directory) / 'pdo.yaml'

    def run(value, expected_error=None, raw=False):
        config.write_text(value if raw else yaml.safe_dump(value, sort_keys=False))
        result = subprocess.run([exe, str(config), '--json'], text=True,
                                capture_output=True, check=False)
        if expected_error:
            assert result.returncode == 1, result.stdout
            assert expected_error in result.stderr, result.stderr
            return None
        assert result.returncode == 0, result.stderr
        return json.loads(result.stdout)

    def check_image(image):
        offset = 0
        addresses = set()
        for mapping in image['mappings']:
            assert 1 <= len(mapping['entries']) <= 255
            for entry in mapping['entries']:
                assert entry['offset'] == offset
                assert 0 < entry['bit_length'] <= 240
                assert entry['bit_length'] % 8 == 0
                address = (entry['object_index'], entry['subindex'])
                assert address not in addresses
                addresses.add(address)
                offset += entry['bit_length'] // 8
        assert offset == image['bytes']

    original = run(base)
    board = original['adapters']['base']
    # v0's 48-byte artificial trailing padding is deliberately omitted.
    assert board['rx']['bytes'] == 976
    assert board['tx']['bytes'] == 1536
    assert original['estimated_frames'] == 3
    assert original['estimated_wire_time_ns'] == (976 + 1536 + 3 * 52) * 80
    check_image(board['rx'])
    check_image(board['tx'])

    reordered = copy.deepcopy(base)
    pdo = reordered['adapters']['base']['pdo']
    pdo['can']['rx_slots'] = dict(reversed(list(pdo['can']['rx_slots'].items())))
    pdo['uart'] = dict(reversed(list(pdo['uart'].items())))
    assert run(reordered) == original

    small = copy.deepcopy(base)
    pdo = small['adapters']['base']['pdo']
    pdo['can'] = {'tx_slots': {'chassis': 6, 'gimbal': 0}, 'rx_slots': {2: 3, 3: 3}}
    pdo['uart'] = {1: {'tx_bytes': 0, 'rx_bytes': 32},
                   2: {'tx_bytes': 0, 'rx_bytes': 128}}
    pdo['onboard_imu'] = False
    reduced = run(small)['adapters']['base']
    assert reduced['rx']['bytes'] == 64 + 16 + 6 * 16 + 2 * 16
    assert reduced['tx']['bytes'] == 64 + 2 * (16 + 3 * 24) + 2 * 16 + 160 + 224
    assert reduced['layout_signature'] != board['layout_signature']
    check_image(reduced['rx'])
    check_image(reduced['tx'])
    fields = [e['name'] for m in reduced['rx']['mappings'] for e in m['entries']]
    assert 'uart1.peer_ack_sequence' in fields
    assert not any(f.startswith('uart1.data') for f in fields)
    assert not any(f.startswith('can.gimbal.') for f in fields)

    multi = copy.deepcopy(base)
    multi['adapters']['arm'] = copy.deepcopy(small['adapters']['base'])
    multi['adapters']['arm']['position'] = 1
    multi['adapters']['head'] = copy.deepcopy(small['adapters']['base'])
    multi['adapters']['head']['position'] = 2
    mixed = run(multi)
    assert len(mixed['adapters']) == 3
    assert mixed['adapters']['arm']['layout_signature'] == reduced['layout_signature']
    assert mixed['total_rx_bytes'] == board['rx']['bytes'] + 2 * reduced['rx']['bytes']
    assert mixed['total_tx_bytes'] == board['tx']['bytes'] + 2 * reduced['tx']['bytes']
    for adapter in mixed['adapters'].values():
        check_image(adapter['rx'])
        check_image(adapter['tx'])

    # Exercise object/subindex and mapping rollover with maximum per-bus slots.
    wide = copy.deepcopy(base)
    wide['adapters']['base']['pdo']['can']['rx_slots'] = {i: 32 for i in range(4)}
    expanded = run(wide)['adapters']['base']
    assert len(expanded['tx']['mappings']) > 1
    check_image(expanded['tx'])

    all_three = copy.deepcopy(base)
    for i, name in enumerate(['arm', 'head'], 1):
        all_three['adapters'][name] = copy.deepcopy(base['adapters']['base'])
        all_three['adapters'][name]['position'] = i
    assert run(all_three)['estimated_wire_time_ns'] == 640320
    over_budget = copy.deepcopy(all_three)
    for adapter in over_budget['adapters'].values():
        adapter['pdo']['can']['rx_slots'] = {i: 32 for i in range(4)}
    run(over_budget, 'aggregate wire time')
    receive_only = copy.deepcopy(base)
    receive_only['adapters']['base']['pdo']['uart'] = {2: {'tx_bytes': 0, 'rx_bytes': 512}}
    large_uart = run(receive_only)['adapters']['base']
    check_image(large_uart['rx'])
    check_image(large_uart['tx'])

    def reject(mutator, message):
        value = copy.deepcopy(base)
        mutator(value)
        run(value, message)

    reject(lambda c: c['ethercat'].__setitem__('cycle_time_us', 500), 'Only cycle_time_us')
    reject(lambda c: c.__setitem__('adapters', {}), 'Expected 1..3')
    reject(lambda c: c['adapters'].__setitem__('copy', copy.deepcopy(c['adapters']['base'])),
           'Duplicate EtherCAT')
    reject(lambda c: c['adapters']['base']['pdo']['can']['rx_slots'].__setitem__(4, 1), 'out of range')
    reject(lambda c: c['adapters']['base']['pdo']['can']['tx_slots'].__setitem__('chassis', 17), 'out of range')
    reject(lambda c: c['adapters']['base']['pdo']['uart'].__setitem__(0, {'tx_bytes': 1, 'rx_bytes': 1}),
           'Invalid or duplicate UART')
    reject(lambda c: c['adapters']['base']['pdo']['uart'][2].__setitem__('rx_bytes', 513), 'out of range')
    reject(lambda c: c['adapters']['base']['pdo']['uart'][2].__setitem__('tx_bytes', -1), 'unsigned integer')
    reject(lambda c: c['adapters']['base']['pdo'].__setitem__('unknown', True), 'Unknown or duplicate')
    reject(lambda c: c['ethercat']['limits'].__setitem__('max_rxpdo_bytes', 100), 'process image planning')
    reject(lambda c: c['ethercat']['limits'].__setitem__('max_wire_time_us', 100), 'aggregate wire time')
    invalid = copy.deepcopy(all_three)
    invalid['adapters']['extra'] = copy.deepcopy(base['adapters']['base'])
    run(invalid, 'Expected 1..3')
    invalid = pathlib.Path(config_file).read_text().replace('position: 0', 'position: 0\n    position: 1')
    run(invalid, 'Unknown or duplicate', raw=True)
    invalid = pathlib.Path(config_file).read_text().replace('0: 8, 1: 8', '0: 8, 0x0: 8, 1: 8')
    run(invalid, 'Duplicate CAN channel', raw=True)
    invalid = pathlib.Path(config_file).read_text().replace('1: {tx_bytes: 16', '0x1: {tx_bytes: 16')
    assert run(invalid, raw=True) == original

print('PDO layouts, independent capacities, mixed boards, mapping bounds and rejection cases passed')
