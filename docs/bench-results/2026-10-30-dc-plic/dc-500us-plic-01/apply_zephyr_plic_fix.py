#!/usr/bin/env python3
"""Apply the pinned Zephyr empty-claim backport without overwriting other edits."""
import argparse
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[1]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--zephyr', type=Path, default=root.parent / 'zephyr')
a = p.parse_args()
patch = root / 'patches/zephyr/0001-plic-handle-empty-claim.patch'
def check(reverse=False):
    return subprocess.run(['git', '-C', str(a.zephyr), 'apply', '--check',
        *(['--reverse'] if reverse else []), str(patch)], capture_output=True, text=True)
if check(reverse=True).returncode == 0:
    print('PLIC backport already applied')
else:
    result = check()
    if result.returncode:
        p.error('PLIC source does not match the backport; no files modified.\n' + result.stderr)
    subprocess.run(['git', '-C', str(a.zephyr), 'apply', str(patch)], check=True)
    print('PLIC backport applied')
