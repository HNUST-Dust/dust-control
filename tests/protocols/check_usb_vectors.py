"""Compare protocol output with bytes captured before the USB refactor."""
import difflib
from pathlib import Path
import subprocess
import sys


def main():
    result = subprocess.run([sys.argv[1]], check=True, capture_output=True, text=True)
    expected = Path(sys.argv[2]).read_text()
    if result.stdout != expected:
        sys.stderr.writelines(difflib.unified_diff(
            expected.splitlines(keepends=True), result.stdout.splitlines(keepends=True),
            fromfile="USB v4 baseline", tofile="current output"))
        return 1
    print(f"USB v4 baseline: {len(expected.splitlines())} byte vectors/signatures matched")
    return 0


if __name__ == "__main__":
    sys.exit(main())
