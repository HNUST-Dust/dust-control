#!/usr/bin/env python3
"""Format repository-owned C++ with clang-format 18 or newer."""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="report formatting differences")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    formatter = (
        os.environ.get("CLANG_FORMAT")
        or shutil.which("clang-format-18")
        or "clang-format"
    )
    if not shutil.which(formatter):
        parser.error(f"{formatter} not found; install clang-format 18 or newer")
    version = subprocess.check_output([formatter, "--version"], text=True)
    match = re.search(r"version (\d+)", version)
    if not match or int(match.group(1)) < 18:
        parser.error("clang-format 18 or newer is required (InsertBraces and ignore files)")
    # 排除项统一由 .clang-format-ignore 定义并交给格式工具处理。
    files = sorted(
        str(path.relative_to(root))
        for directory in ("src", "tests")
        for path in (root / directory).rglob("*")
        if path.suffix in (".h", ".hpp", ".cpp", ".cc")
    )
    flags = ["--dry-run", "--Werror"] if args.check else ["-i"]
    subprocess.run([formatter, *flags, *files], cwd=root, check=True)


if __name__ == "__main__":
    main()
