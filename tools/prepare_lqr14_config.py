#!/usr/bin/env python3
"""复制独立 LQR14 调试配置；不覆盖现有配置，不启动程序。"""
import argparse
from pathlib import Path
import shutil

import yaml


def prepare(source, output, mode, simulate=False, non_realtime=False):
    source, output = Path(source).resolve(), Path(output).resolve()
    if source == output or (output.exists() and any(output.iterdir())):
        raise ValueError("output must be a new or empty directory, separate from source")
    if mode != "observe" and (simulate or non_realtime):
        raise ValueError("output-enabled modes require real hardware and realtime scheduling")
    app = yaml.safe_load((source/"dust_control.yaml").read_text())
    chassis = yaml.safe_load((source/"chassis_params.yaml").read_text())
    app["execution"]["simulate"] = simulate
    app["execution"]["realtime"] = not (simulate or non_realtime)
    app["execution"]["enable_output"] = mode != "observe"
    chassis["balance"]["lqr14"] = {"roll_sign": 1.0}
    chassis["balance"]["min_leg_length_m"] = .16
    chassis["balance"]["max_leg_length_m"] = .30
    if mode != "deploy":
        chassis["balance"]["max_body_speed_mps"] = .3
        chassis["balance"]["max_turn_wheel_speed_mps"] = .215
    chassis["stool"]["target_leg_length_m"] = .20
    output.mkdir(parents=True, exist_ok=True)
    for path in source.glob("*.yaml"):
        shutil.copy2(path, output/path.name)
    (output/"dust_control.yaml").write_text(yaml.safe_dump(app, sort_keys=False))
    (output/"chassis_params.yaml").write_text(yaml.safe_dump(chassis, sort_keys=False))
    return app, chassis


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=Path(__file__).resolve().parents[1]/"config")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mode", choices=("observe", "active", "deploy"), default="observe")
    parser.add_argument("--simulate", action="store_true")
    parser.add_argument("--non-realtime", action="store_true")
    args = parser.parse_args()
    app, chassis = prepare(args.source, args.output, args.mode, args.simulate, args.non_realtime)
    print(f"configuration: {args.output.resolve()}")
    print(f"execution: {app['execution']}")
    print(f"lqr14: {chassis['balance']['lqr14']}")
    print("No process started. Motor output still requires the existing remote enable sequence.")


if __name__ == "__main__":
    main()
