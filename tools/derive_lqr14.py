#!/usr/bin/env python3
"""扩展已恢复的十状态模型，生成双侧支撑十四状态研发增益调度。"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.linalg import expm

import derive_lqr_schedule as legacy
import design_lqr14 as design

ROOT = Path(__file__).resolve().parents[1]


def read_parameters(path):
    parameters = json.loads(Path(path).read_text())
    if parameters.get("schema_version") != 1:
        raise ValueError("unsupported LQR14 parameter schema")
    if not isinstance(parameters.get("calibration_status"), str) or not parameters["calibration_status"].strip():
        raise ValueError("calibration_status is required")
    if not isinstance(parameters.get("roll_inertia_source"), str) or not parameters["roll_inertia_source"].strip():
        raise ValueError("roll_inertia_source is required")
    if parameters.get("roll_model") != "body_only_quasistatic_legs":
        raise ValueError("unsupported roll reduction")
    for key in ("body_roll_inertia_kg_m2", "vertical_force_weight"):
        value = float(parameters[key])
        if not np.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    for key in ("roll_rate_damping_nm_s", "vertical_damping_n_s_per_m"):
        value = float(parameters[key])
        if not np.isfinite(value) or value < 0:
            raise ValueError(f"{key} must be nonnegative and finite")
    for key in ("roll_state_weights", "height_state_weights"):
        value = np.asarray(parameters[key], dtype=float)
        if value.shape != (2,) or not np.isfinite(value).all() or (value <= 0).any():
            raise ValueError(f"{key} must contain two positive finite weights")
    return parameters


def validate_samples(samples):
    if len(samples) < 2:
        raise ValueError("at least two leg samples are required")
    lengths = np.array([s["leg_length_m"] for s in samples])
    coms = np.array([s["leg_com_from_wheel_m"] for s in samples])
    inertias = np.array([s["single_leg_inertia_kg_m2"] for s in samples])
    if not np.isfinite([lengths, coms, inertias]).all() or \
            (np.diff(lengths) <= 0).any() or (coms <= 0).any() or \
            (coms >= lengths).any() or (inertias <= 0).any():
        raise ValueError("invalid or duplicate leg geometry/inertia samples")
    slopes = np.diff(coms) / np.diff(lengths)
    if (slopes <= 0).any() or (slopes >= 1).any():
        raise ValueError("leg COM slopes must be between zero and one for this reduction")
    return lengths, slopes


def sample_at(samples, length):
    lengths, slopes = validate_samples(samples)
    if not np.isfinite(length) or not lengths[0] <= length <= lengths[-1]:
        raise ValueError("model leg length must lie inside measured CSV range")
    sample = legacy.interpolate_sample(samples, lengths, length)
    # CSV 采用分段线性重心轨迹；内部节点使用左右导数平均，避免任意选边。
    index = int(np.searchsorted(lengths, length))
    if index < len(lengths) and length == lengths[index] and 0 < index < len(lengths)-1:
        slope = 0.5 * (slopes[index-1] + slopes[index])
    else:
        interval = int(np.clip(np.searchsorted(lengths, length, side="right")-1,
                               0, len(slopes)-1))
        slope = slopes[interval]
    return sample, float(slope)


def derive_model(samples, length, parameters):
    sample, com_slope = sample_at(samples, length)
    robot = legacy.RobotParameters()
    a10, b10 = legacy.derive_model(sample)
    a, b = np.zeros((14, 14)), np.zeros((14, 6))
    a[:10, :10], b[:10, :4] = a10, b10
    # h 为髋部中点高度；轮轴在水平地面保持固定高度。
    # 总竖直动量导数 = (m_body + 2*m_leg*dc/dL)*h_ddot。
    # 轮质量只进入静态总重，不随 h 一起作竖直平移。
    moving_mass = robot.body_mass_kg + 2*robot.leg_mass_each_kg*com_slope
    total_mass = robot.body_mass_kg + 2*robot.leg_mass_each_kg + 2*robot.wheel_mass_each_kg
    # 这是机体 CAD 惯量，不是整机有效惯量。准静态腿近似忽略腿轮滚转惯性，
    # 暂把接地竖直力差直接映射成机体 roll 力矩；接入前需验证动态载荷传递。
    inertia = parameters["body_roll_inertia_kg_m2"]
    a[10, 11] = a[12, 13] = 1.0
    # 机体 COM 相对髋部的有符号竖直偏移 d 沿用原模型。
    # 准静态腿时 F_hip,sum = m_body*g，产生 d*m_body*g*sin(roll)。
    a[11, 10] = robot.body_mass_kg*robot.gravity_m_s2*robot.body_com_offset_m/inertia
    a[11, 11] = -parameters["roll_rate_damping_nm_s"]/inertia
    a[13, 13] = -parameters["vertical_damping_n_s_per_m"]/moving_mass
    b[11, 4:] = [robot.half_track_m/inertia, -robot.half_track_m/inertia]
    b[13, 4:] = [1/moving_mass, 1/moving_mass]
    q, r = np.zeros((14, 14)), np.zeros((6, 6))
    q[:10, :10], r[:4, :4] = legacy.Q, legacy.R
    q[10:, 10:] = np.diag(parameters["roll_state_weights"] + parameters["height_state_weights"])
    r[4:, 4:] = np.eye(2)*parameters["vertical_force_weight"]
    equilibrium = np.zeros(14)
    equilibrium[12] = robot.wheel_radius_m + length
    inputs = np.zeros(6)
    inputs[4:] = total_mass*robot.gravity_m_s2/2
    return {
        "state_order": design.STATES, "input_order": design.INPUTS,
        "sample_time_s": design.SAMPLE_TIME, "time_domain": "continuous",
        "contact_mode": "double_support",
        "input_convention": "physical_torques_upward_vertical_forces",
        "model_source": "recovered derive_lqr_schedule.py + CSV COM slope heave reduction + explicit body-only roll approximation",
        "height_definition": "world z of hip midpoint above common flat ground; h0 = wheel_radius + leg_length",
        "roll_convention": "positive raises left hip; positive left-minus-right vertical force accelerates positive roll",
        "calibration_status": parameters["calibration_status"],
        "leg_length_m": float(length), "leg_com_slope": com_slope,
        "heave_effective_mass_kg": float(moving_mass),
        "robot_parameters": asdict(robot), "extension_parameters": parameters,
        "operating_state": equilibrium.tolist(), "operating_input": inputs.tolist(),
        "A": a.tolist(), "B": b.tolist(), "Q": q.tolist(), "R": r.tolist(),
        "assumptions": ["flat double support", "symmetric upright zero-speed zero-spin equilibrium",
                        "small-angle first-order heave/roll reduction; no spin or impact coupling",
                        "body CAD roll inertia assumed about COM; vertical COM offset shared with pitch model",
                        "piecewise-linear leg COM path; body CAD roll inertia; leg/wheel roll inertia neglected",
                        "vertical ground force inputs require a separate actuator allocation"],
    }


def check_schedule(samples, parameters, lengths, gains, count):
    if count < 2:
        raise ValueError("dense validation requires at least two points")
    worst = 0.0
    for length in np.unique(np.concatenate((np.linspace(lengths[0], lengths[-1], count), lengths, np.nextafter(lengths[1:-1], -np.inf),
                                              np.nextafter(lengths[1:-1], np.inf)))):
        model = derive_model(samples, float(length), parameters)
        a, b = np.asarray(model["A"]), np.asarray(model["B"])
        augmented = np.zeros((20, 20))
        augmented[:14, :14], augmented[:14, 14:] = a, b
        transition = expm(augmented*design.SAMPLE_TIME)
        upper = min(int(np.searchsorted(lengths, length, side="right")), len(lengths)-1)
        lower = max(upper-1, 0)
        ratio = 0.0 if lower == upper else (length-lengths[lower])/(lengths[upper]-lengths[lower])
        gain = (1-ratio)*gains[lower] + ratio*gains[upper]
        radius = float(np.max(np.abs(np.linalg.eigvals(transition[:14, :14]-transition[:14, 14:]@gain))))
        if not np.isfinite(radius) or radius >= 1:
            raise ValueError(f"interpolated schedule unstable at L={length}: radius={radius}")
        worst = max(worst, radius)
    return worst


def schedule_header(lengths, gains):
    # 研发数据放在 tools/generated；生产库不包含此表。
    lines = ["// DEVELOPMENT ONLY: unverified roll inertia; do not enable on hardware.",
             "// Generated by tools/derive_lqr14.py; model provenance in adjacent JSON.",
             "#pragma once", '#include "chassis/lqr14_schedule.h"',
             "namespace dust_control::chassis {",
             "inline const std::array<Lqr14GainSample, " + str(len(lengths)) + ">& Lqr14DevelopmentSamples() {",
             "    static const std::array<Lqr14GainSample, " + str(len(lengths)) + "> samples = [] {",
             "        std::array<Lqr14GainSample, " + str(len(lengths)) + "> value{};"]
    for index, (length, gain) in enumerate(zip(lengths, gains)):
        lines.append(f"        value[{index}].leg_length_m = {length:.17g};")
        rows = [", ".join(f"{v:.17g}" for v in row) for row in gain]
        lines.append(f"        value[{index}].gain <<\n            " + ",\n            ".join(rows) + ";")
    lines += ["        return value;", "    }();", "    return samples;", "}",
              "}  // namespace dust_control::chassis", ""]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=ROOT/"tools/data/leg_mass_properties0820.csv")
    parser.add_argument("--parameters", type=Path, default=ROOT/"tools/data/lqr14_development.json")
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--dense-points", type=int, default=401)
    args = parser.parse_args()
    parameters = read_parameters(args.parameters)
    samples = legacy.read_leg_samples(args.input)
    lengths, _ = validate_samples(samples)
    models = [derive_model(samples, float(length), parameters) for length in lengths]
    results = [design.synthesize(model) for model in models]
    gains = np.array([r["gain"] for r in results])
    worst = check_schedule(samples, parameters, lengths, gains, args.dense_points)
    report = {"sample_time_s": design.SAMPLE_TIME,
              "calibration_status": parameters["calibration_status"],
              "csv_sha256": hashlib.sha256(args.input.read_bytes()).hexdigest(),
              "legacy_script_sha256": hashlib.sha256(Path(legacy.__file__).read_bytes()).hexdigest(),
              "length_range_m": [float(lengths[0]), float(lengths[-1])],
              "interpolated_dense_max_spectral_radius": worst,
              "dense_points": args.dense_points,
              "models": models, "designs": results}
    # 全部验证通过后才写出；不改生产增益表。
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir/"lqr14_schedule.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n")
    (args.output_dir/"lqr14_development.h").write_text(schedule_header(lengths, gains))
    print(f"A=14x14 B=14x6 K=6x14; {len(lengths)} scheduled designs at 1000 Hz")
    print(f"dense maximum discrete spectral radius: {worst:.12f}")
    print(f"calibration status: {parameters['calibration_status']}")


if __name__ == "__main__":
    main()
