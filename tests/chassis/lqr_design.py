"""合成可控耦合系统验证设计链路；所有参数只用于数学回归，不是机器人模型。"""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("design_lqr", ROOT / "tools/design_lqr.py")
design = importlib.util.module_from_spec(spec)
spec.loader.exec_module(design)


def fixture_model():
    a = np.zeros((14, 14))
    b = np.zeros((14, 6))
    for i in range(7):
        a[2*i, 2*i+1] = 1
        a[2*i+1, 2*i+1] = -0.2
    # pitch 是不稳定模态；它与腿角和轮输入耦合，不是七个独立 PD。
    a[9, 8] = 8
    a[5, 8] = a[7, 8] = 0.4
    a[9, 4] = a[9, 6] = 0.3
    b[1, :2] = [1, 1]
    b[3, :2] = [-2, 2]
    b[5, 2] = 2
    b[7, 3] = 2
    b[9, :4] = [-1, -1, -0.5, -0.5]
    b[11, 4:] = [0.215/0.1133, -0.215/0.1133]
    b[13, 4:] = [1/12.054, 1/12.054]
    return {"state_order": design.STATES, "input_order": design.INPUTS,
            "contact_mode": "double_support", "model_source": "SYNTHETIC TEST ONLY",
            "input_convention": "physical_torques_upward_vertical_forces",
            "height_definition": "synthetic vertical coordinate",
            "operating_state": [0.0] * 12 + [0.2, 0.0],
            "operating_input": [0.0] * 4 + [60.0, 60.0],
            "sample_time_s": 0.001, "time_domain": "continuous",
            "A": a.tolist(), "B": b.tolist(),
            "Q": np.diag([10, 2, 10, 2, 30, 3, 30, 3, 80, 5,
                          1200, 30, 12000, 80]).tolist(),
            "R": np.diag([1, 1, 1, 1, 0.025, 0.025]).tolist()}


def fixture_header(result):
    text = "// SYNTHETIC TEST ONLY: never use this gain on the robot.\n" + design.cpp_header(result)
    text += "\nnamespace dust_control::chassis {\n"
    for name, shape, key in (("LqrTestA", (14, 14), "discrete_a"),
                             ("LqrTestB", (14, 6), "discrete_b")):
        values = ",\n        ".join(", ".join(format(x, ".17g") for x in row)
                                   for row in result[key])
        text += (f"inline Eigen::Matrix<double, {shape[0]}, {shape[1]}> {name}() {{\n"
                 f"    Eigen::Matrix<double, {shape[0]}, {shape[1]}> value;\n"
                 f"    value <<\n        {values};\n    return value;\n}}\n")
    return text + "}  // namespace dust_control::chassis\n"


class DesignTests(unittest.TestCase):
    def test_coupled_closed_loop(self):
        result = design.synthesize(fixture_model())
        a, b, k = (np.array(result[key]) for key in ("discrete_a", "discrete_b", "gain"))
        self.assertEqual(k.shape, (6, 14))
        self.assertGreater(np.max(np.abs(np.linalg.eigvals(a))), 1)
        self.assertLess(result["closed_loop_spectral_radius"], 1)
        self.assertLess(result["relative_dare_residual"], 1e-10)
        # 每个状态的独立初始扰动均应衰减。
        response = np.linalg.matrix_power(a - b @ k, 20000)
        self.assertLess(np.linalg.norm(response, ord=2), 0.01)
        # 检查提交的 C++ 测试增益与当前求解结果数值一致。
        header = (ROOT / "tests/chassis/lqr_test_design.h").read_text()
        rows = header.split("design.gain <<", 1)[1].split(";", 1)[0]
        committed = np.fromstring(rows.replace("\n", " "), sep=",").reshape(6, 14)
        np.testing.assert_allclose(committed, k, rtol=1e-8, atol=1e-8)

    def test_reject_invalid_models(self):
        changes = [{"state_order": list(reversed(design.STATES))},
                   {"input_order": list(reversed(design.INPUTS))},
                   {"sample_time_s": 0.01}, {"contact_mode": "single_support"},
                   {"input_convention": "axial_forces"}, {"model_source": ""},
                   {"B": np.zeros((14, 6)).tolist()},
                   {"R": (-np.eye(6)).tolist()}, {"Q": (-np.eye(14)).tolist()},
                   {"A": [[0]]}, {"A": np.full((14, 14), np.nan).tolist()}]
        for change in changes:
            with self.subTest(change=list(change)):
                with self.assertRaises((ValueError, np.linalg.LinAlgError)):
                    design.synthesize({**fixture_model(), **change})

    def test_discrete_and_cli(self):
        continuous = fixture_model()
        first = design.synthesize(continuous)
        discrete = {**continuous, "time_domain": "discrete",
                    "A": first["discrete_a"], "B": first["discrete_b"]}
        second = design.synthesize(discrete)
        np.testing.assert_allclose(first["gain"], second["gain"], atol=1e-10)
        with tempfile.TemporaryDirectory() as directory:
            model, output, header = (Path(directory) / name for name in
                                     ("model.json", "gain.json", "gain.h"))
            model.write_text(json.dumps(continuous))
            subprocess.run([sys.executable, str(ROOT / "tools/design_lqr.py"),
                            str(model), "--output", str(output), "--header", str(header)],
                           check=True, capture_output=True)
            self.assertEqual(json.loads(output.read_text())["gain"], first["gain"])
            self.assertIn("MakeLqrDesign", header.read_text())


if __name__ == "__main__":
    if sys.argv[1:] == ["--write-fixture"]:
        result = design.synthesize(fixture_model())
        (ROOT / "tests/chassis/lqr_test_design.h").write_text(fixture_header(result))
    else:
        unittest.main()
