"""基于恢复脚本和 CSV 的物理模型回归；不是实机验收。"""
from pathlib import Path
import re
import sys
import unittest

import numpy as np
from scipy.linalg import expm, solve_discrete_are

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "tools"))
import derive_lqr14 as model
import derive_lqr_schedule as legacy
import design_lqr14 as design


class RobotModelTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.samples = legacy.read_leg_samples(ROOT/"tools/data/leg_mass_properties0820.csv")
        cls.parameters = model.read_parameters(ROOT/"tools/data/lqr14_development.json")
        cls.lengths, _ = model.validate_samples(cls.samples)
        cls.models = [model.derive_model(cls.samples, float(l), cls.parameters) for l in cls.lengths]
        cls.designs = [design.synthesize(m) for m in cls.models]

    def test_legacy_provenance(self):
        self.assertEqual(len(self.samples), 12)
        self.assertEqual((self.lengths[0], self.lengths[-1]), (0.15362, 0.31101))
        gains = [legacy.continuous_lqr(*legacy.derive_model(s)) for s in self.samples]
        coefficients = legacy.fit_cubic(self.lengths, np.asarray(gains))
        text = (ROOT/"src/chassis/unified_lqr_coefficients.inc").read_text().split("=", 1)[1]
        old = np.array([float(v) for v in re.findall(
            r"[-+]?(?:\d+\.\d*|\.\d+|\d+)(?:[eE][-+]?\d+)?", text)]).reshape(4, 10, 4)
        np.testing.assert_allclose(coefficients, old, atol=1e-9, rtol=1e-8)

    def test_dimensions_physics_and_symmetry(self):
        robot = legacy.RobotParameters()
        total_mass = robot.body_mass_kg+2*robot.leg_mass_each_kg+2*robot.wheel_mass_each_kg
        self.assertAlmostEqual(total_mass, 12.054)
        for m, sample in zip(self.models, self.samples):
            a, b = np.array(m["A"]), np.array(m["B"])
            old_a, old_b = legacy.derive_model(sample)
            np.testing.assert_array_equal(a[:10, :10], old_a)
            np.testing.assert_array_equal(b[:10, :4], old_b)
            np.testing.assert_array_equal(a[:10, 10:], np.zeros((10, 4)))
            self.assertGreater(m["heave_effective_mass_kg"], robot.body_mass_kg)
            self.assertLess(m["heave_effective_mass_kg"], total_mass)
            self.assertAlmostEqual(b[11, 4], -b[11, 5])
            self.assertAlmostEqual(b[13, 4], b[13, 5])
            self.assertLess(a[11, 10], 0)  # 原模型 d<0：机体 COM 在髋部下方。
            self.assertAlmostEqual(sum(m["operating_input"][4:]), total_mass*robot.gravity_m_s2)
            self.assertAlmostEqual(m["operating_state"][12], robot.wheel_radius_m+sample["leg_length_m"])
            # 直接由 CSV 有限差分检查竖直总动量对髋高速度的增益。
            length = sample["leg_length_m"]
            if self.lengths[0] < length < self.lengths[-1]:
                epsilon = 1e-7
                cp = model.sample_at(self.samples, length+epsilon)[0]["leg_com_from_wheel_m"]
                cm = model.sample_at(self.samples, length-epsilon)[0]["leg_com_from_wheel_m"]
                momentum_gain = robot.body_mass_kg+2*robot.leg_mass_each_kg*(cp-cm)/(2*epsilon)
                self.assertAlmostEqual(momentum_gain, m["heave_effective_mass_kg"], places=7)

    def test_new_channels_against_momentum_equations(self):
        robot = legacy.RobotParameters()
        non_body_weight = 2*(robot.wheel_mass_each_kg+robot.leg_mass_each_kg)*robot.gravity_m_s2
        for m in self.models:
            a, b = np.asarray(m["A"])[10:, 10:], np.asarray(m["B"])[10:, 4:]
            equilibrium_force = np.asarray(m["operating_input"])[4:]
            mass = m["heave_effective_mass_kg"]
            inertia = self.parameters["body_roll_inertia_kg_m2"]
            def derivative(x, force):
                roll, rate, _, speed = x
                body_support = force.sum()-non_body_weight
                moment = robot.half_track_m*np.cos(roll)*(force[0]-force[1])
                moment += robot.body_com_offset_m*np.sin(roll)*body_support
                moment -= self.parameters["roll_rate_damping_nm_s"]*rate
                acceleration = (force.sum()-equilibrium_force.sum()
                                -self.parameters["vertical_damping_n_s_per_m"]*speed)/mass
                return np.array([rate, moment/inertia, speed, acceleration])
            np.testing.assert_allclose(derivative(np.zeros(4), equilibrium_force), 0, atol=1e-12)
            step = 1e-6
            numerical_a = np.column_stack([
                (derivative(np.eye(4)[i]*step, equilibrium_force)
                 -derivative(-np.eye(4)[i]*step, equilibrium_force))/(2*step)
                for i in range(4)])
            numerical_b = np.column_stack([
                (derivative(np.zeros(4), equilibrium_force+np.eye(2)[i]*step)
                 -derivative(np.zeros(4), equilibrium_force-np.eye(2)[i]*step))/(2*step)
                for i in range(2)])
            np.testing.assert_allclose(numerical_a, a, atol=1e-8, rtol=1e-8)
            np.testing.assert_allclose(numerical_b, b, atol=1e-8, rtol=1e-8)

    def test_full_discrete_design(self):
        for m, result in zip(self.models, self.designs):
            k = np.array(result["gain"])
            ad, bd = np.array(result["discrete_a"]), np.array(result["discrete_b"])
            q, r = np.array(m["Q"]), np.array(m["R"])
            p10 = solve_discrete_are(ad[:10, :10], bd[:10, :4], q[:10, :10], r[:4, :4])
            k10 = np.linalg.solve(r[:4, :4]+bd[:10, :4].T@p10@bd[:10, :4],
                                  bd[:10, :4].T@p10@ad[:10, :10])
            np.testing.assert_allclose(k[:4, :10], k10, atol=1e-8, rtol=1e-8)
            np.testing.assert_allclose(k[:4, 10:], 0, atol=1e-8)
            np.testing.assert_allclose(k[4, 10:12], -k[5, 10:12], atol=1e-8)
            np.testing.assert_allclose(k[4, 12:], k[5, 12:], atol=1e-8)
            self.assertGreater(k[4, 10], 0)  # roll>0 时反馈减少左力。
            self.assertGreater(k[4, 12], 0)  # 高于目标时减少两侧力。
            self.assertLess(result["closed_loop_spectral_radius"], 1)
            self.assertLess(result["relative_dare_residual"], 1e-10)

    def test_dense_interpolation(self):
        radius = model.check_schedule(self.samples, self.parameters, self.lengths,
                                      np.array([d["gain"] for d in self.designs]), 401)
        self.assertLess(radius, 1)

    def test_roll_heave_recovery_with_force_limits(self):
        # 小扰动、固定双支撑模型；以绝对输入模拟重力平衡，防止重复加入 mg/2。
        for result in self.designs:
            ad, bd, k = (np.array(result[key]) for key in ("discrete_a", "discrete_b", "gain"))
            x = np.zeros(14)
            x[10], x[11], x[12], x[13] = 0.02, 0.01, 0.005, -0.01
            feedforward = np.array(result["operating_input"])
            lower = np.array([-4.7, -4.7, -54, -54, 0, 0])
            upper = np.array([4.7, 4.7, 54, 54, 200, 200])
            for _ in range(7000):
                command = np.clip(feedforward-k@x, lower, upper)
                x = ad@x + bd@(command-feedforward)
            self.assertLess(np.linalg.norm(x), 1e-5)

    def test_invalid_geometry_and_parameters(self):
        for length in (0.1, 0.33, np.nan, np.inf):
            with self.assertRaises(ValueError):
                model.derive_model(self.samples, length, self.parameters)
        with self.assertRaises(ValueError):
            model.validate_samples([self.samples[0], self.samples[0]])
        bad = [dict(s) for s in self.samples]
        bad[0]["single_leg_inertia_kg_m2"] = -1
        with self.assertRaises(ValueError):
            model.validate_samples(bad)


if __name__ == "__main__":
    unittest.main()
