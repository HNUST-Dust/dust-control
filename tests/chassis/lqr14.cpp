/* SPDX-License-Identifier: Apache-2.0 */
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>

#include "chassis/chassis_config.h"
#include "chassis/balance/unified_lqr14.h"
#include "lqr14_test_design.h"

namespace chassis = dust_control::chassis;
using Lqr = chassis::UnifiedLqr14;

int main() {
    // 测试模型由 Python/SciPy 独立求解，覆盖完整耦合矩阵；不是实机增益。
    const auto design = chassis::MakeLqr14Design();
    Lqr::Limits limits;
    limits.lower << -5, -5, -10, -10, 0, 0;
    limits.upper << 5, 5, 10, 10, 200, 200;
    const Lqr controller(design, limits);
    Lqr::Reference reference;
    reference.state[Lqr::kHeight] = 0.2;
    reference.feedforward[Lqr::kLeftVerticalForce] = 60;
    reference.feedforward[Lqr::kRightVerticalForce] = 60;
    Eigen::internal::set_is_malloc_allowed(false);
    auto result = controller.Compute(reference.state, reference);
    assert(result.valid && !result.saturated);
    assert(result.command == reference.feedforward);

    // 实际 C++ 反馈闭环：在合成耦合系统上验证所有状态的扰动恢复。
    const auto a = chassis::Lqr14TestA();
    const auto b = chassis::Lqr14TestB();
    for (int initial = 0; initial < Lqr::kStateCount; ++initial) {
        auto simulated = reference.state;
        simulated[initial] += 0.001;
        for (int tick = 0; tick < 20000; ++tick) {
            const auto step = controller.Compute(simulated, reference);
            assert(step.valid && !step.saturated);
            simulated = reference.state + a * (simulated - reference.state) +
                        b * (step.command - reference.feedforward);
        }
        assert((simulated - reference.state).norm() < 1e-5);
    }

    auto state = reference.state;
    state[Lqr::kYaw] += chassis::chassis_config::kTwoPi - 0.01;
    result = controller.Compute(state, reference);
    assert(result.valid);
    assert(std::abs(result.error[Lqr::kYaw] + 0.01) < 1e-12);

    // 所有十四个通道都参与反馈，并与独立标量求和比较。
    for (int i = 0; i < Lqr::kStateCount; ++i) {
        state = reference.state;
        state[i] += 0.001;
        result = controller.Compute(state, reference);
        assert(result.valid && !result.saturated);
        for (int j = 0; j < Lqr::kInputCount; ++j) {
            assert(std::abs(result.feedback[j] + design.gain(j, i) * 0.001) <
                   1e-10);
        }
    }
    state = reference.state;
    state[Lqr::kHeight] -= 100;
    result = controller.Compute(state, reference);
    assert(result.valid && result.saturated);
    assert((result.command.array() >= limits.lower.array()).all());
    assert((result.command.array() <= limits.upper.array()).all());
    state[Lqr::kRoll] = std::numeric_limits<double>::quiet_NaN();
    result = controller.Compute(state, reference);
    assert(!result.valid && result.command.isZero());
    state = reference.state;
    reference.feedforward[0] = std::numeric_limits<double>::infinity();
    assert(!controller.Compute(state, reference).valid);
    Eigen::internal::set_is_malloc_allowed(true);

    for (int failure = 0; failure < 5; ++failure) {
        auto invalid_design = design;
        auto invalid_limits = limits;
        if (failure == 0) {
            invalid_design.sample_time_s = 0.01;
        } else if (failure == 1) {
            invalid_design.gain(0, 0) =
                std::numeric_limits<double>::quiet_NaN();
        } else if (failure == 2) {
            invalid_limits.upper[0] = -6;
        } else if (failure == 3) {
            invalid_limits.lower[Lqr::kLeftVerticalForce] = -1;
        } else {
            invalid_design.gain.setZero();
        }
        bool rejected = false;
        try {
            const Lqr invalid(invalid_design, invalid_limits);
        } catch (const std::invalid_argument&) {
            rejected = true;
        }
        assert(rejected);
    }
}
