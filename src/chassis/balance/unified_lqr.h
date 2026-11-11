/* SPDX-License-Identifier: Apache-2.0 */
#pragma once

#include <Eigen/Core>

namespace dust_control::chassis {

/**
 * 双轮接地工作点附近的十四状态、六输入离散 LQR 跟踪器。
 * 增益在离线以 1 ms 采样周期求解；周期内只做固定大小矩阵运算。
 * 不推断接触状态、不估计高度、不执行设备 I/O；调用方负责这些边界。
 */
class UnifiedLqr {
   public:
    static constexpr int kStateCount = 14;
    static constexpr int kInputCount = 6;
    static constexpr double kSampleTime = 0.001;

    enum StateIndex {
        kPosition,
        kSpeed,
        kYaw,
        kYawRate,
        kLeftTheta,
        kLeftThetaRate,
        kRightTheta,
        kRightThetaRate,
        kPitch,
        kPitchRate,
        kRoll,
        kRollRate,
        kHeight,
        kHeightRate,
    };
    enum InputIndex {
        kLeftWheelTorque,
        kRightWheelTorque,
        kLeftBodyOnLegTorque,
        kRightBodyOnLegTorque,
        kLeftVerticalForce,
        kRightVerticalForce,
    };
    // 公开对象跨库边界传递；显式禁用存储对齐，避免调用方的 Eigen
    // 向量化编译选项改变控制器成员布局（实时库使用 EIGEN_DONT_VECTORIZE）。
    using State = Eigen::Matrix<double, kStateCount, 1, Eigen::DontAlign>;
    using Input = Eigen::Matrix<double, kInputCount, 1, Eigen::DontAlign>;
    using Gain =
        Eigen::Matrix<double, kInputCount, kStateCount, Eigen::DontAlign>;

    static_assert(alignof(State) == alignof(double));
    static_assert(alignof(Input) == alignof(double));
    static_assert(alignof(Gain) == alignof(double));

    struct Design {
        Gain gain = Gain::Zero();
        double sample_time_s = kSampleTime;
    };

    struct Reference {
        State state = State::Zero();
        // 绝对物理输入：前四项 N·m，后两项向上的竖直地面反力 N。
        // 包含重力/轨迹前馈；本算法不再额外加 mg/2 或腿长 PID。
        Input feedforward = Input::Zero();
    };

    struct Limits {
        Input lower = Input::Zero();
        Input upper = Input::Zero();
    };

    struct Result {
        bool valid = false;
        bool saturated = false;
        State error = State::Zero();
        Input feedback = Input::Zero();
        Input requested = Input::Zero();
        Input command = Input::Zero();
    };

    // 启动时校验增益、采样周期和独立输出限幅；无效参数抛 invalid_argument。
    // 这里只能检查参数格式，闭环稳定性由离线模型验证。
    UnifiedLqr(Design design, Limits limits);

    // 无动态分配、无阻塞、无跨周期状态。无效输入返回 valid=false 的零结果，
    // 调用方必须据此回退，不能把零支撑力当作有效平衡命令。
    [[nodiscard]] Result Compute(const State& measured,
                                 const Reference& reference) const noexcept;

   private:
    const Design design_;
    const Limits limits_;
};

}  // namespace dust_control::chassis
