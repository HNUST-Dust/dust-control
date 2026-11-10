# 十四状态、六输入 LQR

`UnifiedLqr14` 是独立的 1 kHz 离散反馈算法，已加入 `dust_chassis`。
它接受离线生成的完整 6×14 增益，不在控制周期内求解 Riccati 方程。
`BalanceController` 默认仍使用十状态增益表；可通过显式配置进入观察或十四状态控制，见末节。

## 坐标和输入

状态顺序固定为：

```text
[s, v, yaw, yaw_rate, theta_L, theta_rate_L, theta_R, theta_rate_R,
 pitch, pitch_rate, roll, roll_rate, h, h_rate]
```

位置/高度使用 m，线速度使用 m/s，角度使用 rad，角速度使用 rad/s。
`theta = alpha - pitch` 沿用现有统一腿角坐标，不能使用 FK 原始角。
`h` 是明确机体参考点相对选定地面基准的竖直高度，不是平均腿长。
调用方提供测量和参考，本算法没有从编码器推断未知台阶高度的能力。
角速度必须与模型一致；gyro 机体轴角速度不能无条件视为欧拉角导数。

输入顺序固定为：

```text
[T_wL, T_wR, T_body_on_leg_L, T_body_on_leg_R, F_vertical_L, F_vertical_R]
```

前四项单位 N·m，沿用现有物理轮力矩和机体对腿力矩方向。
后两项单位 N，为向上的绝对竖直地面反力，不是 VMC 轴向力。
左右安装符号仍在 VMC/设备编码边界应用，不进入统一反馈矩阵。
roll 正向与力差的关系必须在实际 A/B 模型中明确，并与 IMU 标定一致。

## 反馈与前馈

```text
error     = measured - reference.state       # 姿态角误差走短弧
feedback  = -K * error
requested = reference.feedforward + feedback
command   = clamp(requested, lower, upper)
```

独立 LQR 使用平衡点参考和相应支撑前馈；MPC 接入后可使用当前时刻的预测参考
和绝对输入前馈。零误差时输出前馈，不会重复加入 mg/2。
任意参考不一定动力学可行，调用方负责生成一致的状态/输入参考。
零误差跟踪也不意味着有积分抗偏能力。

启动时构造器校验采样周期为 0.001 s、参数有限和限幅有序。
`Compute` 无动态分配和阻塞，无效输入或运算溢出返回 `valid=false` 的零结果。
调用方必须检查有效性并选择回退，不能直接把无效零结果发送给电机。
`requested` 和 `command` 均保留，便于上层观察饱和与预测执行差异。

独立分量限幅不是完整执行分配：轮腿力矩、摩擦、接触和机构可达约束需要额外处理。
竖直支撑力变成 VMC 轴向力也需要明确映射，不能直接作为轴向力发送。
本算法仅针对相应双侧支撑线性化工作点；单侧离地/腾空不能沿用其控制权限假设。

## 离线生成增益

原十状态动力学推导和 CSV 已从用户提供的附件恢复，见下方模型扩展说明。
通用求解器仍要求显式 A/B，不会从旧 K 反推模型。研发调度包含明确近似，
此版本不附带已通过实机验证的默认增益。

使用 Python numpy/scipy，提供明确坐标、工作点、完整 A/B 和离散每步 Q/R：

```bash
python3 tools/design_lqr14.py /absolute/path/model.json \
  --output /tmp/lqr14_design.json --header /tmp/lqr14_gain.h
```

模型 JSON 字段：

| 字段 | 内容 |
| --- | --- |
| `state_order` / `input_order` | 与脚本 `STATES` / `INPUTS` 列表逐项一致 |
| `contact_mode` | `double_support` |
| `input_convention` | `physical_torques_upward_vertical_forces` |
| `model_source` | 动力学推导来源和参数版本 |
| `height_definition` | 机体参考点和地面基准 |
| `operating_state` / `operating_input` | 14 维、6 维线性化工作点 |
| `sample_time_s` | `0.001` |
| `time_domain` | `continuous` 或 `discrete` |
| `A` / `B` | 14×14、14×6 线性扰动动力学矩阵 |
| `Q` / `R` | 14×14 半正定、6×6 正定的对称离散代价矩阵 |

连续 A/B 经精确零阶保持离散化；Q/R 始终按离散每步代价解释，
没有把连续代价自动换算成离散代价。脚本求解离散 DARE，并验证残差和
`A_d - B_d K` 的谱半径小于 1；失败则不输出新设计。
工作点信息是追溯信息，脚本无法证明非线性系统确实在该点平衡，必须由模型提供者验证。
生成头文件提供 `MakeLqr14Design()`；JSON 保留工作点、离散矩阵、P 和验证指标。

## 验证与接入边界

`chassis.lqr14` 使用合成耦合系统验证 C++ 闭环扰动衰减、14 个状态通道、
角度跨界、前馈、限幅、无效参数和无动态分配。
`chassis.lqr14_design` 验证离散求解、连续离散一致性、不可稳定模型拒绝和 CLI。
测试夹具位于 `tests/chassis/lqr14_test_design.h`，只供测试，不能用作机器人增益。
其来源为 `lqr14_design.py`，可用 `--write-fixture` 重建。

实际接入还需要：真实动力学与惯量参数、roll/IMU 符号标定、可靠高度反馈、
工作点/腿长增益调度验证，以及支撑力到执行器的分配。
接管后需移除重复的腿长 P/D 与 roll 比例补偿；积分和扰动补偿应单独设计。
下面提供了明确假设下的机器人研发模型；不声明实机稳定或 1 ms 最坏耗时保证。

## 恢复原推导后的机器人模型扩展

已恢复用户提供的 `tools/derive_lqr_schedule.py` 与
`tools/data/leg_mass_properties0820.csv`。原脚本仅修正默认 C++ 输出路径以适应当前目录。
12 个样本的旧十状态系数能在浮点误差范围内复现当前生产系数表。
原表由连续时间 CARE 得到，本扩展则以精确 ZOH 离散化后求解 1 ms DARE，
因此即使旧 A/B 不变，重新求出的十状态子增益也不要求等于原连续增益。

`tools/derive_lqr14.py` 生成近直立、静止、不自旋、左右对称、双侧平地支撑的
十四状态模型。上述对称工作点下采用一阶解耦的 roll/heave 扩展；
这不是包含高速自旋、非对称支撑和碰撞的完整三维动力学。
旧十状态 A/B 原样保留，新通道的零交叉块是此近似的边界，不能外推到那些工况。

### 高度与竖直动力学

`h` 明确定义为髋部中点相对共同平地的世界竖直高度，不是底盘最低点离地间隙。
直立时 `h0 = wheel_radius + L`。CSV 中 `c(L)` 为轮心到单腿重心的距离。
近直立伸缩时，机体重心竖直速度为 `h_dot`，腿重心速度约为 `c'(L)*h_dot`，
轮轴高度不变。由整机竖直动量平衡得到：

```text
M_total = m_body + 2*m_leg + 2*m_wheel
M_heave = m_body + 2*m_leg*c'(L)
M_heave*h_ddot = F_L + F_R - M_total*g - D_h*h_dot
```

因此静态支撑力仍为整机重量，动态等效质量却不同于整机总质量，
也不是把高度直接改名为腿长。CSV 采用分段线性重心轨迹，内部样本节点采用左右斜率平均。
忽略未测的机构形变与高阶耦合；这是局部模型，不预测未知台阶地面。

### 横滚近似与 CAD 惯量

用户确认 `0.113303628 kg·m²` 是**机体 CAD 惯量**。首版采用
`body_only_quasistatic_legs`：忽略腿轮的横滚惯性和动态载荷传递，假设竖直力差
准静态传递到机体；暂假定该 CAD 惯量绕机体 COM 的 roll 轴。
它不能被称为整机有效惯量，也不能把 CSV 的矢状面单腿惯量加到 roll 轴上。

令 `d=-0.05327 m` 为原模型沿用的机体 COM 相对髋部的有符号竖直偏移，
`b=0.215 m`，roll 正向为左髋升高；该约定仍需与实机 IMU 对齐。
在平衡工作点附近：

```text
I_body_roll*roll_ddot = b*(delta_F_L-delta_F_R)
                     + m_body*g*d*roll - D_roll*roll_dot
```

`d<0` 对应此模型下的恢复力矩；roll 模型没有遗漏已知的重心偏移项。
超出近直立范围、伸腿引起的腿轮惯性、轮胎侧向约束和大 yaw-rate 耦合需要后续扩展。
当前参数文件明确标为 `unverified_development_only`，包括起始 Q/R 调参值，
不能把数学稳定性当成模型精度或实机稳定性证明。

### 生成与 C++ 调度

```bash
python3 tools/derive_lqr14.py --output-dir build/lqr14-development
```

输出 `lqr14_schedule.json`（完整 A/B、Q/R、工作点、K、P、来源哈希与诊断）和
`lqr14_development.h`（12 个 6×14 增益）。先通过所有工作点和 401 个密集点、
重心插值节点两侧的离散稳定性检查，再输出文件。

`Lqr14GainSchedule` 在构造时复制并验证这十二个工作点，按平均等效腿长作线性插值，
不在周期内求解 Riccati 方程，也不分配内存。超出 CSV 的 0.15362–0.31101 m
范围会夹紧并返回 `clamped=true`；这不代表对 0.15/0.33 m 已完成模型验证。
生成表已以 `lqr14_gain_samples.h` 随源码保存，避免生产构建依赖 SciPy；仅在显式配置启用时使用。

示例：在启动时取 `Lqr14DevelopmentSamples()` 构造调度器并设置物理限幅；
每周期取 `Evaluate(mean_leg_length)` 的增益、当前高度参考和整机重量前馈，
交给 `UnifiedLqr14` 计算。生成表的静态初始化应在启动时完成。
末节说明新增高度反馈与电机控制接管；本次未接入 MPC。

新增 `chassis.lqr14_robot_model` 和 `chassis.lqr14_robot_schedule` 回归覆盖旧表复现、
动量方程线性化、左右对称性、全部工作点及插值稳定性、受限小扰动恢复、
C++ 反馈方向和无动态分配。需要 numpy/scipy 时才生成这些研发测试。

## 平衡主链路接入与首次调试

`balance.lqr14` 是可选 schema 2 配置块：

```yaml
balance:
  lqr14: {enabled: false, shadow_only: true, roll_sign: 1.0}
```

`enabled=false` 保持原十状态控制；`enabled=true, shadow_only=true` 只计算并发布
新控制结果，电机仍使用旧控制；`enabled=true, shadow_only=false` 才允许新控制接管。
`roll_sign` 只接受 ±1，用于把 IMU roll 对齐“左髋抬高为正”的新模型坐标。
默认配置没有开启新控制。启用时会校验整机质量、轮径、轮距、减速比和轮质量与生成模型一致，
并要求指令腿长处于 CSV 范围内。

高度反馈沿用“竖直虚拟腿长＋轮半径”。两侧 `hz` 是原始镜像机构坐标，不能直接取负号。
设校准腿角为 alpha、IMU pitch=p、模型 roll=r，则向下投影系数为：

```text
q_i = sin(p)*sin(alpha_i) + cos(r)*cos(p)*cos(alpha_i)
h_left  = wheel_radius + L_left*q_left
h_right = wheel_radius + L_right*q_right
h = (h_left+h_right)/2
```

这里沿用现有支撑力估计器的姿态投影约定。在零 roll 时 q=cos(theta)。
高度速度由同一公式解析求导得到，进入反馈前作 20 Hz 低通；不是对位置数值差分。
只有共同水平地面假设下，两侧平均值才是髋部中点的世界高度。
同平地残差为 `L_left*q_left-L_right*q_right-2*b*sin(r)*cos(p)`。

初次接入只覆盖低速双支撑域：左右支撑估计都大于每侧名义支撑的 20%，连续 50 ms；
roll≤10°、pitch≤15°、左右 theta≤20°、同平地残差≤15 mm、
实测 yaw-rate≤1 rad/s、估计前向速度≤0.3 m/s、周期为 0.5–2 ms、腿长调度不夹紧。
支撑估计不是接触传感器，这些条件不能证明地面一定水平，也不能支持下台阶试验。
满足条件后 250 ms 线性接管轮、腿输出；退出条件发生时立即退回旧平衡控制。
跳跃预压缩、回正、腾空和爬楼仍用现有动作控制，外部支撑命令优先于 LQR14。

新分支直接输出支撑力与姿态力矩，经 VMC 和同一个最终输出策略执行。
它不会叠加原腿长 PID 或 roll 比例力；过渡期是两套完整输出的凸组合。
接管期间清空被遮蔽的腿长积分，避免退出时释放积累量。

支撑力映射计入轮重与切向力的竖直分量：

```text
t_z = -sin(p)*cos(alpha) + cos(r)*cos(p)*sin(alpha)
F_axial = (F_ground - m_wheel*g + t_z*T_body_on_leg/L) / q
```

这与现有支撑力估计器的静态轮重关系一致；仍忽略轮竖直加速度与腿自重分布的高阶影响。
最终关节限幅、弹簧补偿和电机协议映射只在原输出边界执行一次。
遥测中的 allocated force 是轴向限幅后的模型值，不包含后续关节饱和及混合比例，
不能当成实际地面反力测量。

准备独立配置（不覆盖 config，也不启动程序）：

```bash
python3 tools/prepare_lqr14_config.py --mode observe --output .cache/lqr14-observe
python3 tools/prepare_lqr14_config.py --mode shadow --output .cache/lqr14-shadow
python3 tools/prepare_lqr14_config.py --mode active --output .cache/lqr14-active
```

调试范围为 160–300 mm、默认腿长 200 mm，平移/自旋指令上限相应降低。
`observe` 明确关闭电机输出；`shadow` 允许旧控制输出；`active` 允许新控制接管。
后两者仍受既有“先观察遥控失能姿态、再使能”的运行门控。
不能把程序启动成功等同于新控制已接管。

ROS `ChassisTelemetry` 升级为 version 2：新增左右髋高、平均髋高及速度、
支撑力估计、新力目标、轴向分配值、状态和接管比例。接收端需要同步重编译消息。
保留原 `left_height_m/right_height_m` 原始几何字段；控制和新观测应使用 `*_hip_height_m`。

状态码：0 关闭，1 观察计算，2 接管中/已接管，3 等待双支撑，4 输入或周期无效，
5 超出模型域，6 外部支撑覆盖，7 其他动作覆盖，8 数值失败。
必须结合 `lqr14_blend` 判断实际占比，完整接管时为 1。

输出关闭时也发布高度与支撑估计，可先核对传感器。机体 CAD 惯量的参考轴、
IMU roll 正向以及支撑估计准确度仍需实测确认；未验证前不进行自旋或台阶测试。

启动裸 CMake 构建可使用 `tools/run_lqr14.sh CONFIG_DIRECTORY [BUILD_DIRECTORY]`，
脚本 source ROS 并设置当前构建的动态类型库路径，不修改配置、不自动提权。
例如在 NUC 项目目录执行 `sudo ./tools/run_lqr14.sh .cache/lqr14-observe`，
该配置明确关闭电机输出。若账号缺少实时权限，sudo 密码只在用户终端输入；当前 NUC 已有 rtprio=80 与无限 memlock，可直接启动。
调试路径默认 `build/lqr14-hardware`，原 `build/production` 保持不变。

## 常规运行配置（2026-11-10）

使用 `tools/prepare_lqr14_config.py --mode deploy --output <新配置目录>`
生成常规运行配置：原 recover 不变，平衡阶段启用 LQR14，恢复源配置的
速度上限（当前平移 1.6 m/s、转向轮速 2.7 m/s，约 12.56 rad/s）。
转向加速度与转向力矩从未在首轮配置中调低，保持原值。
首轮代码中的 0.3 m/s、1 rad/s 接管门槛已移除。

这不代表高速工况已验证：模型未包含自旋耦合，高速下可能出现明显模型误差。
保留模型腿长、姿态、平地双支撑、反馈时效、过倾和执行器输出保护。
运行腿长仍为 160–300 mm，目标 200 mm，未采用外推的 340 mm 参数。
遥控失能仍立即禁止输出。部署不自动启动或自动使能机器人。

## 单向漂移诊断（遥测 version 3）

在 `motion_diagnostics_valid=true` 时对比 `raw_body_speed_mps`、
`estimated_body_speed_mps`、`target_body_speed_mps`，并观察
`estimated_body_position_m`、`target_body_position_m`、`body_position_error_m`
（实际送入控制器的误差，限幅 ±0.15 m）。里程为恢复后局部积分，不是绝对定位。
`remote_chassis_x` 为死区前输入；目标速度经过死区和加速度限制。
`pitch_reference_deg`、`theta_reference_deg`、`roll_reference_deg` 为当前姿态参考。
失能、未执行平衡计算或反馈无效时运动诊断数值为 NaN。

先确认新控制器已完全接管，再在前进指令为零时观察单向漂移。
若目标速度非零，查输入；若物理移动而速度近零，查轮速及腿部补偿；
若速度正确而位置误差增大，查位置限幅、姿态平衡点及执行器余量。
这次仅增加观测，不修改 Kalman、LQR 或参考更新算法。
