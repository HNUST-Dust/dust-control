# 14状态轮足平衡控制

平衡、flight及jump现在统一使用 `lqr14_gain_samples.h`。旧十状态控制计算、
三次多项式增益表、disabled/shadow选择及新旧混合回退已删除。
flight/jump保留各自动作时序和状态/输出掩码，但增益数值来自14状态表，须实机复核。

状态顺序：位置、速度、yaw及角速度、左腿theta及角速度、右腿theta及角速度、
pitch及角速度、roll及角速度、髋部中点世界高度及变化率。
输入：左右轮力矩、左右机体对腿力矩、左右向上地面反力。
Q/R仍按1kHz离散代价设计，保留当前速度权重1及7°经验参考。

## 配置迁移

`balance.lqr14`现在仅接受`roll_sign`，删除旧部署配置中的`enabled`和`shadow_only`：

```yaml
balance:
  lqr14: {roll_sign: 1.0}
```

旧选择键会报配置错误，不会静默忽略。缺省lqr14块也使用14状态控制。
不要直接沿用NUC旧`.cache/lqr14-deployed`目录中的选择键；下次同步时必须迁移。
输出许可仍由`execution.enable_output`及遥控使能门控决定。
`prepare_lqr14_config.py`支持observe、active、deploy，不再提供shadow。

## 生成和运行

共享动力学与前十状态权重：`tools/lqr14_robot_dynamics.py`。
在`derive_lqr14.py`中覆盖Q/R，只作用于14状态生成；roll/高度权重来自
`tools/data/lqr14_development.json`。

```sh
source /opt/ros/humble/setup.bash
OPENBLAS_NUM_THREADS=1 python3 tools/derive_lqr14.py --output-dir .cache/lqr14-generated
cp .cache/lqr14-generated/lqr14_development.h src/chassis/balance/generated/lqr14_gain_samples.h
cmake --build build/lqr14-hardware -j4
ctest --test-dir build/lqr14-hardware --output-on-failure
export LD_LIBRARY_PATH="$PWD/build/lqr14-hardware${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./build/lqr14-hardware/dust_control_node --ros-args -p "config_directory:=$PWD/.cache/lqr14-deployed"
```

生成成功后才替换表。控制与Foxglove需要显式加载ROS及消息库路径，完整命令见[编译与运行](BUILD_RUN.md)。
停止前先遥控失能，再退出控制进程。新版本不自动修改实机配置或启动电机。

## 降级与保护

- 腿长超表采用端点增益；实际姿态误差不夹成零，保留原过倾及输出保护。
- 地面投影、平地残差或双支撑条件不成立，屏蔽高度/roll误差贡献和竖直力输出，
  由腿长PID/VMC接管。支撑不成立时屏蔽yaw反馈，已知低支撑侧轮力矩置零。
- 已知单侧接触时重力前馈仅给支撑侧，非接触腿保留有界腿长动作。
  支撑估计无效不等同于已知离地，此时使用均分重力近似。
- 双支撑持续50ms后，250ms内渐变恢复高度控制，完全接管时清除腿长积分。
- IMU丢失进入原SafetyStop；电机/运动学反馈无效按原WaitingFeedback处理。
  recover及双侧低支撑约20ms后的flight流程保留。

反馈掩码与接触切换不自动继承完整LQR稳定性保证，尚待实机验证。
默认增益仍为原零工作点；`lqr14_trim7_candidate.json`是未启用的经验工作点候选，
不能将其等同于已完成静态标定。

## 遥测与记录

遥测version6，status=2为完整扩展控制、status=9为统一增益降级控制。
blend是高度接管比例，不再是新旧控制器混合比例。
`lqr14_endpoint_gain`和`lqr14_height_control_active`表示端点及高度控制状态。
轮力矩14项贡献按状态顺序排列，request/allocated/output均不是电机实测力矩。
C620反馈保留协议原始电流；最终关节命令包含气弹簧补偿及限幅。
`capture_balance_diagnostics.py`采集，`analyze_balance_diagnostics.py`离线分析。
详细实验结论见`benchmarks/balance-oscillation-2026-11-11.md`。
