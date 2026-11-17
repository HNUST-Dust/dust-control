# 前向运动估计：固定噪声三状态 KF

当前方案按用户确认收敛为固定Q/R的三状态KF与接触条件切换。
不分类打滑，不根据残差改变轮速方差，也不加入IMU冲击降权。
LQR14、参考生成和执行器策略不在本次改动范围。

## 状态和递推

状态为 `[v,a,b]`：机体前向速度、速度导数、加速度计零偏。
位移由估计速度积分，是沿机体前向速度的累计量，不是世界X/Y。

`F=[[1,dt,0],[0,1,0],[0,0,1]]`。
每周期 `G=[dt²/2,dt,0]ᵀ`，`Q=q*G*Gᵀ`，再向Q[2,2]加dt*qb。
q定义为每周期随机增量驱动方差参数，不是连续白噪声jerk谱密度。
周期变化时应重新核对这一参数定义。

默认q=3000、qb=0.1、Rv=10、Ra=3、P0=I、x0=0，取自参考文章，
尚未按本机器人独立辨识。Rv单位(m/s)²，Ra单位(m/s²)²，不能直接当作融合百分比。
固定的是噪声参数，Kalman增益仍随协方差递推。

加速度观测H=[0,1,1]，轮速观测H=[1,0,0]。
每周期先预测，再更新有效的新IMU样本，最后更新存在的轮速平均观测。
同一IMU sequence不重复校正；协方差采用Joseph形式并显式对称化。
位置使用更新后的速度积分。

## 轮速观测存在条件

保留轮速安装方向、减速比、机体/腿转动及伸缩速度补偿。
左右中心速度为 `zL=vL+d*yaw_rate`、`zR=vR-d*yaw_rate`。

- 双侧明确接地、两路轮速有效且有限：`zv=(zL+zR)/2`，使用固定Rv更新。
- 任一侧离地、接触未知或轮速无效：跳过整路轮速观测，不切换成单轮观测。
- 恢复双支撑后立即按固定Rv恢复更新，不再渐变权重。
- yaw有效性仅影响相应诊断；等权平均时两侧yaw补偿相消，不据此改变Rv。
- 无IMU加速度时不构造零加速度观测；无轮速时不构造零残差伪观测。
- 两路均缺失时仅预测，不虚假缩小协方差。

接触使用现有支撑力估计，不是直接力传感器。仅保留三个参数：
接触建立为总重12%，解除阈值6%，已建立接触的低支撑持续5ms才解除。
滞回区保持既有状态。接触未知时不假设轮地纯滚动成立。

`wheel_contact_gate.*`仅管理这个二值接触/有效性条件。
状态0=轮速无效，1=离地，2=接触未知，4=接地有效。
confidence字段只表示0/1可用性，不是概率或融合系数。
左右差值及预测残差保留记录，但无论多大都不触发降权或拒绝。
因此当前实现不声称能识别、分类或消除持续滑移。

## IMU 边界与配置

`motion_estimation`段可省略，省略时使用默认值；部分参数也可只覆盖所需项。
`contact`只接受contact_off_fraction、contact_on_fraction、contact_loss_delay_s。
旧版`reliability`段及恢复时间、残差阈值等自适应参数已移除，加载时明确拒绝，
不会静默忽略。未知键、非法轴或噪声参数在启动时拒绝。

`forward_acceleration_axis=-1`默认不启用加速度融合。
NUC日志支持当前安装的axis=1、sign=+1，转换为：

`a_forward=g*(sign*accel_g[axis]-sin(pitch))`

静态倾斜只能验证重力抵消，动态前向符号必须用已知方向的速度变化另行核对。
此前sign=-1且加sin(pitch)的公式会使动态加速度反号，不应沿用。
当前本地简化没有启用融合，也没有改NUC配置或安装外参。

尚未补偿IMU杆臂、机体横向运动耦合及精确延迟，因此仍保留配置的yaw适用域，
默认abs(yaw_rate)<=0.5rad/s才用加速度。此条件是适用域检查，不是打滑门控。
速度和位移不能保证长期无漂移；双轮持续等量滑移时尤其缺乏独立约束。

## 控制接入和维护

`control/chassis_input_reader.cpp`负责原始IMU物理转换。
`balance/balance_controller.cpp::EstimateBodyMotion`负责轮速和腿运动补偿。
`wheel_contact_gate.*`负责接触条件；`body_motion_estimator.*`负责KF。
估计器不访问设备、不推断动作模式、不执行I/O。固定大小实时计算不动态分配。

沿用BalanceController::Reset生命周期：失能、故障、既有动作交接入口清空估计、
位移和接触历史。此版不新增原始/滤波速度模式硬切换，也不改变动作控制状态机。
数值输入异常会跳过相应更新；非法dt保持状态并报告无效，数值失败复位。

## 遥测和日志

本地消息版本为10，部署时需同时重建发送端和接收端。
原始IMU、传感器时间、轮速输入、支撑比例、更新前预测、协方差、最终估计和
每次是否更新仍保留。motion_wheel_speed_variance在更新时始终为配置Rv，
不更新时为NaN。残差仅作诊断，不作为门控。

新增motion_contact_parameters=[off_fraction,on_fraction,loss_delay_s]。
历史motion_reliability_parameters全NaN；prediction_gate始终关闭，gate/ratio为NaN。
状态码3/5/6及自适应证据位4..6保留供历史日志解释，当前实现不产生。

主话题`~/chassis/telemetry`发布最新帧；`~/chassis/telemetry_samples`在有订阅时
逐控制样本发布。有界队列和BestEffort不能保证无损，必须检查sequence、
queue_drops、missed_releases及采集器summary。实时线程不写磁盘。

```bash
cd /home/panpoming/dust-control
# source实际使用的ROS与本项目消息overlay后执行；本次只改本地，尚未部署NUC。
python3 tools/capture_motion_diagnostics.py \
  --config-directory config \
  --output .cache/motion-calibration/fixed-fusion-01.jsonl \
  --scenario balance_idle_fixed --condition unlabeled --duration 30
python3 tools/analyze_motion_diagnostics.py \
  .cache/motion-calibration/fixed-fusion-01.jsonl \
  --output-dir .cache/motion-calibration/fixed-fusion-01-report
```

日志工具不使能电机，不更改配置。保存所有收到的样本，包括失能和估计无效阶段。
metadata保存操作者提供的配置文本与SHA256，不假称已核对运行进程；
实际KF/contact参数以有效样本快照为准。summary记录收样、落盘和丢样。

离线分析保留version9历史自适应日志支持。version10只输出残差诊断、
接触参数、数据质量、低支撑区间和状态事件，不生成自适应阈值扫描或调参建议。
位移来自估计速度积分，不是独立真值；离线回放不等于闭环验证。
