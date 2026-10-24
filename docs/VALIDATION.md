# 初次重构验证记录（2026-10-22）

## 遥控固定标定与参数生成移除（2026-10-25）

- 删除 remote_input_params.yaml/schema，中心 1024、量程 670、开关阈值
  689/1359 直接归 SbusDecoder 私有常量，类型、比较边界和丢帧策略保持不变。
- 删除最后的参数头生成入口 cmake/Parameters.cmake、generate_params.py 及相关
  构建依赖；Python3/PyYAML 仅作为项目回归依赖，ROS 自身代码生成依赖不受影响。
  遥控 freshness 仍从 sensors_params.yaml 加载，不与标定或丢帧策略合并。
- 剩余 71 项 YAML 均为启动配置。三种构建通过，根目录 build 21/21、
  development（含 MPC）23/23、无 ROS/Foxglove 静态构建 19/19 测试通过。
  默认平衡 fixture 输出逐字节一致，格式、依赖边界和 diff 空白检查通过。
  全部为离线/mock 验证。

## 电机编译期 schema 与生成依赖清理（2026-10-25）

- 删除 motor_config.schema.yaml 与 motors_params_generated.h 生成；电机 YAML
  只在启动时解析和校验，版本仍为 3。内建接线归 motor_map.h，期限与方向归
  MotorConfig，默认换算归当前 DJI 型号定义；运行配置仍覆盖实例默认值。
- 底盘默认轮力矩预算与 MotorConfig 共用同一换算基线；核对 40 项旧默认值与
  型号参数一致。移除电机、IMU、底盘的参数生成依赖及无用生成目录传播，
  遥测的 Foxglove schema 使用自身私有生成目录。仅遥控保留参数头生成。
- 三种构建通过：根目录 build 21/21、development（含 MPC）23/23、无 ROS/
  Foxglove 静态构建 19/19 测试通过；71/71 头文件独立检查通过。
  默认平衡 fixture 输出逐字节一致，三个 MCU 协议头一致，格式、依赖边界和
  diff 空白检查通过。全部验证为离线/mock。

## 底盘运行配置（2026-10-25）

- chassis_params.yaml 移至 config，保留版本 2 的 44 项字段；删除编译期 schema
  和参数头生成，清理已知构建目录中的旧生成头。脚本与 launch 通过原配置目录
  自动加载三个 YAML，显式配置缺少任一文件时启动失败。
- ChassisConfig 以只读实例副本传入控制器、输入运动学、VMC、气弹簧补偿、
  输出策略和指令仲裁；无全局可变参数。启动加载与 YAML 依赖留在底盘运行层，
  数值校验位于纯控制库。未指定配置目录时使用代码内建基线。
- 回归验证非默认几何/零偏、VMC 增益、弹簧关闭、关节限幅、倾角保护和
  实际队列指令换算，以及调用方修改后配置副本稳定、多个配置实例隔离。
  非法文件/字段、重复字段、非有限数、零尺寸、范围倒置与版本均在启动前拒绝。
- 根目录 build 21/21、development（含 MPC）23/23、无 ROS/Foxglove 静态构建
  19/19 测试通过；71/71 头文件独立检查通过。44 项内建默认值与 YAML 一致，
  默认平衡 fixture 输出逐字节一致，三个 MCU 协议头一致。格式、依赖和 diff
  空白检查通过；全部为离线/mock 验证。

## YAML 参数削减与代码归属（2026-10-25）

- YAML 参数从 202 项缩减至 75 项（不含 schema_version）：底盘 44、遥控 4、
  电机 25、传感器期限 2。迁回代码 118 项、推导 3 项、删除无使用者项 6 项。
- 稳定算法参数直接归各控制器、估计器、VMC 与输出策略的现有 .cpp；共用
  物理量、坐标约定、关节控制限值和周期归现有 chassis_config.h。
  未保留新增的 defaults 头文件。遥控开关阈值仍作为独立标定项。
- HI91 单参数 YAML/schema 移除，有效性阈值归 Decoder；SBUS frame_lost
  策略保持原值并归 Decoder。控制默认 dt 从实际周期常量推导。
- DJI 限幅与 DM MIT 量程归电机型号定义，轮反馈 ID 从槽位推导。电机启动
  配置为版本 3；底盘与遥控编译期 YAML 为版本 2，旧字段严格拒绝。
- 148 个有效底盘常量与 40 个电机参数核对一致，平衡 fixture 输出逐字节一致。
  三个 MCU 共享协议头保持一致；仅离线/mock 验证。
- 最终布局根目录 build 21/21、development（含 MPC）23/23、无 ROS/Foxglove
  静态构建 19/19 测试通过；独立 benchmark 13/13 测试通过。
  71/71 头文件独立语法检查、格式、依赖边界和 diff 空白检查通过。

## 剩余 YAML 按模块归属（2026-10-25）

- 移除 src/chassis/defaults：底盘参数与 schema 移至 src/chassis，遥控参数与
  schema 移至 src/hardware/remote，HI91 参数与 schema 移至 src/hardware/imu。
  更新参数生成依赖、MPC 注释和文档引用；参数值及生成头文件接口不变。
- 三种构建通过；development 23/23、无 ROS/Foxglove 静态构建 19/19 测试通过。
  根目录 20 项首次通过，ROS mock 首次传感器流超时，单独重跑通过。
- 遥控与 HI91 YAML/schema 内容未变，底盘生成参数头与迁移前一致；平衡 fixture
  输出逐字节一致，格式、依赖边界和 diff 空白检查通过。全部为离线/mock 验证。

## 电机 YAML 归属整理（2026-10-25）

- 电机映射、期限、轮输出方向、DJI 电流换算与限幅、DM MIT 编解码量程统一到
  config/motors_params.yaml；内建默认头也从该文件生成。删除底盘中的电机默认
  YAML、对应 schema 和全局 motor_range.hpp，schema 归电机模块维护。
- 电机配置版本为 2，完整配置在启动时校验并注入 Encoder/Decoder；底盘通过
  数值接收轮力矩预算。几何标定、反馈坐标方向和关节控制力矩上限仍属于底盘。
- 回归覆盖方向、电流限幅、MIT 量程覆盖、非有限数、非法量程、旧版本拒绝，
  以及平衡控制使用运行时轮力矩预算。默认平衡 fixture 输出逐字节一致，
  三个 MCU 共享协议头保持一致。验证仅为离线/mock。
- 根目录 build 21/21、development（含 MPC）23/23、无 ROS/Foxglove 静态构建
  19/19、独立 benchmark 13/13 测试通过；69/69 头文件独立检查、格式、依赖
  与 diff 空白检查通过。

## 运行职责分散验证（2026-10-25）

- 删除 src/runtime 和集中式 ControlRuntime。底盘线程、外部运动指令与仲裁归属
  chassis；USB 会话、原始队列和收发资源由 UsbSession 管理；遥测线程仍由
  TelemetryPublisher 管理。根目录 Robot 仅装配与协调生命周期，没有工作线程。
- 启动先完成配置校验，注册业务命令组、启动 USB 后再启动底盘；退出先回收
  业务线程，再停止 USB 发送、关闭链路并回收遥测。异常启动也按同样顺序清理。
- 新增底盘生命周期合约：独立离线运行、禁止重复启动、Stop/析构不停止借用的
  USB 发送器、会话由其所有者停止。ROS 模块状态检查为 5 ms，观测发布仍为 20 ms。
- 根目录 build 构建与 21/21 CTest、完整 development（含 MPC）构建与
  23/23 CTest、无 ROS2/Foxglove 静态构建与 19/19 CTest 通过。
- 独立 benchmark 构建与 13/13 合约、70/70 头文件独立语法检查、格式、依赖与
  diff 空白检查通过。平衡 fixture 输出逐字节一致，三个 MCU 共享协议头保持一致。
- 全部验证为离线/mock，尚未进行硬件时序验收。

## 模块配置与构造注入验证（2026-10-25）

- 删除 src/config 及集中配置库，电机、HI91、遥控分别拥有启动配置加载函数；
  公共 YAML 工具不持有状态。ControlRuntime 完整校验后才创建使用者并启动线程。
- 电机编码、反馈路由、USB 通道检查和 HI91 恢复使用实例配置副本；底盘只接收
  InputDeadlines 数值，不链接配置或 YAML 库。进程级全局配置访问和加载标志已移除。
- 根目录 build 构建与 20/20 CTest、完整 development（含 MPC）构建与
  22/22 CTest、无 ROS2/Foxglove 静态构建与 18/18 CTest 通过。
- 新回归覆盖失败加载不改变已有对象、多 runtime 共存、构造后配置副本稳定、
  电机反馈路由与编码槽位一致、微秒边界和 HI91 实例恢复阈值隔离。
- 68/68 头文件独立语法检查、格式、依赖与 diff 空白检查通过；平衡 fixture 输出
  逐字节一致，三个 MCU 共享协议头保持一致。验证仅为离线/mock。

## 硬件按设备归并验证（2026-10-24）

- motors、remote、imu、referee 各自拥有协议函数与 Decoder/Encoder，
  分别构建为独立设备模块；移除平行 devices/protocols 目录与库，USB 独立。
- DR16、VT03、CubeMars 协议完整保留并参与编译；共享有界拼帧缓冲位于 hardware。
- 根目录 build 构建与 20/20 CTest 通过，完整 development（含 MPC）构建与
  22/22 CTest 通过，无 ROS2/Foxglove 静态构建与 18/18 CTest 通过。
- 65/65 头文件独立语法检查通过；格式、依赖与 diff 空白检查通过。
- 平衡 fixture 输出逐字节一致，三个 MCU 共享 USB 协议头保持一致。
- 本轮验证为离线/mock，尚未进行硬件时序验收。

## 数据定义归属调整验证（2026-10-24）

- 移除集中 model 目录：底盘拥有 ControllerInputs，设备目录拥有反馈、目标、
  遥控与 IMU 数据定义，外设快照与对应 Decoder 相邻，CAN 帧位于 hardware。
- HI91 Decoder 仅依赖设备 IMU 样本；底盘通过受限的数据头引用获取状态，
  依赖检查继续禁止引用 Decoder、Encoder、协议和通信实现。
- 根目录 build 构建与 20/20 CTest 通过；development（含 MPC）构建与
  22/22 CTest 通过。65/65 头文件独立语法检查、格式与 diff 空白检查通过。
- 平衡 fixture 输出逐字节一致，三个 MCU 共享协议头保持一致；仅离线/mock 验证。

## 旧代码清理验证（2026-10-24）

- 根目录 build 构建通过，20/20 CTest 通过。
- 完整 development 构建（含 MPC）通过，22/22 CTest 通过。
- 独立通信 benchmark 构建通过，13/13 合约通过；它使用历史固件协议，
  不属于当前 USB v4 验收入口。
- 62/62 个头文件独立语法检查通过，clang-format 18 与 diff 空白检查通过。
- 当前每个 src 下的 C++ 实现都有正式或可选模块构建入口；全局电机映射别名没有残留调用者，旧二进制打包流程和失效交接文档已清理。
  DR16、VT03、CubeMars 作为后续设备支持保留协议实现和构建项。
- 平衡 fixture 输出与清理前逐字节一致；三个 MCU 共享 USB 协议头保持一致。
- 全部为离线/mock 验证，未进行实机时序验收。

## CAN 组归属与统一发送验证（2026-10-24）

- 根目录 `build` 的 ROS2/Foxglove 配置构建通过，20/20 CTest 通过。
- 无 ROS2/Foxglove 静态库构建通过，18/18 CTest 通过。
- 65 个头文件独立语法检查、格式与 diff 空白检查通过；平衡 fixture 输出
  与修改前逐字节一致，三个 MCU 共享 USB 协议头保持一致。
- 新增确定性调度测试覆盖同总线同 ID 所有权冲突、独立总线、最新批次覆盖、
  重放拒绝、16 帧拆包、独立来源超时、迟到目标无法复活、UART 有界队列、
  分块限速、消息过期与 CAN 槽位预留。
- 新增真实 UsbLink + mock 集成测试，云台和 UI 两线程持续提交时，底盘停顿
  仍触发整会话停止；原有重连、UART 回环、生命周期和控制回归继续通过。
- 验证仅为离线/mock；云台、UI 实际算法和硬件实时性尚未验收。

## Encoder 直接发送验证（2026-10-24）

- 当前 `build` 的 ROS2/Foxglove 配置构建通过，18/18 CTest 通过。
- 独立无 ROS2/Foxglove 静态库配置构建通过，16/16 CTest 通过。
- 用修改前的实际输出链路记录 11 种控制状态、每种 1200 个周期的 CAN 帧，
  修改后目标 → Encoder 路径的 13200 行输出逐字节一致，包含地址、载荷、
  帧序、控制状态复位结果和使能阶段完成标志。
- 新增输出目标回归覆盖禁用周期、上一周期命令不残留、清错/进入时序、
  等待反馈零目标、恢复状态轮目标和无效 Encoder 目标拒绝。
- 64 个头文件独立语法检查通过；格式和 diff 空白检查通过。
- 平衡算法 fixture 输出逐字节一致，三个 MCU 共享协议头未改动。
- 全部运行均为离线/mock，不代表真实硬件与实时性验收。

## Decoder 内快照验证（2026-10-24）

- 当前 `build` 的 ROS2/Foxglove 配置构建通过，17/17 CTest 通过。
- 无 ROS2、无 Foxglove 静态库配置构建通过，15/15 CTest 通过。
- 新增快照并发测试：三个读取者检查 30000 次发布的大状态完整性；真实设备
  解码、快照读取与重置并发，检查电机、遥控和板载 IMU 状态没有字段撕裂。
- 66 个头文件独立语法检查通过；格式与 diff 空白检查通过。
- 平衡 fixture 输出与修改前逐字节一致，三个 MCU 共享 USB 协议头未改动。
- 原有 USB mock 生命周期、会话隔离、遥控超时、UART 队列测试全部通过。
- 仅离线/mock 验证，不代表真实硬件实时性验收。

## 独立设备 Decoder 验证（2026-10-24）

- 当前 `build` 的 ROS2/Foxglove 配置构建通过，16/16 CTest 通过。
- 独立的无 ROS2、无 Foxglove 静态库配置构建通过，14/14 CTest 通过。
- 新测试直接向设备 Decoder 输入字节，覆盖 SBUS/HI91 每个字节位置的分包、
  独立实例、CRC 拒绝、重复时间戳、断流与重置，并覆盖裁判拼帧和板载样本所有权。
- USB 路由测试继续覆盖六路原始队列，并增加裁判/板载 IMU 的实际快照路径。
- 平衡算法 fixture 的 200 行输出与本轮修改前逐字节一致。
- 64 个仓库头文件独立语法检查通过，clang-format 18 和 diff 空白检查通过。
- 三个 MCU 共享 USB 协议头逐字节不变。全部运行均为离线/mock，未进行硬件验收。

## 设备与算法解耦验证（2026-10-24）

- 当前 `build` 的 ROS2/Foxglove 配置构建通过，15/15 CTest 通过。
- 独立的无 ROS2、无 Foxglove 静态库配置构建通过，13/13 CTest 通过。
- 新增设备边界测试覆盖电机反馈物理量、模式帧、轮电流限幅和发送载荷一致性；
  新增依赖检查阻止控制层引用设备协议和通讯代码。
- 平衡算法 fixture 的 200 行输出与本轮修改前逐字节一致。
- 60 个仓库头文件独立语法检查通过，clang-format 18 和 diff 空白检查通过。
- 三个 MCU 共享 USB 协议头文件与本轮修改前逐字节一致。
- 仅离线/mock 验证；此次未测真实设备时序，不代表硬件验收。

## 历史验证

以下是最初导入版本的历史结果；分支隔离后，当前 USB 分支的测试不再包含 EtherCAT。

环境：Orb Ubuntu 22.04，GCC 11.4，CMake 3.22，Linux x86_64。

- `production`：全新构建通过，默认仅构建当前底盘应用及依赖；离线启动成功。
- `development`：全部开发目标构建成功，10/10 CTest 通过，含原生 Foxglove WebSocket。
- `headless`：不含 Foxglove SDK，9/9 CTest 通过。
- 独立 CAN/USB benchmark：构建成功，13/13 离线测试通过。
- 重构前后主机 balance fixture 同输入对照：2000 tick、1200 个输出，最大差异 0。
  该项是重构前后主机程序的直接比较，不是新一轮 MCU 硬件对照。
- 55 个底盘控制、配置、MPC、EtherCAT 和共享 realtime 实现文件逐字节一致。
- 生产构建预设没有启用 MPC、EtherCAT、开发应用或测试。

测试覆盖纯控制契约、USB mock 生命周期/会话恢复、遥测队列与单位/过期、原生
WebSocket、MPC 数学与限制、MPC Link 编解码及 EtherCAT ABI/安全逻辑。

当前主机没有 IgH/SOEM 库，硬件后端没有编译或实跑；EtherCAT 验证限于 mock、
协议与安全逻辑。所有运行均为离线/mock，没有连接真实电机、刷写 MCU 或访问 NUC。
普通调度下的离线运行存在漏周期，因此本次不报告 RT 性能或最坏执行时间。
后续需在实际机器人主机验证重构版本的时序和对应硬件链路。

源码来源：`IMPORT_MANIFEST.json`；原仓库和未提交工作树保持原样。
