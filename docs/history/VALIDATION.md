# 重构与验证记录（2026-10-22 起）

> 历史记录：保留当时的决策、路径、命令和结果，不作为当前使用说明。
> 当前入口见 [文档索引](../README.md)。

## 文档结构整理（2026-10-26）

- 新增 docs/README.md 阅读索引；现行架构、配置、ROS2、测试说明按当前单节点入口整理。
- USB/CAN 说明合并到 protocols/USB.md，UART 接口归 protocols/UART.md。
  迁移、参数筛选、验证记录和来源清单归 history，发布及性能结果保留历史语义。
- 修正过时的外部指令队列/仲裁描述和启动入口，更新引用与协议文档安装路径。
- 62 个本地文档链接、4 个标题锚点和 diff 检查通过；CMake 配置与协议文档安装通过。
  两份来源 JSON 清单与迁移前逐字节一致。本轮不改运行逻辑，不重跑性能测试。

## 统一 ROS2 应用入口（2026-10-26）

- 删除 standalone_main.cpp、chassis_controller 可执行与安装目标，清理已知构建目录中的旧二进制。
  应用统一由 main.cpp 启动 dust_control_node，模拟模式继续从 execution 加载。
- run.sh 默认启动构建目录中的 ROS2 节点，并补充消息类型支持库搜索路径；
  外部配置通过原生 ROS config_directory 参数指定。无 ROS 构建仅用于库与模块测试，
  缺少 ROS 的非测试生产构建明确报错。
- 应用集成回归迁入 ROS2 入口，纯 C++ 配置和双会话测试仍直接验证 Robot 及各模块。
  root、development、无 ROS 三种构建通过；development 完整 24/24，模块版 18/18 通过。
- 主构建最新完整回归 22/23，usb.lifecycle 发生 USB mock 租约断言；单项复查为
  传感器流超时，延续 OrbStack 已记录的时序不稳定，保护阈值和 mock 契约未放宽。
  初测另有消息类型支持库路径遗漏，已修复；初轮并发编译时 ROS 节点提前退出，
  最终在构建结束后回归，ros.node_integration 通过。
- 全新安装只包含 ROS2 生产入口；启动脚本与安装后的节点均能读取临时模拟 YAML，
  并通过退出信号正常回收线程。格式、Python 语法、依赖边界、shell 语法及 diff 检查通过。
- 未连接实际硬件，不进行 ROS2/Foxglove 或实时性能验收。

## 启动配置统一到 robot.yaml（2026-10-26）

- 删除 options.*，execution 严格保留 simulate、realtime、enable_output 三个布尔字段。
  字段必须完整，未知/重复字段及模拟开启真实输出在创建线程前拒绝。
- Robot 从 YAML 获取执行模式，业务模块继续接收强类型数值配置；运行循环不解析 YAML。
  CLI 只保留 --config/--help，ROS 只保留只读 config_directory；旧运行参数不再覆盖 YAML。
  删除运行期限，使用 SIGINT/SIGTERM 正常停止，脚本支持默认或外部配置目录。
- 暂不接入外部规划：移除 ROS 运动目标订阅、MotionCommand 消息、底盘目标队列与仲裁，
  控制输入保留物理遥控，ROS 底盘观测发布与其 QoS 不变。
- 初轮清理旧 ROS 消息生成缓存后，主构建、development、纯 C++ 完整回归分别
  23/23、24/24、20/20 通过。全新安装的 launch 只显示 config_directory，
  脚本和安装后的 ROS 入口可从临时 YAML 模拟运行并正常退出。
- 安装检查出现一次虚拟机 20 ms 控制看门狗失败，短时启动复查通过；修正测试工具
  向整个进程组发送退出信号，避免 ros2 run 包装进程退出后遗留子节点。
  后续主 build 回归为 22/23，ROS USB mock 租约断言失败；单项复查仍出现传感器流超时。
  最终 development 24/24、纯 C++ 20/20 通过，主 build 的 ROS USB mock 时序仍有波动。
  73/73 头文件独立编译及格式、Python 语法、依赖边界、diff 检查通过。
  保护阈值和 mock 契约均未放宽。
- 本轮不进行 ROS2/Foxglove 或实时性能验收，也未连接实际硬件。

## YAML 多线程与多 Adapter 装配（2026-10-25）

- 新增 robot.yaml，统一声明具名 Adapter、模块启用与绑定，以及业务、USB RX/TX
  线程的 CPU 和调度优先级。配置在开线程前完整校验；多板要求不同的完整序列号。
- Robot 显式装配 chassis、gimbal、ui；每块板独立拥有会话与设备状态，模块借用接口。
  每板启动后立即启动对应业务线程，全局就绪许可在全部启动成功后开放；停机先撤销许可。
- 空云台/UI 不注册虚假 CAN 生产者，没有真实 CAN 生产者的会话保持 idle，不打开或 Arm。
  双会话测试覆盖独立状态、共享板绑定、实际 CPU 亲和性及普通调度，不代表双实板验证。
- 主 build、development、纯 C++ 静态构建与独立历史基准目标均编译通过。
  完整 CTest 分别 24/24、25/25、21/21 通过；76/76 项目头文件独立编译通过。
  格式、Python 语法、依赖边界、diff 检查通过，balance fixture 与原基线逐字节一致。
- 全新安装到 /tmp/dust-topology-install 成功；安装后的 launch 参数可查询，模拟 ROS 节点
  从安装目录加载配置并按 run_seconds 正常退出。未连接实际硬件。
- 本轮不运行 ROS2、Foxglove 或控制周期性能基准；实机性能与双板硬件验证待上机完成。

## 1 kHz 最新状态与通讯性能（2026-10-25）

- 底盘每控制周期生成观测；ROS 1 ms 回调有界消费队列、只发布最近状态，
  QoS KeepLast(1) / BestEffort / Volatile。Bridge 同话题最佳努力订阅且深度限制为 1。
  诊断约 50 Hz，标准 IMU/关节话题最多 100 Hz，避免附带字符串诊断一起升至 1 kHz。
- 新增独立、默认不构建不安装的 1 kHz 忙等待测试发布者，以及包含 ROS、真实 Bridge
  WebSocket 和慢订阅者三场景的性能脚本。测试应用与生产代码分开，没有引入生产忙等待。
- 两种测试源各预热 2 秒、测量约 10 秒/场景：独立 1 kHz 源下，ROS+Bridge 同时订阅时
  ROS 997.35 Hz、样本年龄 P99 0.588 ms；WebSocket 999.63 Hz、P99 1.243 ms。
  慢 ROS 订阅者 19.04 Hz，样本年龄 P99 0.993 ms，未积压历史。
- 本机 OrbStack 普通调度下完整模拟节点端到端仅约 320 Hz；1 ms sleep 中位约 2.993 ms。
  使用 sudo、无限 memlock、原有实时配置的长对照触发 20 ms 控制看门狗，未完成测量。
  通讯链路容量通过，完整节点在此虚拟机中达到稳定 1 kHz 的结论不成立；未放宽保护阈值。
- 根 build 与单独基准目标编译通过，最终完整 CTest 23/23 通过；格式、Python 语法、
  依赖边界与 diff 检查通过。初测 ROS USB mock 有超时与 abort，单项复测后完整回归通过；
  mock 仅新增失败位置日志，未移除契约或修改租约。
- 环境、CPU、RSS、载荷、完整分位数与原始统计见
  [性能报告](../benchmarks/ros_state_1khz_2026-10-25.md)。无实际 Adapter、跨机网络或 GUI 渲染负载。

## ROS 观测发布 100 Hz（2026-10-25）

- ROS 发布回调由 20 ms 改为 10 ms，逐条发布观测队列中的样本，不主动合并。
  空队列不重复发布新鲜样本；保留源停更超过 200 ms 后的失效状态发布。
- 根 build 编译、格式与 diff 检查通过。ROS 消息转换、节点集成与真实 Bridge
  回归 3/3 通过。真实 ROS 订阅模拟节点约 1.5 秒：150 条观测，99.91 Hz，
  序号无跳变、无重复；未连接实际硬件。

## 底盘观测归属与 UDP 清理（2026-10-25）

- 删除 src/telemetry、TelemetryPublisher、UDP JSON 序列化与 --telemetry-port；
  Robot 和 ChassisRuntime 不再借用或拥有遥测发布器，移除 dust_telemetry 构建/安装目标。
- ChassisObservation、ChassisObservationQueue 归 src/chassis/chassis_observation.hpp；
  保留预分配单生产者单消费者队列，满时不等待。queue_drops 改为该观测队列的累计丢样，
  随下一次成功入队的样本上报。ROS 消息转换仍归 src/chassis_telemetry.*。
- 三种构建通过，74/74 头文件独立编译通过；格式、依赖边界、diff 检查通过。
  原队列饱和/回绕/并发测试迁为 chassis.observation_queue，删除无使用者的 UDP 测试。
- 完整回归：主 build 初测 22/23、development 初测 23/24，通过项包含观测队列、
  ROS 消息转换和真实 Bridge 多节点验证；两者失败项均为已有非实时 ROS USB mock 超时，
  单独复测最终均通过（development 中间一次复测仍超时）。未放宽看门狗或屏蔽断连断言。
  纯 C++ 静态构建完整 20/20 通过。
- balance fixture 与此前基线逐字节一致。旧 UDP 参数在启动前被拒绝；新安装目录
  无 libdust_telemetry，保留 ROS 消息与独立 Bridge launch。未连接实际硬件。

## ROS 2 统一 Foxglove Bridge（2026-10-25）

- 移除内嵌 Foxglove SDK、下载脚本、JSON schema 生成及旧 CLI/节点端口参数；
  新增独立 foxglove.launch.py，系统共用一个只读 WebSocket 入口，默认本机 8765。
- ChassisTelemetry 保留原 28 个遥测字段，通过非实时 ROS 发布回调输出；
  控制线程仍只提交有界观测，左右腿独立新鲜度、NaN、源停更及样本时间转换有契约测试。
- 主 build 23/23、development（含 MPC）24/24、纯 C++ 静态构建 20/20 通过。
  真实官方 Humble Bridge 3.6.0 从临时解包目录运行，验证两独立节点共用端口、
  自定义消息 schema/CDR、只读能力，以及两次 Bridge 启停后控制样本序列继续递增。
  系统未安装 Bridge 包，部署依赖已声明于 package.xml，安装/启动见 tools/foxglove/README.md。
- 初测非实时 ROS mock 曾触发控制源超时；将测试观察端/Bridge/DDS 负载移出 CPU 3 后，
  最终三套完整回归通过；没有放宽控制、USB 或 CAN 来源的看门狗阈值。
- 74/74 自有头文件独立编译通过；格式、模块依赖和 diff 检查通过。
  底盘 balance fixture 与此前基线逐字节一致；安装目录包含消息定义及独立 Bridge launch，
  安装后 ROS launch 参数解析与模拟节点短时运行通过。以上均未连接实际硬件。

## 云台与 UI 空线程架构（2026-10-25）

- 新增 src/gimbal、src/ui 及独立构建目标，由模块各自持有 dust-gimbal、dust-ui
  线程。当前为普通调度的条件变量等待，不空转、不注册 CAN 来源或生成设备命令。
- Robot 装配并启动底盘、云台、UI；先回收业务线程，再停止 USB，最后回收遥测。
  UI 借用底盘板的裁判 Decoder 和 UART 提交端口；云台先离线，提供第二块板的
  反馈与 CAN 端口装配接口，尚未打开第二板或使用底盘电机配置代替云台配置。
- CanCommandPort 与 UartFramePort 为非拥有能力端口，只开放注册/提交，不开放
  USB 生命周期。新合约覆盖真实线程名、Start/Stop/析构、重复启动拒绝、借用
  会话不被关闭、空云台不占 CAN 注册以及 Robot 的模拟/USB mock 装配与回收。
- 更新 Foxglove 回归，分别识别三条业务线程；SDK 线程 CPU/调度隔离校验保留。
  线程条目退出检查允许 /proc 在 join 后短暂回收，不更改生产 Stop 的同步 join。
- 三种构建通过：根目录 build 22/22、静态无 ROS/Foxglove 20/20 测试通过；
  development 其余 23 项首次通过，ROS mock 一度传感器流超时，独立重跑通过。
  73/73 头文件独立检查、格式、依赖边界和 diff 空白检查通过，默认平衡 fixture
  输出逐字节一致。全部为离线/mock 验证。

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

## 2026-10-26 多 Adapter 电机子集与跨板底盘

- 开发构建成功，`ctest --test-dir build/development --output-on-failure`：24/24 通过，42.85 秒。
- `configuration.robot_topology` 增加两板和三板同时活动的 mock 会话：独立序列号、每板电机子集、
  跨板相同 CAN 通道/ID、反馈归属、命令路由、后续板延迟 35 ms 握手、第三个启动会话 Arm 拒绝、
  运行断连及整体回收。缺失/重复电机、未知/重复板引用、缺失文件及轮电流换算不一致均在启动前拒绝。
- 初次跨板回归失败：空 CAN 命令批次触发 mock 生成非法零帧反馈，且 mock 传感器流错误地依赖命令。
  已改为每个 USB context 独立状态，并在多板模式独立产生传感器报告，保留命令序列与租约校验。
- 发现并修复生产生命周期问题：顺序停止时每板 25 ms 保护等待使其他板超过 20 ms 租约。
  现在先请求全部发送线程停止，再逐一等待和关闭会话。仅关节板在安全状态下每 10 个周期
  重新生成真实失能命令，避免沿用单板 100 周期失能间隔而缺少可续租的安全命令。
- USB 身份筛选回归曾因 0.3 秒后提前 SIGINT 打断 ROS 初始化而失败；预期启动失败的身份用例
  现在等待进程自然失败退出并核验诊断，未放宽控制看门狗、MCU 租约或传感器期限。
- 所有验证均为离线/mock；未连接真实电机、未刷写固件，尚未完成实体两/三板与目标主机实时验收。

## 2026-10-26 逐电机 Adapter 归属

- 将归属移到每台电机的 adapter 字段，地址校验使用 adapter + bus + CAN ID。
  robot.yaml 使用 schema 2、根节点 motors 引用唯一设备文件；电机文件使用 schema 5。
  移除底盘 motor_adapters 和每板 motors 文件绑定，chassis.sensor_adapter 仅选择传感器来源。
- 全量文件按 adapter 自动生成每板子集；单会话 Encoder/Decoder 明确拒绝混合多板配置。
- 两/三板实际活动 mock 回归仍通过；新增同 bus/ID 两电机的独立解码、同板冲突拒绝、
  未知/缺失/空 Adapter 归属和旧版模块绑定拒绝检查。未修改实机租约及控制超时阈值。
- 开发构建成功，完整 CTest 24/24 通过，43.01 秒；验证范围仍为离线/mock，未做实机验收。

## 2026-10-26 IMU 文件归并

- HI91 的帧解析、流式拼帧、CRC、物理量与安装映射、有效性检查、时钟恢复及快照发布
  集中到 hi91_protocol.h/.cpp，删除独立 decoder 和 time_guard 文件。
- 公共样本和板载原始采样快照集中到 imu_protocol.hpp，配置加载仍在 imu_config.*。
  目录由 10 个文件减少到 6 个；不增加没有实际用途的协议分派或编码接口。
- Hi91TimeGuard 移至设备命名空间；恢复策略仅用具名常量替换字面值，数值、CRC 恢复要求、
  时间戳、安装轴映射及新鲜度期限不变。板载 raw IMU 不覆盖控制用 HI91。
- 架构检查继续禁止控制算法调用设备服务，并禁止 USB 直接操作 HI91 帧与时间恢复状态。
- 开发构建成功，完整 CTest 24/24 通过，43.18 秒；覆盖分片、CRC、设备时间回退/恢复、
  配置实例隔离、快照并发和两/三板 USB mock 集成。未做实机验收。


## 2026-10-26 USB 协议与实现职责精修

- 将消息定义、收发编解码和 SessionPolicy 集中到 usb_protocol.hpp/.cpp，
  删除三个分散实现头；纯协议目标 dust_usb_protocol 不依赖 libusb 或设备驱动。
- 维持原有状态、线程及启停顺序；usb_link.cpp 管理底层传输，
  usb_receive.cpp 处理报告与传感器路由，usb_session.cpp 集中会话握手及启停。
- 重构前捕获并冻结 47 项报文/签名，重构后逐字节匹配；检查工具也用同一组
  报文验证旧源码快照，并确认改动 magic 会被拒绝。此处旧快照是本轮重构前
  的主机来源，不是重新读取真实 MCU checkout，不作实机兼容性验收结论。
- 开发构建通过。25 项回归中 24 项首次通过；ROS node integration 首次因
  收到遥测后 ROS 图尚未发现节点而失败，未修改产品或测试代码，单独重跑通过。
  协议边界、六路 UART、多板拓扑、发送调度及固定字节基准均通过。
- 更新协议、架构、风格文档和安装清单；架构依赖及 diff 空白检查通过。
  本次不改变线上协议，不修改或刷写 MCU。验证为离线/mock。


## 2026-10-26 统一 dust_control 根命名空间

- 自有生产、测试及独立基准代码统一为 dust_control 根，移除 dust、wbr 等旧根。
  modules、protocols、platform 收为子命名空间；遥控输入使用 dust_control::input，
  ROS 生成消息维持 dust_control::msg，CMake 具名目标别名同步更新。
- 外部生成的 bridge_layout.hpp 保留原文，在基准边界导入布局；第三方与 C ABI
  不改。架构检查约束完整限定命名空间声明，禁止旧根兼容别名。
- 开发构建及独立基准构建通过。主工程 24 项首次通过，ROS 集成首次复现既有
  NodeNameNonExistentError；测试增加 3 秒有界节点发现等待后，连续 3 次通过。
  主工程 25 项检查均已覆盖通过；独立基准 13 项全部通过。
- 平衡 fixture 的 200 行输出与调整前逐字节一致；47 项 USB 报文/签名保持一致。
  旧根协议源码快照通过同组用例，说明源码根名变化不要求 MCU 修改线上协议。
  该快照来自此前主机实现，不是本轮读取真实 MCU checkout，验证仅为离线/mock。
- 架构约束与 diff 空白检查通过。ROS 包名、消息类型、话题及通信格式保持不变。
