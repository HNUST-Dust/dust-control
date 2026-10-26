# 启动时运行配置

默认底盘电机映射由 `motors_params.yaml` 管理，dust_control.yaml 根节点 motors 指定该文件；
每台电机通过 adapter 声明所属板。
MCU 独立维护 CAN 设备白名单和保护帧；
新增地址时需修改 MCU 配置并重新构建、刷写固件，主机配置修改后重启即可。

运行期配置文件：

- `dust_control.yaml`：execution 启动方式、具名 Adapter、模块启用/绑定、业务及 USB 收发线程的 CPU/优先级；
  完整示例与启动约束见 [线程与 Adapter 装配](../docs/DUST_CONTROL_CONFIGURATION.md)。
- `motors_params.yaml`：按 CAN 总线分组的具名电机、protocol、命令/反馈 ID、轮电机 command_slot、
  输出方向 command_sign，以及各自的 freshness_deadline_us。
- `sensors_params.yaml`：HI91 和遥控的 `freshness_deadline_us`。
- `chassis_params.yaml`：机构标定、主要控制增益和整机工作限值，版本为 2。

默认值保持原基线：每个电机 3000 us，HI91 3000 us，遥控 1000000 us。
本次范围是当前控制实际使用的输入；板载 raw IMU 和 referee 数据没有参与控制，
不添加不会生效的调试阈值。MCU 的 20 ms command lease 不由这些配置修改。

```bash
./scripts/build.sh
# 离线调试：先将 dust_control.yaml 的 simulate 改为 true、realtime 改为 false
./scripts/run.sh  # Ctrl+C 停止
# 真实 USB：恢复 simulate: false、realtime: true，保持 enable_output: false
sudo ./scripts/run.sh
# 显式指定其他配置目录：
sudo ./scripts/run.sh --ros-args -p config_directory:=/path/to/config
```

脚本默认传入仓库的 config 目录。直接运行节点时，传 `--ros-args -p config_directory:=DIR`；
配置目录不可省略。加载发生在创建控制器和 USB/业务线程、提升 RT
调度之前。运行期间不读配置、不做 YAML 解析，不支持热更新。

电机配置采用 `schema_version: 5`，根节点仅包含 schema_version 和 motors 列表。
每台电机必须指定 name、adapter、bus、protocol、command_id、feedback_ids 和
freshness_deadline_us；DJI 设备还需要 command_slot 和 command_sign。
protocol: dji / dm 表示通信协议，具体型号规格仍在代码中维护。
当前底盘要求左右轮使用 DJI、四个腿关节使用 DM MIT，六个名字必须完整且唯一。
adapter 必须引用 dust_control.yaml 中存在的板；设备的排列顺序不影响映射。
设备路由按 adapter + bus + ID 校验：同板冲突拒绝，不同板允许相同 bus/ID。

dust_control.yaml 使用 schema 2，根节点 motors 指定唯一电机文件；底盘仅通过
sensor_adapter 选择遥控与 HI91 来源。底盘的 motor_adapters 列表和各板的 motors 文件
字段已移除，启动时直接从每台电机的 adapter 推导板卡集合和每板设备表。
旧版 schema 4 电机文件需将 can_buses/devices 展开为 motors 列表，每台设备填写
adapter 和 bus；dust_control.yaml 的迁移步骤见装配说明。旧版不隐式回退。
这些版本与 USB 线协议及 Adapter 配置版本无关。

配置拒绝未知/重复字段、未知/缺失/重复电机、未知 Adapter、非法协议、非法数值与越界。
轮输出方向只能为 ±1。DJI 电流/力矩换算固定在 dji_protocol.h，不再接受 YAML 的 dji 字段。
ID 支持十进制或 0x 十六进制；活动设备仅支持当前网关 CAN2/CAN3。
当前协议每台电机恰好一个反馈 ID，空列表和多个 ID 都会拒绝。
DJI 命令 ID 必须为 0x200，slot 为 0..3，反馈 ID 必须等于 0x201 + command_slot。
保护帧、波特率及租约由 Adapter 固件维护，不放进主机电机配置。
DJI 协议电流上限、DM MIT 量程由 hardware/motors 中的型号定义提供，
更换型号应同时审查编码与反馈解码；它们不是运行配置。
电机/传感器超时范围为 1..4294967294 us，保留 UINT32_MAX 作为无效年龄标记。
这些校验不替代下位机接线与协议的一致性确认。
任一文件非法或缺失时启动失败；DustControl 构造时协调校验，完整成功后才
创建配置使用者和业务线程。电机、IMU、遥控加载函数分别归属对应 hardware 模块。
配置以值传递给 Encoder、Decoder 和底盘输入检查，各实例只读自己的配置副本；
没有进程级全局配置或单次加载状态，不同配置的实例可以同时存在。

HI91 过期判定和时间基准恢复共同使用配置的 HI91 阈值。
遥控 freshness 按实际单调微秒判断，age_ms 只用于显示；与此前毫秒取整的边界
相比，非整数毫秒到达相位会有不足 1 ms 的差异。原 SBUS frame_lost 保持/失能策略
仍是独立的固定策略，没有随 freshness 参数隐式改写。

底盘 YAML 仅保留机构标定、主要控制增益和工作限值，位于本目录。
启动加载器在 chassis 模块内严格校验完整字段、有限数、正尺寸、非负增益和范围关系；
原编译期 schema 与参数头生成已移除。
稳定的状态机、滤波和观测器参数直接放在各控制器/估计器的 .cpp 中；
共享物理常量、坐标约定、关节限值与控制周期由现有 chassis_config.h 维护，默认 dt 从周期推导。
遥控中心、量程、独立开关阈值与 SBUS 丢帧策略直接归 SbusDecoder；不再保留遥控 YAML。
HI91 有效姿态范围归 Hi91Decoder，不再单独生成参数头。
底盘 YAML 在启动时加载，修改后重启即可，不需重新构建；运行中不自动重载。
应用必须从配置目录加载，脚本默认选择本目录；单独运行节点时显式传入 config_directory。显式配置目录必须包含 dust_control.yaml 及部署所引用的设备文件；默认配置共四个文件，不做缺失回退。
各控制器、运动学、VMC、气弹簧补偿和指令换算使用同一底盘配置的只读副本，
不同实例的配置互不影响。
MotorAddress 定义在 motor_config.hpp，生产代码不内置机器人接线表；
默认 MotorConfig 不包含任何电机。DustControl 始终加载显式配置文件，不使用内置接线回退。
固定协议/型号规格仍在对应的 protocol 文件中维护，接线与协议常量分开管理。
字段与数值由 motor_config.cpp 在启动时校验，不从 YAML 生成参数头。
更改接线 YAML 后重启即可；更改协议规格代码才需重新构建。
显式配置在启动时覆盖路由、编码方向和期限，修改后重启即可。
底盘转向分配使用同一电流配置推导的轮力矩预算。

## MCU 保护与启动握手

主机不维护 MCU CAN 设备表或保护帧副本。USB Open 检查固件保护能力和所需
CAN 通道，按 MCU 上报的租约上限选择不超过 20 ms 的租约，确认选择固件默认
保护后再 Arm。主机不比较保护表签名，也不在本地校验 MCU 的逐 ID 白名单；
未被 MCU 允许的发送由 MCU 拒绝，主机收到数据拒绝后使链路失败。


UART 传感器绑定在 `dust_control.yaml` 的每块 Adapter 下独立配置，例如：

```yaml
adapters:
  base:
    # 与 serial、motors、receive_thread、send_thread 同级
    uart_sensors: {remote: 4, hi91: 6, referee: 5}
```

`remote` 当前使用 SBUS 解码，`hi91` 为控制 IMU，`referee` 为裁判系统。
通道号为 1～6，`null` 表示不绑定；同一块板上非空通道不能重复。
显式设置 `uart_sensors` 时三个键都必须填写；省略整个字段兼容默认 1/2/3 接线。
校验在启动线程及打开 USB 前完成。所有通道仍保留原始块队列。
业务模块读取其所绑定 Adapter 上的设备；该配置只改变接收路由，
不改变 MCU UART 波特率、物理接线或 UI 发送时指定的通道。修改后重启生效。
传感器新鲜度仍在 `sensors_params.yaml` 配置；禁用控制传感器会使底盘缺少有效反馈。

## EtherCAT PDO 布局配置

`ethercat_pdo.yaml` 由独立离线工具加载，支持每板通道及容量配置，
不会被当前 USB ROS 节点自动加载。构建、使用和限制见 [PDO 配置说明](../docs/ETHERCAT_PDO_CONFIGURATION.md)。
