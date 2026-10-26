# 线程、Adapter 与设备装配

`dust_control.yaml` 描述 USB 板、线程和模块启动方式；`motors_params.yaml` 描述每个电机的
归属与协议参数。设备地址为 **adapter + bus + CAN ID**，接收和发送使用同一份配置。

```text
DustControl
├─ 按名字拥有 UsbSession：每板独立 USB 接收线程、Decoder 和发送线程
├─ ChassisRuntime：一个底盘线程，汇集所有底盘电机反馈并按所属板分发目标
├─ GimbalRuntime：独立云台线程，目前为空实现
└─ UiRuntime：独立 UI 线程，目前为空实现
```

模块只借用设备快照和提交端口，不枚举 USB、不解析 YAML、不关闭共享会话。
不同板的会话、序号、解码状态和调度独立；同板多个模块共用该板的收发资源。

## 板卡定义与设备归属

dust_control.yaml 使用 schema 2，默认文件见 [dust_control.yaml](../config/dust_control.yaml)：

```yaml
schema_version: 2
motors: motors_params.yaml
execution:
  simulate: false
  realtime: true
  enable_output: false
adapters:
  base:
    serial: "第一块板的完整序列号"
    uart_sensors: {remote: 1, hi91: 2, referee: 3}
    receive_thread: {cpu: 3, priority: 80}
    send_thread: {cpu: 3, priority: 79}
  left:
    serial: "第二块板的完整序列号"
    uart_sensors: {remote: null, hi91: null, referee: null}
    receive_thread: {cpu: 3, priority: 80}
    send_thread: {cpu: 3, priority: 79}
  right:
    serial: "第三块板的完整序列号"
    uart_sensors: {remote: null, hi91: null, referee: null}
    receive_thread: {cpu: 3, priority: 80}
    send_thread: {cpu: 3, priority: 79}
modules:
  chassis:
    enabled: true
    sensor_adapter: base
    thread: {cpu: 3, priority: 80}
  gimbal:
    enabled: true
    adapter: null
    thread: {cpu: null, priority: 0}
  ui:
    enabled: true
    adapter: base
    thread: {cpu: null, priority: 0}
```

`motors` 是全量电机文件的路径，相对配置目录解析，也可填写绝对路径。
每台电机在该文件中直接声明归属，例如两台关节电机使用相同 bus 和 ID：

```yaml
schema_version: 5
motors:
  - name: left_joint_b
    adapter: left
    bus: 3
    protocol: dm
    command_id: 0x000
    feedback_ids: [0x010]
    freshness_deadline_us: 3000
  - name: right_joint_b
    adapter: right
    bus: 3
    protocol: dm
    command_id: 0x000
    feedback_ids: [0x010]
    freshness_deadline_us: 3000
```

上面是片段；启用底盘时必须在同一电机文件中完整定义六个具名电机。
默认六台全部写 adapter: base；拆成两/三块板时只需修改各电机的 adapter、bus 和 ID，
并在 dust_control.yaml 的 adapters 定义对应板和序列号。无需填写底盘 motor_adapters 列表，
也无需把电机文件拆成每板一份。总线和设备的排列顺序不影响归属。
同一 Adapter 内不允许 CAN 路由冲突，不同 Adapter 可以复用相同 bus 和 ID。

启动时加载器按电机 adapter 自动生成每板设备子集；Decoder 只解析本板数据，
底盘按角色复制各板反馈；每板 Encoder 只编码本板目标并提交给本板发送器。
单会话编解码器拒绝混合多板配置。跨 USB 会话不保证 MCU 原子执行。

当前电机名称对应底盘六个角色，协议要求 DJI 轮电机、DM MIT 腿关节；云台仍为空实现，
不虚构云台电机或控制来源。真实云台设备需随实际模块实现接入。

## 传感器绑定与模块

`modules.chassis.sensor_adapter` 仅指定遥控和 HI91 的来源板，不决定任何电机的归属。
该板当前也必须有真实底盘 CAN 来源；独立纯接收板的会话语义仍待固件配套。
UI/云台的 adapter 表示借用其当前设备接口；null 表示离线接口。
enabled 为 false 时不创建对应业务模块。

UART 传感器在每块 Adapter 的 uart_sensors 配置：remote 使用 SBUS，hi91 为控制 IMU，
referee 为裁判系统。通道为 1～6，null 为不绑定，同板非空通道不得重复。
显式填写 uart_sensors 时三个键都要填写；省略整个字段沿用 1/2/3。
全部 UART 保留原始块队列；此配置不修改 MCU 的波特率、接线或 UI 发送通道。
新鲜度期限仍在 sensors_params.yaml 配置。修改后重启生效。

## 启动、线程与身份校验

simulate 为 true 时不打开 USB，以离线输入运行控制流程，不是动力学仿真。
模拟模式禁止 enable_output 为 true；真实输出还受遥控、安全状态及整机启动许可限制。
realtime 为 false 时禁用各业务/USB 线程的 CPU 绑定、实时调度和内存锁定。

每个线程填写 cpu 和 priority：cpu 为 null 时不绑定；priority 为 0 时使用普通调度，
1～99 为 FIFO 实时优先级且必须指定有效 CPU。示例 CPU 须按实际主机调整。
ROS executor/DDS 内部线程不由此配置管理。

Adapter 名称为 1～8 个字母、数字或下划线，USB 线程名为 usb-<name>-rx/tx。
serial 对应完整 USB 序列号；多板必须填写不同的完整序列号。
单板 serial 可留空，但发现多个匹配设备时不会任意选择。
可选身份字段为 product、pid、strict_hi91_crc；身份配置统一来自 YAML。

DustControl 在启动线程与打开 USB 前完成全部配置校验，包括未知/重复字段、未知 Adapter、
重复板名/序列号、电机缺失/重名/地址冲突及非法线程参数。未使用的板不启动。
仅有空 UI/云台且未注册真实 CAN 来源的板保持 configured, idle，不握手、不 Arm、不续租。

底盘线程先启动，逐板就绪后立即提交安全输出；全部所需板成功前禁止使能。
仅关节板在安全状态下每 10 个控制周期重新生成失能命令，满足 20 ms 租约，
不重放旧力矩目标、不发送虚假心跳。任一活动板或业务线程故障，整体停止。
退出先撤销许可并回收业务线程，再请求所有发送器停止，最后逐一等待并回收会话，
避免单板 25 ms 保护等待使其他板超时。部分启动失败采用相同清理顺序。

入口为 ROS 节点，只读参数 config_directory 指定配置目录；默认脚本和 launch 使用
仓库/安装目录。目录包含 dust_control.yaml、其 motors 引用文件及所需传感器/底盘文件。
不支持热更新或旧 CLI 身份参数。离线/mock 回归不替代实体多板及目标主机实时验收。

## 旧配置迁移

dust_control.yaml 从 schema 1 升级为 2：motors 文件路径移到根节点；删除 adapters 下的 motors
和 chassis.motor_adapters；chassis.adapter 改名为 sensor_adapter。
电机文件从 schema 4 升级为 5：can_buses/devices 展开为 motors 列表，
每台设备增加 adapter 和 bus，保留 name、protocol、命令/反馈 ID、期限及 DJI 槽位/方向。
DJI 电流/力矩换算固定在 dji_protocol.h，删除 YAML 的 dji 段。
旧版字段会明确拒绝；不做隐式回退。详细字段见 [配置说明](../config/README.md)。
