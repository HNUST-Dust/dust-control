# EtherCAT 装配与线程配置

`dust_control.yaml` schema 3 管理 execution、motors、ethercat、adapters、modules。
旧 USB schema 2/serial/receive_thread/send_thread 被拒绝，不隐式兼容。

```yaml
schema_version: 3
motors: motors_params.yaml
execution: {simulate: false, realtime: true, enable_output: false}
ethercat:
  pdo_file: ethercat_pdo.yaml
  thread: {cpu: 3, priority: 80}
adapters:
  base:
    uart_sensors: {remote: 1, hi91: 2, referee: 3}
    strict_hi91_crc: false
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

PDO 文件相对应用 YAML 所在目录解析；设备参数仍相对配置目录加载。
每台电机在 motors_params.yaml 声明 adapter，底盘 sensor_adapter 只选遥控与 HI91 来源。
每板独立 Decoder/序号/命令组，不同板允许同 CAN ID。所有板共享主站线程。

应用 adapters 与 PDO adapters 名字集合必须一致，数量为 1～3。
增加板时同时配置应用绑定、PDO 身份/容量及电机归属。alias/position 定位，
vendor_id/product_code 校验身份，初始 0 不允许真实启动。
传感器通道为 1～6，null 禁用，不能重复绑定。绑定需要实际 UART RX 容量。
电机需要本板 CAN RX 容量；底盘命令组数量不得超过本板 chassis tx_slots。

`simulate: true` 不创建主站/会话，只运行离线控制流程，不是动力学仿真。
模拟禁止 enable_output。`realtime: false` 禁用 FIFO 和内存锁定，但保留显式 CPU 绑定；
需要完全不绑核时配置 cpu:null。priority=0 为普通调度，1～99 为 FIFO。

模块 enabled:false 不创建线程；云台/UI adapter:null 表示离线借用接口。
底盘必须完整拥有六台电机和遥控/HI91 绑定；云台/UI 当前仍为空业务实现。
主站 Arm 是安全通信转发授权，原控制器使能流程保持不变。

配置启动时加载，无热更新。相关文件错误在申请 IgH 主站前失败。
单次会话/对象不支持重启；恢复故障需创建新应用并重新经过门控。
