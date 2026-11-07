# 配置文件

| 文件 | 用途 |
|---|---|
| `dust_control.yaml` | 运行时 schema 4：execution、主站、从站身份、模块和线程 |
| `pdo_schema.yaml` | 构建期 DTG1 布局、固定路由和启动 SDO 值；CMake 自动生成协议头 |
| `motors_params.yaml` | schema 5：六个具名电机的板卡、总线、ID、方向和新鲜度 |
| `sensors_params.yaml` | HI91 与遥控新鲜度 |
| `chassis_params.yaml` | 机构、控制与安全参数 |

PDO schema 不在启动时解析。修改后必须重新构建；布局、路由、签名与从站固件共同维护。
当前协议只有 DTG1，RxPDO=60 B、TxPDO=180 B，周期固定 125 μs。
UART5 SBUS、UART2 HI91 与板载 IMU 固定在协议中，不再配置通用 UART 绑定或串口 CRC。

运行时配置完整校验后装配，无热更新。未知字段、旧应用 schema、缺失字段被拒绝。
从站身份必须与实际设备一致，非零示例值也不表示已经完成匹配验收。
`simulate: true` 不允许 `enable_output: true`。

见 [装配说明](../docs/DUST_CONTROL_CONFIGURATION.md) 和
[PDO 配置](../docs/ETHERCAT_PDO_CONFIGURATION.md)。
