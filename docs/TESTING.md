# 构建与测试

```bash
./scripts/test.sh ethercat-ci  # IgH OFF，MPC OFF
./scripts/test.sh headless     # 无 ROS，MPC 与工具启用
```

原 USB 专属测试/压测随 USB 后端移出本分支。保留电机、独立设备解析器、快照、
底盘安全输出门控、观测队列和统计回归；新增 EtherCAT 原生覆盖：

- `ethercat.pdo_layout`：可变容量、混合板、映射拆分、签名、边界和带宽拒绝。
- `ethercat.configuration`：schema3/设备绑定/缺文件/旧 USB 配置拒绝，调用不申请主站的配置探针。
- `ethercat.runtime`：来源期限、旧序号、空批次、UART 独立窗口及 ACK 超时、PDO 去重与边界、
  1/3 板握手、错签名、WKC 丢失、重启、旧影像、统一 Stop 与调用线程所有权、整机模拟生命周期。
- ROS 可用时 `ros.telemetry_contracts`、`ros.ethercat_node_smoke`：消息语义及真实节点离线启动/退出。
- MPC 依原开发选项继续启用数学回归；可选原 MCU 数值对照保留。

运行时测试通过构造函数注入测试 transport；生产固定构造 IghTransport。
测试不申请真实主站，不开放电机输出。DUST_BUILD_IGH=OFF 的真实启动明确报错。

真实 IgH 编译需显式 ON 和 ecrt.h/libethercat，可与 ROS2 Humble 一起构建。
库编译/模拟测试不代表已实现配套从站，也不代替 1/3 板硬件周期、丢线、复位和本地保护验收。
运行边界与固件要求见 [运行时](ETHERCAT_RUNTIME.md)。
