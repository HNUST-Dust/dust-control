# HPM5E31 首轮实机验证

独立工具 `dust_ethercat_discover` 使用与主程序相同的 YAML 布局生成器，直接调用 IgH。
只接纳一块 alias=0、position=0、VID=0、PID=0x44544331、revision=0x10000 的实验网关。
不链接机器人控制器，默认只发送 Discover；拒绝任何 CAN/UART TX 容量和板载 SPI IMU。
显式添加 `--receive-only` 可在 Discover 后进入零发送容量的 Arm，仅接收外设数据。
生产运行时继续拒绝 VID=0，不能用此工具结果替代正常控制运行验收。

构建（需本机 IgH 用户态库和 yaml-cpp）：

```sh
cmake -S . -B build/bench -G Ninja \
  -DDUST_BUILD_ROS2=OFF -DDUST_BUILD_MPC=OFF \
  -DDUST_BUILD_TOOLS=ON -DDUST_BUILD_IGH=ON -DBUILD_TESTING=ON \
  -DIGH_INCLUDE_DIR=/usr/local/include \
  -DIGH_LIBRARY=/usr/local/lib/libethercat.so
cmake --build build/bench --target dust_ethercat_discover
./build/bench/src/hardware/ethercat/dust_ethercat_discover config/ethercat_bench.yaml
```

运行前先断开电机动力，连接网线与调试器。首次空载台架使用固件仓库专用配置：

```sh
# 在 dust-adapter 仓库执行；保留正常控制固件的默认 CAN 白名单/保护。
west build -b dust-hpm5e31 -d build/ethercat_bench -- \
  -DDUST_ADAPTER_CAN_CONFIG="$PWD/config/can_bench_rx_only.yaml"
west flash -d build/ethercat_bench
```

该 MCU 配置只有 CAN 反馈过滤器，没有 CAN 发送地址或保护帧；它用于无 CAN ACK 的通信台架。
正常默认配置仍会在初始化提交本地停止帧；电机不供电时可能触发 CAN 故障并拒绝 OP。
主机 Discover 工具只允许各 CAN/UART TX 容量为 0，不产生业务控制帧。
启动计数使用保留 Flash，刷写不要全片擦除或覆盖该区域。

工具核对在线身份后下载 22 项配置、动态映射并运行名义 1 kHz 周期。
15 秒内收到至少 5000 个不同发布序号、逐次匹配只读状态请求序号的 Ready 回复才通过，
状态请求只读取网关状态；Discover 阶段 accepted_session_id 必须为 0，只有 Arm 后才接纳会话。
核对 ABI、布局签名、非零且稳定的 boot_id 和本地保护能力位；握手后丢失 OP/WKC 则失败。
这是通信功能检查，不是主机调度延迟/时钟抖动的硬实时验收。
退出释放 master，不自动恢复业务；从站重新配置通常需要显式复位后再运行。

2026-10-27 预检记录：

- wbr-nuc，内核 5.15.0-1114-realtime；IgH CLI 显示 1.7.0。
- 专用口 MAC 54:b2:03:9e:f2:fa，链路 UP，一块 PREOP 从站；检查时无丢帧/发送错误。
- 在线板仍是旧计数器：VID=0、PID=0x57425231，名称 WBR HPM5E FreeRun。
- ESC RAM 寄存器报告 60 KiB；用户属于 ethercat 组，可以访问 /dev/EtherCAT0，无需 sudo。
- 工具已用 NUC 的真实 IgH 头/库编译，并在映射前正确拒绝旧 PID。
- 已刷入新网关，确认 PID=0x44544331、动态 PDO 配置能力及 SDO 对象 0x2100:0=22。
- 修复 HPM Flash 未声明显式擦除，解决 NVS 首次初始化 ENOSYS。
- 修复主机未显式下载 FreeRun：0x1c32:1/0x1c33:1 均以 u16 写 0；否则 SSC 自动选 SM Sync。
- YAML 对应 Rx=96 B、Tx=672 B，已到 SAFEOP，过程域 WKC=3/3；这不代表已完成应用握手。
- 默认电机配置在无 ACK 的 CAN 上触发保护；另建 MCU 只接收台架配置，不放宽默认保护。
- 后续刷写/诊断期间出现启动异常，最后 NUC 链路 DOWN、从站数 0，JTAG 无法暂停 CPU。
  已请求控制板单独断电重启，保持电机动力断开；尚未完成 OP/Discover 持续交换。
- 没有发送 Arm，没有进行 CAN/UART 输出或断线保护实测。

2026-10-27 时钟修复后复测：

- 已烧入 clockfix-514ab7c RX-only 镜像；修正 PLL0=480 MHz、DCDC=1275 mV 与 LMM 时钟。
- 单块从站已进入 OP，过程域 WKC=3，15 秒窗口内获得 14290 次完整工作计数，boot_id 稳定。
- 首版工具错误要求 Discover 回显 accepted_session_id；固件只在 Arm 后设置此字段。
  已改为 Discover 携带只读网关状态请求，以不同请求序号逐次核对回显，同时要求接纳会话仍为 0。
- 修正后的端到端复测尚未通过：上次退出后配置锁定，复测停在 PREOP，需控制板重启。
  当前探针序列号为 Vllink.Basic2.3091FD7A76，JTAG 扫描全 0，不能通过调试器复位。
- 全程未发送 Arm 或 CAN/UART 控制输出。以上结果尚不能替代双向 Discover 验收。

2026-10-27 控制板断电重启后，双向 Discover 验证通过：

- 主机工具提交 f3fa98e，在 NUC 上使用真实 IgH 库重新编译；配置为 `config/ethercat_bench.yaml`。
- 单板 Port0、FreeRun、名义 1 kHz；动态 Rx=96 B、Tx=672 B，布局签名 3868350841。
- 5000 次不同发布序号的 Ready 应答逐次匹配只读网关状态请求，工具正常退出，返回码 0。
- 验证期间 OP、WKC=3，首次有效应答后 OP/WKC 丢失计数为 0；
  boot_id=7198499774940554477，稳定且非零，accepted_session_id 始终为 0。
- 原始输出见 [测试日志](bench-results/2026-10-27-discover.txt)。请求回显采用逐次等待，
  约每两个 1 ms 主机周期完成一次，不能据此推断端到端延迟或硬实时指标。
- 全程未 Arm，CAN/UART TX 容量为 0；尚未验证业务数据、断线保护、多板及 Port1。
  下一阶段先准备外设输入台架与专用验证工具，逐项核对 CAN 反馈和 UART 接收数据。

仅接收验证（须明确授权接收 Arm，电机动力保持断开）：

```sh
./build/bench/src/hardware/ethercat/dust_ethercat_discover \
  config/ethercat_receive_bench.yaml --receive-only
```

此配置按实际接线启用 CAN2/3、UART2 HI91（115200）与 UART5 SBUS（100000、8E2）；
CAN0/1 仍分配反馈容量以便观测，但 MCU 当前过滤表为空。
工具强制所有 CAN/UART TX 容量为 0，来源掩码为 0，仅发协议头、只读状态请求和 UART 接收 ACK。
先 Discover 核对启动标识、保护能力与请求回显，再带 expected_boot_id 发 Arm。
仅在匹配会话的 Active 应答中消费外设数据，并按 YAML 生成的字段偏移更新累计 UART ACK。
5000 次 Active 请求回显通过只代表协议状态正常；各通道必须另行核对实际收到的帧和内容。
退出后主站释放，从站保护/配置冻结；复测前通常需重启控制板。

2026-10-27 首轮接收结果（[原始输出](bench-results/2026-10-27-receive.txt)）：

- 动态 Rx=96 B、Tx=736 B，签名 855855321；5000 次 Active 状态请求回显通过，OP/WKC 丢失 0。
- boot_id=7198499774940554464，全程稳定；所有外设 TX 容量为 0，没有发送外设控制帧。
- CAN3 共 10040 帧，前三帧 ID 均为 513（0x201）；CAN2 为 0。尚未证明全部关节反馈正常。
- UART5 共 17850 字节，前三个完整 25 字节 SBUS 抽样帧头 0x0f、帧尾 0x00。
  flags=0x0f：frame_lost 与 failsafe 均置位；接收链路有数据，但不能作为有效遥控输入验收。
- UART2 共 57728 字节；首个 128 字节抽样未见 HI91 0x5a 0xa5 帧头，未通过 HI91 解码验收。
  等待核对传感器实际波特率，不能仅凭字节数判定 IMU 正常。
- 等待核对右腿控制电源/主动反馈条件；不为通信测试接通电机动力。
