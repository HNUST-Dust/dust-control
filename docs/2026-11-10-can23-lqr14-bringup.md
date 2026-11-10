# 2026-11-10 新板 CAN2/CAN3 与 LQR14 接入记录

主机已完成 LQR14 的显式配置接入、高度投影反馈、支撑力映射、观察模式、
双支撑/模型域门控与遥测 version 2。默认配置仍关闭新控制和电机输出。
实现提交 `2a8c1e9`，裸 CMake ROS 启动修复 `5946d5c`，CAN 接线提交 `7eae44a`。

用户更换 adapter 板并要求左侧 CAN3、右侧 CAN2。按每侧轮电机和 B/D 共用总线执行：

| 设备 | CAN | 命令 ID | 反馈 ID |
| --- | --- | --- | --- |
| 左轮 | 3 | 0x200 | 0x201 |
| 左 B | 3 | 0x000 | 0x010 |
| 左 D | 3 | 0x003 | 0x013 |
| 右轮 | 2 | 0x200 | 0x201 |
| 右 B | 2 | 0x001 | 0x011 |
| 右 D | 2 | 0x002 | 0x012 |

两端固定路由、固件白名单、发送线程、CAN2 IRQ 优先级、反馈容量和启动 SDO 已同步。
CAN RX 容量为 `[0,0,3,3]`。主机增加启动容量与 routes 不一致拒绝检查。

## 编译与烧录

- Mac 固件工程：`/Users/panpoming/Documents/zephyr_projects/dust-adapter-ws/dust-adapter`，
  分支 `ethercat`，接线提交 `33b7be1`。
- 构建目录：`build/can23-20261110`，默认 `prj.conf` 和 `config/can_devices.yaml`，
  `CONFIG_ECAT_CAN_ISOLATION` 未启用。
- HPM5E31，CMSIS-DAPv2 VID/PID `1209:6666`，USB 序列 `Vllink.Basic2.3091FD7A76`。
- `west flash --skip-rebuild --verify -d build/can23-20261110` 已成功完成，
  193156 字节读回校验通过。启动保护开启，命令看门狗仍为 20000 us。
- HEX SHA256：`786dfe15f14776882924e8ed221a466e320c76d821f2c3a432758b8c01f1ca91`。
- ELF SHA256：`df4b1160cf9750ca755a9b2051de18d58931d15a7254d49eed30bcd1a659c6ec`。

NUC 的 `build/lqr14-hardware` 包含真实 IgH 和 ROS 2，28 项 CTest 全部通过。
新接线观察配置 `.cache/lqr14-observe-can23` 明确 `enable_output=false`，
实时线程已按原 CPU/FIFO 配置启动。账号已有 rtprio=80、memlock=unlimited，
本次不需要 sudo。源目录和原 `build/production` 未替换成调试配置。

## 实机观测结果与未完成项

烧录前从站是 HPM 工厂示例身份 `0x0048504d:0x00000001`，与主机不匹配。
烧录后已变为所需 `0x00000d57:0x44544331`、revision `0x00010000`，回到 PREOP。

输出关闭的实时启动仍在 PREOP→SAFEOP 被拒绝：

- IgH AL 状态 `0x0025 Invalid Output Mapping`，域工作计数 `2/3`。
- 启动对象读回：signature `0x44544731`，CAN0 RX=0、CAN2 RX=3，与主机一致。
- RTT：`Gateway: mapping rejected: commit=3`，该错误为 `Error::Capacity`。
- 对应 ELF 调试符号的 MCU 内存只读检查：`Limits.can_mask=0x0f`、`Limits.imu=false`。
  CAN2 能力正常；固定布局要求 `imu=true`，而板载 IMU 初始化未通过。
- IMU 最后一次事务 TX 为 `0x81`（读寄存器 0x01），接收缓冲仍为 `0x00`，
  驱动预期识别值为 `0x6a`。此信息不能单独区分芯片型号、接线或 SPI 传输问题。
- 新板两颗 PHY 识别均为 RTL8201F；当前 BSP 预期第二颗为 TJA1101B，
  该端口配置失败后保留 Port0 工作。

没有通过修改能力标志、伪造 IMU 样本或绕过布局检查来强行进入 OP。
尚未取得正式六电机/IMU观测样本，不能声明电机反馈或 LQR14 实机平衡已验证。
所有观测程序均已停止，主站回到 Idle，电机输出从未开启。

下一步需确认新板型号、板载 IMU 型号及 SPI3/CS=PC21/DRDY=PC18 接线，
按真实硬件适配 BSP/驱动后重新做输出关闭的链路预检，随后再进行支撑条件下的控制验收。
