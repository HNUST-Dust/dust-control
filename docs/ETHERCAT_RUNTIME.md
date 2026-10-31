# IgH EtherCAT 运行时与从站契约

本分支已移除 USB/libusb 后端。ROS 入口、DustControl 装配与底盘/云台/UI 端口使用
`src/hardware/ethercat`；生产构建启用 IgH。原 USB 源码可从 main/USB 分支查看。

主机代码和离线回归不等于已完成 HPM5E31 固件或实机验收。
用户提供的 HPM6750 USB 固件不能直接与本主机通信；本次没有修改该外部仓库。
下面是当前主机实现要求的从站契约，后续协议修改须同步主从。

## 构建与配置

```bash
source /opt/ros/humble/setup.bash
# 需要先安装与运行主机内核匹配的 IgH master，并配置专用网口。
cmake --preset production \
  -DIGH_INCLUDE_DIR=/opt/etherlab/include \
  -DIGH_LIBRARY=/opt/etherlab/lib/libethercat.so
cmake --build --preset production --parallel 2
```

`DUST_BUILD_IGH=ON` 找不到 ecrt.h/libethercat 时配置失败。OFF 只供离线/测试，
真实启动会明确失败，不回退其他后端。ROS 模拟模式不申请主站。

`dust_control.yaml` schema 3 包含 execution、motors、ethercat、adapters、modules。
`ethercat` 提供 `master_index`、`cycle_time_us` 与 `limits`；从站身份
（`alias`/`position`/`vendor_id`/`product_code`）在 `adapters` 下，PDO 容量来自
`config/pdo_schema.yaml` 的固定生产 profile（生成的 `ProductionConfig()`），不再单独加载 PDO 文件。
板卡集合必须一致；传感器绑定必须有对应 UART 接收容量，电机必须有对应 CAN 反馈容量，
注册命令组必须装得下 CAN 发送槽。所有检查先于主站启动。

从站身份来自实际注册身份，默认 0 是未配置标记，真实 IgH 启动拒绝它；不猜测厂商号。
周期仅支持 125/250/500/1000 us。布局预览：

```bash
cmake --preset ethercat-ci
cmake --build --preset ethercat-ci --parallel 2
./build/ethercat-ci/src/hardware/ethercat/dust_ethercat_layout --production
```

## 主站线程与启停

- 一个线程拥有 master 的申请、配置、Receive/Send 与释放；多板不并行调用 IgH。
- 按 YAML 的字段映射配置 SM2/RxPDO、SM3/TxPDO；Rx SM 看门狗启用。
  每个条目由 IgH 注册返回实际 domain 偏移，再按条目复制，不能假定本板偏移等于 domain 偏移。
- 使用单调绝对时间按 1 ms 唤醒；跳过错过的释放点，不突发补发历史目标。
  停顿达到 20 ms 终止会话；可读取 cycles/missed_periods/max_lateness_ns 统计。
- 启动等待最多 5 s，要求所有板 OP、完整 domain WKC、有效业务影像。
  全板发现后统一请求 Arm；全板确认 Active 后才启动业务来源期限并发布就绪。
- Arm 仅允许转发业务（包括安全失能 CAN），不是自动开放电机力矩；
  原 enable_output、遥控失能确认、传感器新鲜度、控制器状态机继续生效。
- 运行中任一板 OP/WKC、ABI、签名、启动标识、会话或来源期限失效，停止整个主站。
  故障不可自动重新 Arm；必须重建应用/会话并重新经过门控。
- 退出前全板影像清空并设置 Stop，统一发送 25 个 1 ms 停止周期，然后释放主站。
  传输失败时无法保证 Stop 送达；从站必须独立看门狗和本地保护。

## 版本与公共头

使用 [布局生成器](ETHERCAT_PDO_CONFIGURATION.md)产生的字段与偏移。ABI 4 的 RxPDO 公共头为 28 B、TxPDO 公共头为 32 B；ABI、周期和映射在 SDO 布局签名中校验，不在周期 PDO 中重复。CRC32 是兼容性检查而非认证。
显式小端；ABI major=4、minor=0；启动签名包含协议版本，与历史版本的 PDO 不兼容。
这是新 EtherCAT 协议，不兼容旧 MPC PDO 或 USB 包格式。

Rx control_flags：Discover=1、Arm=2、Stop=4。Stop 区不携带业务目标。
Tx business_state：Ready=1、Active=2、Protected=3；fault_flags 非零触发主机停止。
capability_mask 的 bit31 必须表示固件本地 CAN 保护和控制期限机制已安装可用。
低位沿用 CAN=bit0、UART1/2=bit1/2、IMU=bit3、UART3～6=bit4～7；
bit16～19 逐位表示 CAN0～3 可用。主机检查所有映射/注册需要的通道能力。

Rx session_id 为每次启动产生的非零 64 位随机标识；expected_boot_id 回显从站当前非零
启动标识。Tx accepted_session_id 在 Active 时必须等于主机新会话。
从站复位必须更新 boot_id；老影像不能被当作新授权。
同 session 的 Arm 是幂等维持状态，不延长任何来源期限，不自动解除 Protected。

公共头历史名称 configuration_signature 在当前实现承载布局 CRC32，
不是 CAN 保护表签名。原设备白名单与安全保护仍须由从站独立维护，不能因布局一致而绕过。
Tx publish_sequence 每次发布新影像递增；重复影像不重新解析设备，也不刷新采样时间，
连续 20 ms 无新影像停止。CAN 批次、UART 块、IMU 样本各有独立序号。

## 从站启动对象 0x2100

以下 u32 对象由 IgH 作为启动 SDO 配置写入；下载失败/拒绝会阻止正常 OP/业务握手。

| subindex | 含义 |
|---:|---|
| 1 | layout_signature |
| 2 | cycle_time_us，必须为 1000 |
| 3 | 控制看门狗上限，20000 us |
| 4～5 | chassis/gimbal CAN tx_slots |
| 6～9 | CAN0～3 rx_slots |
| 10～21 | UART1～6 tx_bytes/rx_bytes，逐路交替 |
| 22 | onboard_imu，0/1 |

从站须先提供支持上限内的候选对象/映射能力；容量 SDO 不是让主机在从站中创建任意对象。
IgH 对 SDO 与 PDO 的配置顺序不作为创建对象的前提。固件在全部参数与实际映射一致后
回显签名并进入 Ready，拒绝超资源、版本不兼容或局部配置。与主机生产输出分离。

从站必须实现 0x1600/0x1A00 系列动态映射、0x1C12/0x1C13 分配及工具输出的对象。
SM 长度、三缓冲 RAM、邮箱与 EEPROM/ESI 应由固件工程验证。本仓库不提供虚假的可刷写 ESI。
首版未启用 DC，避免把主机单调时间误当 EtherCAT 纪元时间。

## CAN

固定两来源分区（可由 YAML 设槽数为 0 省略）。来源注册禁止命令组冲突；
SubmitCan 同步复制最新完整批次，拒绝旧序号、未来时间、过期目标和未注册的 bus/ID。
每来源序号仅在业务线程生成新目标时推进；周期交换序号、UART 流量不续控制期限。
空批次可表达来源活性，不重放上次 CAN。

接纳新 source_sequence 才向 CAN 派发一次；重复 PDO 不重复派发。
remaining_valid_us 已扣主机等待时间；从站首次接纳时建立截止时间，同序号不可延长。
active_source_mask 在 Arm 时锁定；来源首帧等待不超过 20 ms，任何来源失效触发整板保护。
本地保护表和波特率沿用编译期 YAML 职责，不由普通 PDO 改写。

上行 CAN 按通道和标准 ID 保留最新帧，每个已配置 ID 独立覆盖；超过 20 ms 未更新的槽失效。主机按通道和 ID 去重并检查回绕，重复序号的帧内容必须一致。中间状态被覆盖导致的序号跳变属于正常行为，不是丢帧；实际接收队列溢出等 drops 变化仍会重置本板电机解码状态。PDO 中有效帧紧凑排列，槽位位置可变化，必须按 ID 识别。该机制不保证逐命令应答；角度解绕需要确保两次观察间角度变化没有歧义。

每个映射的 CAN 通道在 adapter 侧配置的过滤 ID 数不得超过 YAML 的 rx_slots，否则从站拒绝 PDO 映射。RxPDO 的来源、期限、白名单及 Stop 保护不因本项改变。当前 ABI 4 仅离线验证，联机验收尚未执行。

## UART 与 IMU

UART flags：data_valid=1、ack_valid=2、discontinuity=4。
块序号从 1 开始，u32 按半空间规则比较和回绕，ACK 有效位区分初始 0。
每路独立 4 块窗口，累计确认，发送端在未确认窗口内按序循环发送；
接收端只交付期望下一块，重复不重复写串口/解析，缺口不推进 ACK。

每路主机消息 FIFO 16 条、每条最多 512 B；分块大小来自 YAML。
尚未开始的过期消息可丢弃；已部分转交窗口的消息或在途块过期，终止整机会话，
不拼接旧消息尾部与新消息头部。首次发送后 20 ms 无对应 ACK 也终止会话。
从站确认前须预留完整块输出空间；UART 活动不得维持 CAN 来源期限。
接收 discontinuity 表示固件明确丢块/重置流，主机通知原设备解析器丢弃半帧。

slave_now_us/capture_time_us 来自同一从站时钟，差值必须小于 20 ms；
主机保守加 1 ms 周期传输年龄，真实单程误差还需实机测量，不能宣称时钟已同步。
原电机/HI91 的 3 ms 新鲜度不放宽。板载 IMU 保留 25 字节原始样本、温度和序号，
不取代 HI91；从站本地温控不由通信会话停机替代。

## 状态查询与诊断

每周期读取 224 B 诊断，可通过 Session::diagnostics() 复制快照。
RequestStatus(target, channel, page) 请求原 USB 等价的详细状态分页，target=0会话/1CAN/2UART，
page=0..63，线上的 status_target = (page << 2) | target。
从站在处理后回显 status_reply_sequence，回复位于诊断最后 16 B：result:i32 + 3×u32。
Session::status_reply() 读取结果；接纳与 CAN/UART 实际完成必须区分。

第一页以原 USB 状态结构字段顺序每页 3 个 u32 排列：
会话从 armed/configured/stop_reason 开始，后续 rejected/can_drops/sensor_drops、
stop_errors/reply_drops/transport_send_errors、transport_receive_drops/endpoint_errors/protection_signature、
max_lease_us/0/0；CAN 依次 received/submitted/completed、errors/dropped/queue_depth、
controller_state/tx_errors/rx_errors；UART 为 received_blocks/dropped_blocks/completed_writes、tx_errors/0/0。
无效页/通道返回负错误码（建议 -EINVAL）；具体固件错误枚举后续统一。

## 验证边界

原生测试使用可注入的测试传输验证单线程、多板、错签名、WKC 丢失、重启、
旧数据、来源期限、UART 窗口和整机离线生命周期。测试后端只在测试中构造，生产固定 IgH。
真实 ecrt.h/libethercat 编译检查与硬件交换验收分别记录；没有从站固件时不能进行后者。

## 首版 HPM5E31 FreeRun 选择

动态配置同时以 u16 下载 `0x1c32:1=0` 和 `0x1c33:1=0`，显式选择 FreeRun。
未下载时 SSC 可能自动选择 SM Sync，不能进入只支持 FreeRun 的网关周期。
主站选型和实测配置见 [最终方案](2026-10-30-master-selection-igh.md)。

## 控制频率与交换频率分离

底盘控制线程继续按1kHz计算，EtherCAT独立线程可通过PDO YAML的
`ethercat.cycle_time_us`选择1000、500、250、125µs。默认1000µs保留；
不要把名义交换频率等同于CAN命令频率或已通过实机验收的频率。
周期值贯穿启动SDO0x2100:2、布局CRC、RxPDO偏移40和主站定时循环；
从站拒绝报头与协商周期不一致的报文，20ms看门狗与命令来源期限没有缩短或取消。
IgH同时设置send interval。采样时间推算的保守运输余量也使用配置的交换周期。

相同CAN batch_sequence在多个PDO周期内保持不变时，从站只派发一次CAN发送；
所以通信线程加速不会把1kHz控制指令重复发送成4kHz。
Active时从站以10µs短忙等轮询SSC，不再依赖100µs tick休眠；ISR仍可抢占，非Active阶段继续休眠。
此方式提高MCU占用，尚需全UART、多板负载下验证。正常控制线程使用独立CPU/优先级配置，
高频交换时应避免与耗时控制计算挤在同一个CPU。

主机布局编译器另外检查估算的总线上线时间不超过配置周期；多板/大PDO可能不允许125或250µs。
布局能编译不代表调度、PCIe或ESC延迟满足周期，必须观察实际WKC与端到端时间。
实测数据与运行约束见 [最终方案](2026-10-30-master-selection-igh.md)。

快照修复后的125µs台架已通过1000条失能命令，平均RTT约934µs、P99约1000µs、最大1124µs；
尚未通过每次反馈低于1ms或满负载1kHz命令验收，后续选型对比结果见最终方案。
