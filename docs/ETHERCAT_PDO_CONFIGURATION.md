# YAML 配置 PDO 布局

> 生产布局已固定：唯一事实源是 [config/pdo_schema.yaml](../config/pdo_schema.yaml)（字段结构 +
> `production` 生产 profile），生成的 `pdo_schema.hpp` 由固件与主站共用，运行时不再加载独立 PDO 文件。
> 本文描述的是**离线布局工具/交叉测试**使用的通用 schema-1 PDO 配置格式（用于任意容量预览），
> 生产部署请用 `dust_ethercat_layout --production`。

已实现：配置解析、校验、可变布局生成、候选对象/PDO 映射、签名和线路预算。
主机 IgH 动态映射与业务交换已接入，见 [运行时契约](ETHERCAT_RUNTIME.md)。
配套从站对象字典/固件与 ESI 尚未实现。本布局预览工具本身不访问硬件。

## 使用

生产布局（来自 `config/pdo_schema.yaml`）：

```bash
cmake -S . -B build/pdo -G Ninja \
  -DBUILD_TESTING=ON -DDUST_BUILD_ROS2=OFF -DDUST_BUILD_MPC=OFF
cmake --build build/pdo --target dust_ethercat_layout --parallel 2
./build/pdo/src/hardware/ethercat/dust_ethercat_layout --production --json
```

任意容量的离线预览仍可传入 schema-1 配置文件：

```bash
./build/pdo/src/hardware/ethercat/dust_ethercat_layout tests/ethercat/fixtures/full_pdo.yaml --json
```

也可在启用 `DUST_BUILD_TOOLS` 的开发配置中构建该工具。
JSON 写到 stdout，错误写到 stderr，配置错误退出码 1、命令用法错误退出码 2。
预览输出明确标记 `hardware_accessed: false`，不是已下发硬件的映射。

## 配置项

`schema_version: 1`；拒绝未知、重复和缺失必填字段，不隐式退回其他格式。
`tx` 是主机向从站发送，对应 RxPDO；`rx` 是主机接收，对应 TxPDO。

| 配置 | 含义及当前校验 |
|---|---|
| `ethercat.master_index` | 0～255，单个主站 |
| `ethercat.cycle_time_us` | 必须为 1000 |
| `ethercat.limits` | 可省略；指定时须包含三个限制字段 |
| `limits.max_rxpdo_bytes` | 每板下行预算，默认 4096，范围 64～4096 B |
| `limits.max_txpdo_bytes` | 每板上行预算，默认 4096，范围 288～4096 B |
| `limits.max_wire_time_us` | 聚合线路占用预算，默认 800，范围 1～1000 us |
| `adapters` | 1～3 块板；名称为 1～8 个字母/数字/下划线 |
| `alias`、`position` | 各 0～65535，同一主站中配置对不能重复 |
| `pdo.can.tx_slots.chassis/gimbal` | 各 0～16；0 不映射该来源，两个键均必填 |
| `pdo.can.rx_slots` | CAN0～3 各 0～32 帧；未列出或为 0 不映射 |
| `pdo.uart` | UART1～6，分别指定 `tx_bytes`、`rx_bytes`，各 0～512 |
| `pdo.onboard_imu` | true 映射原始 IMU 区，false 省略 |

上限是本版布局编译器的规划约束，不是宣称已经验证的 HPM5E31 资源上限。
未来必须与从站能力读回结果取交集，不能仅凭此文件启动实际输出。
不同 alias/position 组合仍可能指向同一物理板，最终身份需在线核对。
每板可额外指定 vendor_id/product_code，默认 0 在真实启动时拒绝。
设备绑定、线程调度、波特率和保护帧保留各自配置职责。

例：只使用 CAN0/3、UART1/2 的一块底盘板：

```yaml
schema_version: 1
ethercat:
  master_index: 0
  cycle_time_us: 1000
adapters:
  base:
    alias: 0
    position: 0
    pdo:
      can:
        tx_slots: {chassis: 6, gimbal: 0}
        rx_slots: {2: 3, 3: 3}
      uart:
        1: {tx_bytes: 0, rx_bytes: 32}
        2: {tx_bytes: 0, rx_bytes: 128}
      onboard_imu: false
```

这会生成下行 208 B、上行 656 B；不是强制全板保留默认容量。
UART 单方向为 0 时，仍在该方向保留 16 B 头以回传反方向 ACK；
仅两方向均为 0/省略时，通道才整体移出映射。
具体 UART 确认/窗口规则仍按协议稿评审，本次只生成头字段而不实现确认协议。

## 布局生成规则

顺序固定：公共头、CAN 来源/总线、UART1～6、板载 IMU（仅上行）、诊断（仅上行）。
只对实际分配的区生成映射，偏移紧凑累加，无固定尾部填充。
默认配置为每板下行 **976 B**、上行 **1536 B**；旧 v0 提案的 48 B 下行尾部填充已移除。

- 下行公共头固定 64 B；每个启用的 CAN 来源为 `16 + 16 × tx_slots` B。
- 上行公共头固定 64 B；每条启用的 CAN 总线为 `16 + 24 × rx_slots` B。
- 每个启用的 UART 在两个方向均有 16 B 头，加本方向载荷容量。
- 板载 IMU 为 48 B；上行诊断固定 224 B，保留全板关键诊断。

候选对象：下行头 0x2000、来源 0x2010/0x2011、UART 0x2020～0x2025；
上行头 0x3000、CAN 0x3010～0x3013、UART 0x3020～0x3025、IMU 0x3030、诊断 0x3040。
未启用通道不挤占其他通道的对象索引。对象语义稳定，但容量改变会改变条目数量及影像偏移。
每对象 subindex 从 1 开始；字节数组按最多 30 B（240 bit）分成多个条目，
每映射对象最多 255 项，超过则增加下一 PDO，Rx 从 0x1600、Tx 从 0x1A00 开始。
对象/映射编号是候选 ABI，未表示已经存在于从站对象字典中。

工具返回每项名称、对象索引、subindex、bit_length 和本板方向内的字节 offset。
长度/偏移是位于主机紧凑过程影像的位置；IgH 注册后必须检查实际映射结果，
不能将它们不加验证地当作 domain 地址或 ESC RAM 地址。

`layout_signature` 是版本、周期和两方向的有序字段映射的显式小端 CRC32；
不依赖 YAML 键顺序或 C++ ABI，不包含板名和地址。同布局的不同板有相同签名。
它只验证布局一致性，不能替代协议版本、设备身份、设备配置签名或安全认证。

线路预算采用 100 Mbps、每板分方向以不超过 1400 B 数据分帧、每帧约 52 B 开销的估算。
超过所设预算时拒绝生成；默认三块相同板估计 640.32 us。
该模型不涵盖 DC/邮箱、实际分帧、传播、固件执行及调度抖动，不能替代实机验收。

## 运行时接入

主站读取同一文件，按生成的对象/PDO 映射配置 IgH，并注册每项实际 domain 偏移。
0x2100 启动 SDO、签名回显、OP/WKC 与会话握手必须全部满足才能转发。
从站须实现配套动态对象/映射与资源校验，本仓库没有可直接刷写的 ESI。
详细主从契约见 [EtherCAT 运行时](ETHERCAT_RUNTIME.md)。
