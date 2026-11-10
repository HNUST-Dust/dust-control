# DTG1 PDO 配置

`config/pdo_schema.yaml` 定义当前唯一协议 DTG1 的布局、区域、固定 CAN 路由与启动值。
CMake 通过 `tools/generate_pdo_schema.py` 自动生成构建目录中的 `generated/pdo_schema.hpp`。
YAML 或生成器变化会触发重新生成；不手动编辑生成结果，也不在源目录保留生成副本。

| 方向（以从站为准） | 字节数 | 映射 | 对象条目 |
|---|---:|---|---|
| RxPDO，主机→从站 | 60 | 0x1600 | 0x2050，6 × 80 bit |
| TxPDO，从站→主机 | 180 | 0x1a00 | 0x3051，6 × 80 bit + 4 × 240 bit |

TxPDO 包含六电机记录、偏移60的板载紧凑 IMU 页、偏移120的 HI91/SBUS 页。
字段位置以 schema 及生成常量为准。当前签名为 `0x44544731`，通信周期固定125 μs。
改变协议必须同步从站固件及兼容签名；仅修改主机 YAML 不会自动修改设备。

`dust_control.yaml` schema 4 在运行时配置主站索引、周期、规划预算及从站身份。
每板明确声明 `protocol: dtg1`。不存在独立 PDO 运行时容量文件或其他协议分支。
规划预算不代表 ESC 资源能力或实测实时性能。

```bash
build/headless/dust_ethercat_layout config/dust_control.yaml --json
```

工具复用应用配置加载器，输出映射、偏移、聚合字节和估算线时，不申请真实主站。
固定布局允许配置1～3块具名板卡；每个电机来源必须完整提供该板六条固定路由。
当前底盘六电机必须归属同一板，额外板可独立采集传感器，不伪造电机来源。

IgH 启动写入0x2100的签名、周期、20 ms看门狗和固定启动值，数值与清理前一致。
同步方式仍为 FreeRun，映射注册与域交换继续由唯一主站线程拥有。

2026-11-10 更换 adapter 板后，当前固定接线为左轮/左 B/D 使用 CAN3，
右轮/右 B/D 使用 CAN2。反馈容量 `[CAN0,CAN1,CAN2,CAN3]=[0,0,3,3]`。
电机 YAML、schema routes、启动 SDO 与 MCU 的白名单/固定路由必须一起更新。
旧板 CAN2 故障记录不适用于新板，不通过放宽握手或禁用保护来迁就旧固件。
