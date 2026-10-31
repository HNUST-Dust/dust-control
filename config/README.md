# 启动配置

- `dust_control.yaml`：schema 3，整机 execution、EtherCAT 主站（master_index/cycle/limits）、从站身份与具名板卡/模块绑定。
- `pdo_schema.yaml`：PDO 协议唯一事实源（字段结构 + `production` 生产 profile）；生成的头文件供固件与主站共用，不再有独立 PDO 运行时文件。
- `motors_params.yaml`：schema 5，六个具名电机的 adapter/bus/协议/ID/方向/新鲜度。
- `sensors_params.yaml`：HI91、遥控新鲜度；`chassis_params.yaml`：机构和控制参数。

完整配置在启动时校验，修改后重启，无热更新。未知/重复/缺失字段直接拒绝。
旧 USB schema 和 serial/收发线程键不再支持。本分支没有 USB 回退。

PDO 的 tx 表示主机→从站，rx 表示从站→主机。CAN/UART 保留原逻辑编号，
实际 HPM5E31 外设引脚由从站映射；UART 用途由 uart_sensors 指定。
每板 vendor_id/product_code 必须来自实际身份，默认 0 只用于未配置提示，真实启动失败。

电机按 adapter + bus + ID 路由，不按配置排列顺序。当前底盘仍要求 DJI 左右轮、DM 四关节完整，
现有活动电机总线为 CAN0/3。波特率、CAN 白名单、本地保护表继续由固件编译期 YAML 管理，
普通主机 PDO 不得覆写。当前主站要求从站具有本地保护能力及 20 ms 来源看门狗。

原电机/HI91 新鲜度为 3000 us，遥控为 1000000 us；参数上限/单位校验继续由设备模块负责。
模拟不能开启真实输出，真实默认 enable_output:false；OP/握手成功也不绕过遥控及控制器门控。

见 [装配说明](../docs/DUST_CONTROL_CONFIGURATION.md)、[PDO 配置](../docs/ETHERCAT_PDO_CONFIGURATION.md)、
[从站运行契约](../docs/ETHERCAT_RUNTIME.md)。
