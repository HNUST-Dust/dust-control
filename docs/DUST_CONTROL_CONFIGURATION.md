# 装配与线程配置

`config/dust_control.yaml` schema 4 管理 execution、motors、ethercat、adapters、modules。
完整示例以仓库配置文件为准。未知、重复或缺失字段拒绝；不保留旧schema兼容分支。

| 配置 | 作用 |
|---|---|
| execution.simulate | 离线运行，不创建真实会话或申请主站 |
| execution.realtime | 启用线程CPU绑定、FIFO及内存锁定 |
| execution.enable_output | 整机输出许可；还需遥控、传感器和状态机条件 |
| ethercat.master_index | IgH主站索引 |
| ethercat.cycle_time_us | 必须与构建期schema一致，当前125 μs |
| ethercat.limits | PDO大小和估算线时的规划上限 |
| adapters | 1～3个具名从站，protocol固定dtg1，填写alias/position/vendor_id/product_code |
| modules | 底盘、云台、UI的开关、启动绑定与线程配置 |

六个具名电机由 `motors_params.yaml` 配置实际板卡、总线、协议、ID和方向；
DTG1要求六条路由与构建期schema匹配，当前底盘不支持将六电机分散到不同板。
底盘sensor_adapter选择遥控和HI91来源，必须具有完整的底盘电机来源。
UART5 SBUS、UART2 HI91和板载IMU由当前协议固定，不提供串口CRC或UART用途配置。

底盘按1 kHz执行。主站按125 μs交换；云台/UI独立线程目前等待停止，不接入设备端口。
线程cpu为null表示不绑定，priority=0为普通调度；非零优先级必须配置有效CPU。
realtime=false关闭实时调度和绑定，用于离线测试。

设备新鲜度在 `sensors_params.yaml` 和 `motors_params.yaml` 维护。
结构和控制参数在 `chassis_params.yaml` 维护。相对设备文件路径按配置目录解析。
配置修改后重启；PDO schema修改还必须重新构建并同步从站固件。
