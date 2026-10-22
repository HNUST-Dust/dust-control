# 重构验证记录（2026-10-22）

环境：Orb Ubuntu 22.04，GCC 11.4，CMake 3.22，Linux x86_64。

- `production`：全新构建通过，默认仅构建当前底盘应用及依赖；离线启动成功。
- `development`：全部开发目标构建成功，10/10 CTest 通过，含原生 Foxglove WebSocket。
- `headless`：不含 Foxglove SDK，9/9 CTest 通过。
- 独立 CAN/USB benchmark：构建成功，13/13 离线测试通过。
- 重构前后主机 balance fixture 同输入对照：2000 tick、1200 个输出，最大差异 0。
  该项是重构前后主机程序的直接比较，不是新一轮 MCU 硬件对照。
- 55 个底盘控制、配置、MPC、EtherCAT 和共享 realtime 实现文件逐字节一致。
- 生产构建预设没有启用 MPC、EtherCAT、开发应用或测试。

测试覆盖纯控制契约、USB mock 生命周期/会话恢复、遥测队列与单位/过期、原生
WebSocket、MPC 数学与限制、MPC Link 编解码及 EtherCAT ABI/安全逻辑。

当前主机没有 IgH/SOEM 库，硬件后端没有编译或实跑；EtherCAT 验证限于 mock、
协议与安全逻辑。所有运行均为离线/mock，没有连接真实电机、刷写 MCU 或访问 NUC。
普通调度下的离线运行存在漏周期，因此本次不报告 RT 性能或最坏执行时间。
后续需在实际机器人主机验证重构版本的时序和对应硬件链路。

源码来源：`IMPORT_MANIFEST.json`；原仓库和未提交工作树保持原样。
