# dust-control

Linux / ROS 2 底盘控制工程，使用 IgH EtherCAT 与 DTG1 网关通信。
通信周期为 125 μs，底盘控制为 1 kHz。当前支持 DJI 双轮、DM 四关节、
UART5 SBUS、UART2 HI91 和板载 IMU；云台与 UI 保留独立生命周期，尚无业务输出。

`config/pdo_schema.yaml` 是当前 DTG1 布局和固定路由的事实源。
CMake 自动生成构建目录中的 `generated/pdo_schema.hpp`；应用启动时加载
schema 4 的 `config/dust_control.yaml`，只配置执行选项、主站、从站身份与模块。
修改协议 schema 后需要重新构建，并同步匹配的从站固件和协议签名。

默认 `execution.enable_output: false`。OP、握手和离线回归均不代表实机验收完成。
使能仍受遥控、传感器新鲜度、状态机和启动许可约束。

## 日常流程

完整依赖、NUC硬件配置、运行配置迁移及启动停止命令见[编译与运行](docs/BUILD_RUN.md)。

```bash
source /opt/ros/humble/setup.bash
cmake --preset development
cmake --build --preset development --parallel 4
ctest --preset development
```

NUC使用hardware/production预设，无ROS测试使用headless。
控制直接执行`dust_control_node --ros-args -p config_directory:=...`，
Foxglove显式安装消息包、设置环境再执行`ros2 launch`；完整命令见上述说明。
日常编译与运行不依赖包装脚本，只有批量测试保留`scripts/test.sh`。

- [底盘代码导航](src/chassis/README.md)
- [架构与所有权](docs/ARCHITECTURE.md)
- [配置与线程](docs/DUST_CONTROL_CONFIGURATION.md)
- [PDO schema 与映射](docs/ETHERCAT_PDO_CONFIGURATION.md)
- [通信与故障契约](docs/ETHERCAT_RUNTIME.md)
- [ROS 与 Foxglove](docs/ROS2.md)、[测试](docs/TESTING.md)、[文档索引](docs/README.md)
- [专用网卡 ASPM 配置](tools/system/ethercat-aspm/README.md)
