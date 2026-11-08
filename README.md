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

## 构建

依赖 Linux、CMake >=3.21、C++20、Eigen3、yaml-cpp、Python3/PyYAML，
生产应用还依赖 ROS2 Humble 和与内核匹配的 IgH 主站。

```bash
source /opt/ros/humble/setup.bash
cmake --preset production \
  -DIGH_INCLUDE_DIR=/path/to/ethercat/include \
  -DIGH_LIBRARY=/path/to/libethercat.so
cmake --build --preset production --parallel 2
```

离线运行前设置 `execution.simulate: true`、`realtime: false`，保持输出关闭：

```bash
./scripts/run.sh
```

## 验证

```bash
./scripts/test.sh ethercat-ci  # 原生模块；ROS 可用时包含节点离线测试
./scripts/test.sh headless     # 无 ROS，包含 MPC 开发回归
```

布局预览只读取应用配置，不访问主站：

```bash
build/headless/dust_ethercat_layout config/dust_control.yaml --json
```

- [架构与所有权](docs/ARCHITECTURE.md)
- [配置与线程](docs/DUST_CONTROL_CONFIGURATION.md)
- [PDO schema 与映射](docs/ETHERCAT_PDO_CONFIGURATION.md)
- [通信与故障契约](docs/ETHERCAT_RUNTIME.md)
- [ROS 与 Foxglove](docs/ROS2.md)、[测试](docs/TESTING.md)、[文档索引](docs/README.md)
- [专用网卡 ASPM 配置](tools/system/ethercat-aspm/README.md)
