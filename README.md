# dust-control — wbr_infantry EtherCAT

此分支使用 **IgH EtherCAT** 替代 USB，支持 YAML 配置 1～3 块 CAN/UART 网关，
通信和底盘控制均为 1 kHz。控制算法、电机协议、遥控门控及 ROS2 观测接口继续保留。
USB/libusb 源码、链接依赖和专属压测已移除，历史方案保留在 main/USB 分支。

**主机实现已接入，配套 HPM5E31 从站固件与实机验收尚未完成。**
原 USB 固件不能直接使用；默认从站 vendor_id/product_code 为 0，必须按实际身份填写。
默认 `execution.enable_output: false`；布局匹配和 OP 均不代表允许电机输出。

- [运行时、从站握手与协议契约](docs/ETHERCAT_RUNTIME.md)
- [YAML PDO 配置与布局工具](docs/ETHERCAT_PDO_CONFIGURATION.md)
- [装配与线程](docs/DUST_CONTROL_CONFIGURATION.md)
- [ROS2 与 Foxglove](docs/ROS2.md)
- [测试说明](docs/TESTING.md)、[文档索引](docs/README.md)

## 构建

Linux，CMake >=3.21，C++20，ROS2 Humble，Eigen3，yaml-cpp，以及 IgH ecrt.h/libethercat。
主机还需安装与运行内核匹配的 IgH master 并配置专用网口；本仓库不安装内核模块。

```bash
source /opt/ros/humble/setup.bash
cmake --preset production \
  -DIGH_INCLUDE_DIR=/opt/etherlab/include \
  -DIGH_LIBRARY=/opt/etherlab/lib/libethercat.so
cmake --build --preset production --parallel 2
```

离线调试在 dust_control.yaml 设置 `simulate: true`、`realtime: false`，保持输出关闭：

```bash
./scripts/run.sh
```

真实通信恢复 `simulate: false`、`realtime: true`，保持 `enable_output: false`，
先完成固件/身份/映射和无输出验收，再按现场控制门控执行真实输出验证。
配置修改后重启，不做热更新。

## 回归

```bash
# 不申请主站、不依赖 IgH/ROS：
./scripts/test.sh ethercat-ci
# MPC 与开发工具：
./scripts/test.sh headless
```

ethercat-ci/development/headless 默认关闭 IgH 供离线测试；显式 ON 可以验证真实库构建。
OFF 的真实设备启动会失败，不存在 USB 回退。Foxglove 继续使用独立 ROS2 Bridge。

源码：`src/hardware/ethercat` 主站/会话/布局/命令；`src/chassis` 控制；
`src/hardware/{motors,imu,remote,referee}` 设备协议；`src/gimbal`、`src/ui` 独立模块。
MPC 仍是可选在研模块，默认底盘不切换 MPC。
