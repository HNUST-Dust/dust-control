# EtherCAT 主机替换验证记录

日期：2026-10-26，Linux 开发环境，ROS2 Humble。
IgH：官方 stable-1.6，版本 1.6.13，提交 `6363afe9f97ee3243e2d093d764581b004bb3a07`；用户态库在 /tmp 构建，
未安装内核模块/系统服务，未申请真实 master，未开启电机输出。

验证：

- IgH OFF、ROS OFF 的主机库/布局工具构建与原生回归通过。
- 使用真实 ecrt.h/libethercat.so、IgH ON、ROS ON 的节点/库构建通过。
- 最终 ROS 构建 13 项 CTest 全部通过，包括真实 ROS 模拟节点启动/退出，
  以及来源期限、UART 窗口/超时、CAN 事件缺口与重复、原始 IMU、
  1/3 板握手、错签名、OP/WKC 故障、复位、旧影像、接收阻塞后禁止恢复发送、统一停止。
- 无有效 vendor_id/product_code 时真实驱动在申请 master 前拒绝。
- 临时安装至 /tmp/dust-ethercat-package 成功。
- 全仓 C++ 格式检查（clang-format 18.1.8）、git diff --check 通过。
- 源码/构建依赖检查确认无 libusb、dust_usb 或 hardware/usb 依赖。

初次真实 ROS 链接时选用了官方未开启 PIC 的静态 libethercat.a，导致共享库链接失败；
改用同次官方构建生成的 libethercat.so 后通过。没有修改项目保护参数绕过失败。

边界：测试传输仅用于主机回归；没有配套 HPM5E31 固件/ESI，
未验证实体链路时序、满载带宽、SM RAM 与缓冲、CAN/UART 引脚接线、
从站本地看门狗和保护动作的实际送达。USB 固件工作区保持未修改。
