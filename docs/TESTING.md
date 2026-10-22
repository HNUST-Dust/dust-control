# 测试与构建隔离

生产：`cmake --preset production`，默认不构建测试或 MPC 开发应用。
开发：`./scripts/test.sh`，启用 MPC、开发应用和控制回归。
无 SDK：`./scripts/test.sh headless`，保持 MPC 回归但不下载 Foxglove。

Python3 / PyYAML 是参数生成依赖。额外的 WebSocket 测试客户端可单独安装：

```bash
python3 -m venv --system-site-packages .cache/test-venv
.cache/test-venv/bin/pip install -r tools/foxglove/requirements-test.txt
cmake --preset development -DPython3_EXECUTABLE="$PWD/.cache/test-venv/bin/python"
cmake --build --preset development --parallel 2
ctest --preset development
```

CMake 检测到 WebSocket 客户端时注册 `telemetry.native_websocket`，否则该测试不注册。
该项检查原生订阅、有效值、过期、源停更、端口占用及 SDK 线程 CPU/调度隔离。
其余控制契约、USB mock 生命周期/重连、遥测队列与序列化测试不依赖 Foxglove 客户端。
MPC 测试随对应模块开启，Release 下保留测试断言。

可选 `DUST_REFERENCE_CHASSIS_DIR` 指向原 MCU 的 chassis 控制源码时，还会注册
原始 MCU 数值对照。`chassis_balance_fixture` 始终在测试构建中可用，便于与迁移前
主机同输入输出比较。所有自动测试都不打开真实电机输出。

首次依赖获取需要联网。OSQP 可用 `FETCHCONTENT_SOURCE_DIR_OSQP` 指定本地已获取
的源码；Foxglove 可用 `DUST_FOXGLOVE_SDK_DIR` 指定匹配平台的已解压官方发行包。

EtherCAT 构建和回归只在 `dev/wbr_infantry_ethercat` 执行。共享 realtime 统计
测试独立放在 `tests/runtime`，不依赖下位机通讯方案。
