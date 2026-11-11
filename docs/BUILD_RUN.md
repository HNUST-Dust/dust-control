# 编译与运行

日常操作直接使用CMake、控制可执行文件和ROS命令，不经过启动包装脚本。
只有自动化测试保留`scripts/test.sh`。所有命令从仓库根目录执行。

## 依赖与构建预设

Linux（Orb Ubuntu或NUC）、CMake≥3.21、Ninja、C++20工具链、Eigen3、yaml-cpp、
Python3/PyYAML。增益生成需NumPy/SciPy；ROS应用需ROS2 Humble，实机需与内核
匹配的IgH主站、头文件/库和设备权限。以下命令不会安装依赖或配置系统权限。

`CMakePresets.json`是可直接审阅的配置参数集合；不同预设写入独立构建目录。

| 预设 | ROS | IgH | MPC/工具 | 测试 |
| --- | --- | --- | --- | --- |
| production | 开 | 开 | 关 | 关 |
| hardware | 开 | 开 | 关 | 开 |
| development | 开 | 关 | 开 | 开 |
| headless | 关 | 关 | 开 | 开 |
| ethercat-ci | 开 | 关 | 关 | 开 |

## 配置、编译、测试是三个步骤

Orb开发环境：

```bash
source /opt/ros/humble/setup.bash
cmake --preset development
cmake --build --preset development --parallel 4
ctest --preset development
```

无ROS模块验证（不生成控制节点）：

```bash
cmake --preset headless
cmake --build --preset headless --parallel 4
ctest --preset headless
```

NUC实机开发构建：

```bash
source /opt/ros/humble/setup.bash
cmake --preset hardware
cmake --build --preset hardware --parallel 4
ctest --preset hardware
```

IgH不在默认搜索路径时，首次配置显式传路径：

```bash
cmake --preset hardware \
  -DIGH_INCLUDE_DIR=/实际路径/ethercat/include \
  -DIGH_LIBRARY=/实际路径/libethercat.so
```

之后CMake缓存保留路径。production同理，但没有测试。纯源码修改后通常只需重新
build；切换构建选项、依赖或预设需要configure。编译不会启动控制程序。
编译前先停止使用该构建目录的旧控制程序，保持现场版本明确。

CI或批量验证可用`./scripts/test.sh development`或`./scripts/test.sh headless`，
它仅自动化执行上述三步；支持额外CMake参数，默认并行数为2，可设DUST_BUILD_JOBS。
CI的ROS作业使用ROS Humble容器，headless作业使用无ROS的Ubuntu容器。

## 运行配置独立于编译产物

首次复制模板到一个新的运行目录，之后直接编辑、复用该目录：

```bash
mkdir -p .cache
cp -r config .cache/runtime-hardware
nano .cache/runtime-hardware/dust_control.yaml
nano .cache/runtime-hardware/chassis_params.yaml
```

不要重复cp到已有目录。实机execution配置为：

```yaml
execution:
  simulate: false
  realtime: true
  enable_output: true
```

Orb离线运行应使用另一份配置，将simulate设为true、realtime设为false，
enable_output保持false。不要仅靠“无IgH构建”代替明确的模拟配置。

`enable_output:true`仍须满足原遥控失能确认、使能、反馈与状态机门控。
默认源码模板输出关闭。旧配置`balance.lqr14`仅保留`roll_sign`，删除enabled及
shadow_only，尤其NUC已有`.cache/lqr14-deployed`目录必须迁移。
机械参数、腿长目标、限值与运行许可来自YAML，CMake预设不会修改它们。
配置在启动时加载，修改后重启；改配置本身无需重新编译。

## 直接启动控制程序

NUC上先保持遥控失能，在控制终端执行：

```bash
source /opt/ros/humble/setup.bash
export LD_LIBRARY_PATH="$PWD/build/hardware${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./build/hardware/dust_control_node --ros-args \
  -p "config_directory:=$PWD/.cache/runtime-hardware"
```

LD_LIBRARY_PATH的构建根目录用于查找本次构建的ROS动态类型库。
出现`EtherCAT adapter base ready`表示通信初始化完成，不代表闭环已验收。
程序在前台持续运行，不返回提示符是正常的；之后按原遥控流程使能、recover。

Orb模拟使用build/development的节点和独立模拟配置，同时把LD_LIBRARY_PATH中的
构建目录换为build/development。production运行则使用build/production。
不要混用不同构建目录的可执行文件和消息库。

停止：先遥控失能，再在控制终端Ctrl+C。后台旧进程应查明PID后停止，勿重复运行。

## 直接启动Foxglove桥

另开终端，从同一仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
cmake --install build/hardware --prefix "$PWD/build/hardware/ros-install"
export AMENT_PREFIX_PATH="$PWD/build/hardware/ros-install${AMENT_PREFIX_PATH:+:$AMENT_PREFIX_PATH}"
export LD_LIBRARY_PATH="$PWD/build/hardware/ros-install/lib:$PWD/build/hardware${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
ros2 launch "$PWD/launch/foxglove.launch.py" address:=0.0.0.0 port:=8765
```

install把本次编译的消息定义、ament索引及类型支持库放到明确的位置；
AMENT_PREFIX_PATH让桥找到消息包，LD_LIBRARY_PATH让它加载匹配的动态库。
只source系统ROS会缺少本工程自定义消息，这正是之前“主题不存在”的原因。
消息定义修改后，重新编译、install并重启桥；不要复用旧目录中的消息。

确认类型可发现（不访问电机）：

```bash
ros2 interface show dust_control/msg/ChassisTelemetry
```

Foxglove连接`ws://NUC实际IP:8765`，导入tools/foxglove中的布局。
本机查看可以用address:=127.0.0.1；桥只启动一个，Ctrl+C独立停止。
本节选择的构建目录必须与控制程序一致；development/production同样替换全部路径。

## 修改Q/R或模型

只在明确修改权重、动力学或工作点时生成增益：

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/derive_lqr14.py --output-dir .cache/lqr14-generated \
  && cp .cache/lqr14-generated/lqr14_development.h \
        src/chassis/balance/generated/lqr14_gain_samples.h
cmake --build --preset hardware --parallel 4
ctest --preset hardware
```

可以用已安装NumPy/SciPy的`.cache/lqr14-venv/bin/python`替换python3。
生成失败不要替换表；重新编译并重启控制后才生效。详见[LQR14](LQR14.md)。
模型推导、协议生成和数据分析工具仍保留，它们承担实际计算，不是隐藏编译步骤的包装。

## 已有旧构建目录与Colcon

旧build/lqr14-hardware无需删除。可继续显式操作：

```bash
cmake --build build/lqr14-hardware --parallel 4
ctest --test-dir build/lqr14-hardware --output-on-failure
```

运行和桥接时将上面的build/hardware全部换成build/lqr14-hardware，
配置目录可选已迁移的.cache/lqr14-deployed。旧run_lqr14/run_foxglove脚本已删除。
Colcon安装运行见[ROS接口说明](ROS2.md)，不要与裸CMake共用构建目录。
