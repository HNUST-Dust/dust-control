# 编译与运行

日常采用标准ROS2安装流程：colcon编译安装，source安装环境，再用ros2运行。
不需要手工设置LD_LIBRARY_PATH，也不使用.cache存放日常运行配置。

| 目录 | 用途 |
| --- | --- |
| config/ | 当前运行配置，随源码管理 |
| build/ros/ | ROS编译中间产物 |
| install/ | 程序、库、自定义消息和环境文件 |
| log/ | 编译日志及试验记录 |

从仓库根目录执行命令。依赖Linux、CMake≥3.21、C++20、Ninja、Eigen3、yaml-cpp、
Python3/PyYAML、ROS2 Humble和colcon；增益生成还需NumPy/SciPy。
NUC硬件构建另外依赖IgH头文件/库、匹配内核的主站及设备权限。

## 1. 编译安装

CMake参数集中在CMakePresets.json：ros-orb关闭IgH，ros-nuc开启IgH，
两者都启用ROS和测试、关闭MPC/工具。它们的binaryDir与colcon的
build/ros/dust_control一致。不要在colcon命令中直接使用目录不同的普通预设。
切换环境时重新配置；两种预设共享此安装流程，不同时在同一目录构建。


Orb离线开发：

```bash
source /opt/ros/humble/setup.bash
colcon build --paths . \
  --build-base build/ros --install-base install --merge-install \
  --cmake-args --preset ros-orb
```

NUC实机构建使用同一命令，将预设ros-orb改为ros-nuc。
若IgH不在系统搜索路径，在cmake-args后追加实际的IGH_INCLUDE_DIR、IGH_LIBRARY。
并行编译可在命令前设置CMAKE_BUILD_PARALLEL_LEVEL=4，它只控制编译并行数。

--paths .只选择仓库根包，避免递归发现旧构建目录中的包。
后续源码修改后重新执行该命令即可完成增量编译和安装。
不要同时运行同一构建目录的旧程序；Orb和NUC各自保留自己的构建产物，不复制二进制。

## 2. 测试

```bash
source install/setup.bash
colcon test --build-base build/ros --install-base install \
  --merge-install --packages-select dust_control --event-handlers console_direct+
colcon test-result --test-result-base build/ros --verbose
```

测试不申请真实主站或使能电机。测试通过不等于实机验收通过。
批量CI仍可使用scripts/test.sh；无ROS的算法验证仍可使用headless CMake预设。
这些测试构建与build/ros独立，不作为日常控制启动目录。

## 3. 核对配置

直接编辑config/dust_control.yaml和config/chassis_params.yaml。
NUC execution为simulate:false、realtime:true；默认enable_output:false。
明确允许电机输出时才改为true，仍需原遥控失能确认、使能和recover流程。
Orb模拟应明确simulate:true、realtime:false、enable_output:false。
不因构建关闭IgH就自动进入模拟。

旧配置balance.lqr14仅保留roll_sign；删除enabled和shadow_only。
当前使用7°经验腿角参考、速度Q权重1，候选非零工作点尚未启用。
配置启动时加载，修改后重启生效，本身不需要重新编译。

## 4. 启动控制程序

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dust_control dust_control_node
```

默认使用当前工作目录下的config，因此应从项目根目录启动。
需要其他配置时可显式覆盖：`--ros-args -p config_directory:=/其他配置目录`。
默认配置不存在则明确报错，不回退到安装目录中的另一份配置。
程序在前台持续运行，不返回提示符是正常的。
通信就绪后，按原遥控流程操作；停止前先遥控失能，再Ctrl+C。
本次不会自动复制旧.cache/lqr14-deployed中的现场参数；如需保留，请人工核对合入config。

## 5. 启动Foxglove桥

另开终端，同样从仓库根目录执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch dust_control foxglove.launch.py address:=0.0.0.0 port:=8765
```

本机查看可以address:=127.0.0.1。Foxglove连接ws://NUC实际IP:8765，
导入tools/foxglove中的布局。只启动一个桥，Ctrl+C独立停止。

source install/setup.bash会加载本次安装的程序、消息与动态库路径，
不需要手写AMENT_PREFIX_PATH或LD_LIBRARY_PATH。
消息修改后重新编译安装，并重启控制和桥。
如提示主题不存在，先检查：

```bash
ros2 pkg prefix dust_control
ros2 interface show dust_control/msg/ChassisTelemetry
ros2 topic list
```

包路径应指向本仓库install，而不是其他旧工作区。

## 6. 调整Q/R或模型

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/derive_lqr14.py --output-dir build/lqr14-generated \
  && cp build/lqr14-generated/lqr14_development.h \
        src/chassis/balance/generated/lqr14_gain_samples.h
```

生成成功后重新执行编译安装、测试和启动步骤。仅改Python或生成头文件而不重新
编译安装，实机增益不会改变。生成、协议和分析工具承担实际计算，继续保留。
设计与工况边界见[LQR14](LQR14.md)。

历史.cache中的配置、采集和调参产物暂不删除，避免丢失试验记录；
它们不是新流程的依赖。旧build/lqr14-hardware也不自动删除，但日常不再从它运行。

## 可选便捷脚本

脚本会打印实际执行的命令，不替代上面的原生命令说明，不修改运行配置。

```bash
./scripts/build.sh orb             # Orb离线预设，默认
./scripts/build.sh nuc             # NUC实机预设，需要IgH依赖
./scripts/launch.sh control        # 前台启动控制，读取仓库config
./scripts/launch.sh foxglove        # 另开终端，本机监听
./scripts/launch.sh foxglove 0.0.0.0 8765  # 远程查看
```

control可追加配置目录。脚本自动加载系统ROS及本仓库install环境；
仍须先核对simulate/realtime/enable_output。启动脚本不自动使能或执行recover。
控制与桥接各用一个终端，停止控制前先遥控失能，再Ctrl+C。
