# 编译与运行

本页是仓库的日常命令入口。使用Linux（Orb Ubuntu或NUC），CMake≥3.21、Ninja、
C++20工具链、Eigen3、yaml-cpp、Python3/PyYAML。ROS节点需要ROS2 Humble；
硬件通信还需要与NUC内核匹配的IgH主站、头文件/库及设备权限。
增益生成需要NumPy/SciPy。脚本不会安装依赖、获取sudo、启动主站或使能电机。

## 选择一种构建

| 预设 | ROS | IgH | MPC/工具 | 测试 | 用途 |
| --- | --- | --- | --- | --- | --- |
| production | 开 | 开 | 关 | 关 | 实机运行 |
| hardware | 开 | 开 | 关 | 开 | 实机编译与离线回归 |
| development | 开 | 关 | 开 | 开 | Orb开发、ROS模拟及MPC |
| headless | 关 | 关 | 开 | 开 | 无ROS算法/模块测试，不生成控制节点 |
| ethercat-ci | 开 | 关 | 关 | 开 | ROS与通信模拟回归 |

构建分别写入`build/<预设>`，不要混用。含ROS的脚本自动source
`/opt/ros/${ROS_DISTRO:-humble}/setup.bash`，不要求手工source；headless除外。
并行数由`DUST_BUILD_JOBS`指定，默认2。

```bash
# Orb：编译并测试；不访问实机或使能电机
DUST_BUILD_JOBS=4 ./scripts/test.sh development
# 无ROS环境
./scripts/test.sh headless
# 已有测试构建，只重跑测试
ctest --preset development
```

NUC首次配置示例（IgH在默认搜索路径时省略两个路径参数）：

```bash
./scripts/build.sh hardware \
  -DIGH_INCLUDE_DIR=/实际路径/ethercat/include \
  -DIGH_LIBRARY=/实际路径/libethercat.so
./scripts/test.sh hardware
```

路径会保存在相应构建目录的CMake缓存中。之后`./scripts/build.sh hardware`即可。
production同理，但没有测试；不要用production预设执行test.sh。
编译前先停止使用该构建目录的控制程序，避免现场版本不明确。

## 准备独立运行配置

源码`config/`是模板，默认输出关闭；不为测试而覆盖模板。
下面的输出目录必须是新目录或空目录，已有配置直接复用。

```bash
# Orb：明确模拟、非实时、禁止输出
python3 tools/prepare_lqr14_config.py --mode observe --simulate --non-realtime \
  --output .cache/runtime-sim
# NUC：真实硬件，实时，输出许可开启，但仍由原遥控使能流程门控
python3 tools/prepare_lqr14_config.py --mode deploy --output .cache/runtime-hardware
```

生成器保持现有部署规则（腿长160–300mm、目标200mm）；请核对是否符合试验工况。
应用配置迁移：`balance.lqr14`仅保留`roll_sign`，旧`enabled/shadow_only`必须删除。
已有NUC`.cache/lqr14-deployed`可以继续用，但必须完成这项迁移。
应用execution的simulate/realtime/enable_output由YAML决定，构建预设不会改变它们。

## 启动、观测与停止

以下控制命令在前台持续运行，不返回提示符是正常的。

```bash
# Orb模拟，使用无IgH的development构建
DUST_BUILD_DIR="$PWD/build/development" DUST_CONFIG_DIR="$PWD/.cache/runtime-sim" \
  ./scripts/run.sh
# NUC，先保持遥控失能
DUST_BUILD_DIR="$PWD/build/hardware" DUST_CONFIG_DIR="$PWD/.cache/runtime-hardware" \
  ./scripts/run.sh
```

`run.sh`默认使用`build/production`和源码`config/`。仍支持直接传ROS参数；
显式传参时不再自动添加config_directory。
出现`EtherCAT adapter base ready`说明通信初始化完成，不代表闭环或新模式已验收。
保持原遥控失能确认、使能和recover流程。

另开终端启动一个Foxglove桥，使用与控制程序相同的构建目录：

```bash
DUST_BUILD_DIR="$PWD/build/hardware" DUST_FOXGLOVE_ADDRESS=0.0.0.0 \
  ./scripts/foxglove.sh
```

默认仅监听127.0.0.1:8765；远程时显式指定地址，端口可用DUST_FOXGLOVE_PORT覆盖。
Foxglove连接`ws://NUC实际IP:8765`，导入`tools/foxglove/`的布局。
脚本安装本次构建到`build/<预设>/ros-install`并加载该版本的自定义消息类型。
不要同时运行两个桥，也不要混用另一构建目录的旧消息库。

停止：先遥控失能，再在控制终端Ctrl+C；桥可独立Ctrl+C停止。
已有后台进程应先查明PID再停止，避免重复运行。

## 调整增益

只在明确修改Q/R、模型或工作点时重新生成增益，不是每次启动都要执行。

```bash
OPENBLAS_NUM_THREADS=1 python3 tools/derive_lqr14.py --output-dir .cache/lqr14-generated \
  && cp .cache/lqr14-generated/lqr14_development.h \
        src/chassis/balance/generated/lqr14_gain_samples.h
./scripts/test.sh hardware
```

在NUC可以把python3换成已安装NumPy/SciPy的`.cache/lqr14-venv/bin/python`。
生成失败不替换表；重新编译并重启控制后才生效。详细设计见[LQR14](LQR14.md)。

## 兼容旧试验目录

原`build/lqr14-hardware`不会被新预设覆盖；已有旧构建可用显式目录编译：

```bash
cmake --build build/lqr14-hardware -j4
ctest --test-dir build/lqr14-hardware --output-on-failure
tools/run_lqr14.sh .cache/lqr14-deployed
tools/run_foxglove.sh
```

最后两条是兼容转发，内部使用统一scripts入口，默认仍指向旧试验构建目录。
新部署优先采用hardware或production预设，不再创建另一套LQR专属构建流程。
Colcon用户可继续按[ROS接口说明](ROS2.md)安装运行；不要将colcon和裸CMake产物
放到同一个构建目录。

CI也调用相同的`./scripts/test.sh`：ethercat-ci/development使用ROS Humble容器，
headless使用不含ROS的Ubuntu容器。容器作业需在远程CI执行，本地验证不代表CI已运行。
