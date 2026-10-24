# 启动时运行配置

主机电机映射统一由 `motors_params.yaml` 管理。MCU 独立维护 CAN 设备白名单和保护帧；
新增地址时需修改 MCU 配置并重新构建、刷写固件，主机配置修改后重启即可。

运行期配置文件：

- `motors_params.yaml`：CAN 总线、关节命令/反馈 ID、轮电机 command_slot、
  输出方向 command_sign、电流换算，以及各自的 freshness_deadline_us。
- `sensors_params.yaml`：HI91 和遥控的 `freshness_deadline_us`。
- `chassis_params.yaml`：机构标定、主要控制增益和整机工作限值，版本为 2。

默认值保持原基线：每个电机 3000 us，HI91 3000 us，遥控 1000000 us。
本次范围是当前控制实际使用的输入；板载 raw IMU 和 referee 数据没有参与控制，
不添加不会生效的调试阈值。MCU 的 20 ms command lease 不由这些配置修改。

```bash
./scripts/build.sh
./scripts/run.sh --simulate --no-rt --seconds 1
# 真实 USB，默认禁止输出：
sudo ./scripts/run.sh --foxglove
# 显式指定其他配置目录：
sudo ./scripts/run.sh --config /path/to/config --foxglove
```

脚本默认传入仓库的 config 目录。直接运行二进制时，可用 `--config DIR`；
未指定则使用内建基线默认值。加载发生在创建控制器、USB 和遥测线程、提升 RT
调度之前。运行期间不读配置、不做 YAML 解析，不支持热更新。

电机配置采用 schema_version: 3；旧版文件需删除 dm_range、dji.protocol_current_limit
及轮电机 command_id/feedback_id，并更新版本。此版本与 USB 线协议版本无关。
配置必须完整，拒绝未知字段、重复字段、非法数值、越界、重复 CAN 地址和不支持的
总线。轮输出方向只能为 ±1，电流换算必须为有限正数。
ID 支持十进制或 `0x` 十六进制；总线仅支持当前网关 CAN2/CAN3。
DJI 轮电机命令组由型号固定为 0x200，slot 为 0..3，反馈 ID 从 0x201+slot 推导。
DJI 协议电流上限、DM MIT 量程由 hardware/motors 中的型号定义提供，
更换型号应同时审查编码与反馈解码；它们不是运行配置。
电机/传感器超时范围为 1..4294967294 us，保留 UINT32_MAX 作为无效年龄标记。
这些校验不替代下位机接线与协议的一致性确认。
任一文件非法或缺失时启动失败；Robot 构造时协调校验，完整成功后才
创建配置使用者和业务线程。电机、IMU、遥控加载函数分别归属对应 hardware 模块。
配置以值传递给 Encoder、Decoder 和底盘输入检查，各实例只读自己的配置副本；
没有进程级全局配置或单次加载状态，不同配置的实例可以同时存在。

HI91 过期判定和时间基准恢复共同使用配置的 HI91 阈值。
遥控 freshness 按实际单调微秒判断，age_ms 只用于显示；与此前毫秒取整的边界
相比，非整数毫秒到达相位会有不足 1 ms 的差异。原 SBUS frame_lost 保持/失能策略
仍是独立的固定策略，没有随 freshness 参数隐式改写。

底盘 YAML 仅保留机构标定、主要控制增益和工作限值，位于本目录。
启动加载器在 chassis 模块内严格校验完整字段、有限数、正尺寸、非负增益和范围关系；
原编译期 schema 与参数头生成已移除。
稳定的状态机、滤波和观测器参数直接放在各控制器/估计器的 .cpp 中；
共享物理常量、坐标约定、关节限值与控制周期由现有 chassis_config.h 维护，默认 dt 从周期推导。
遥控中心、量程、独立开关阈值与 SBUS 丢帧策略直接归 SbusDecoder；不再保留遥控 YAML。
HI91 有效姿态范围归 Hi91Decoder，不再单独生成参数头。
底盘 YAML 在启动时加载，修改后重启即可，不需重新构建；运行中不自动重载。
未传 --config 的直接运行使用 ChassisConfig 的内建底盘基线；通常应使用脚本或
显式 --config 加载本目录。显式配置目录必须包含上述三个文件，不做缺失回退。
各控制器、运动学、VMC、气弹簧补偿和指令换算使用同一底盘配置的只读副本，
不同实例的配置互不影响。
电机内建基线直接由 MotorConfig、motor_map.h 与 DJI 型号定义提供；
不再从 YAML 生成参数头，也不保留构建期电机 schema。字段与数值统一由
motor_config.cpp 在启动时校验。更改运行 YAML 后重启即可，更改代码内建基线才需构建。
显式配置在启动时覆盖路由、编码方向、电流换算和期限，修改后重启即可。
底盘转向分配使用同一电流配置推导的轮力矩预算。

## MCU 保护与启动握手

主机不维护 MCU CAN 设备表或保护帧副本。USB Open 检查固件保护能力和所需
CAN 通道，按 MCU 上报的租约上限选择不超过 20 ms 的租约，确认选择固件默认
保护后再 Arm。主机不比较保护表签名，也不在本地校验 MCU 的逐 ID 白名单；
未被 MCU 允许的发送由 MCU 拒绝，主机收到数据拒绝后使链路失败。
