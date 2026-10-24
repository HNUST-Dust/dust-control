# 启动时运行配置

主机电机映射统一由 `motors_params.yaml` 管理。MCU 独立维护 CAN 设备白名单和保护帧；
新增地址时需修改 MCU 配置并重新构建、刷写固件，主机配置修改后重启即可。

运行期配置文件：

- `motors_params.yaml`：六个电机的 CAN 总线、命令 ID、反馈 ID、轮电机 command_slot，
  以及各自的 `freshness_deadline_us`。
- `sensors_params.yaml`：HI91 和遥控的 `freshness_deadline_us`。

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

配置必须完整，拒绝未知字段、重复字段、零/负数、越界、重复 CAN 地址和不支持的
总线。ID 支持十进制或 `0x` 十六进制；总线仅支持当前网关 CAN2/CAN3。
DJI 轮电机命令组固定 0x200，slot 为 0..3，反馈 ID 必须为 0x201+slot。
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

其余几何、零点、方向、算法常量和状态机参数位于 `src/chassis/defaults`，
继续编译期生成，不属于日常运行配置。内部 motors 默认值供未传 --config 时回退；
显式文件始终在启动时覆盖实际反馈路由和执行器映射。

## MCU 保护与启动握手

主机不维护 MCU CAN 设备表或保护帧副本。USB Open 检查固件保护能力和所需
CAN 通道，按 MCU 上报的租约上限选择不超过 20 ms 的租约，确认选择固件默认
保护后再 Arm。主机不比较保护表签名，也不在本地校验 MCU 的逐 ID 白名单；
未被 MCU 允许的发送由 MCU 拒绝，主机收到数据拒绝后使链路失败。
