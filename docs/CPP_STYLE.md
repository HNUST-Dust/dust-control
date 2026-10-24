# C++ 编码风格与维护约定

本仓库采用 2026-10-24 的 `dust-adapter/docs/CPP_STYLE_HANDOFF.md` 约定。
以可读性和可维护性为标准，不以缩短代码、增加文件或增加抽象为目标。

## 排版与维护

- `.clang-format`：Google 基础、80 列、4 空格缩进、禁用 Tab。
  `Standard: c++17` 是格式解析选项，项目编译标准仍由 CMake 指定为 C++20。
- 非空短函数展开，`if/else/for/while` 均使用大括号。
- 不同含义的变量独立声明；局部变量靠近使用点并缩小作用域。
  跨循环状态放循环前，不以循环内 `static` 隐藏状态寿命。
- 函数与完整逻辑单元之间适当留白；较长注释放语句上方。
- 类型、常量、实例和函数按依赖与职责组织，内部实现放匿名命名空间。
  拆文件应有职责或所有权依据，不为单个转调函数新增封装。
- 函数表达动作和对象，常量使用 `k` 前缀，变量使用 snake_case。
  保留外部 API 与既有公共接口名称，内部操作可按职责改名。
- 说明性注释使用中文，保留 SPDX、Doxygen 标签、代码名称和技术术语。
  优先解释约束、原因、线程所有权和失败路径。
- 接口注明阻塞、复制与缓冲区寿命；明确入队、硬件完成和设备确认的区别。
- wire 布局变更须由协议层与固件共同维护，不能混入格式整改。
  控制算法、参数、协议、目录重组与行为修改应分别验证。

## 模型、所有权与阅读顺序

- 头文件先展示公共模型和操作，字节处理、锁和资源细节放在实现边界内。
- 同类请求与报告相邻，收发方向以 MCU 为参照；USB IN/OUT 仍按 USB 标准相对主机。
- 函数按校验、处理、提交结果组织，错误提前返回；按职责连续排布，避免阅读时来回跳转。
- 状态归其真正所有者，跨线程状态注明发布方式；不使用包揽无关字段的 Context。
- 结构体表达逻辑消息，显式编解码定义 wire；业务读取具名字段，不通过 sizeof 或 packed struct 发送。
- 数组容量和实际传输长度分开校验；解码失败不发布部分结果。
- 抽函数须集中完整操作、约束或重复逻辑，不为重命名和单次转调新增文件。
- 替换实现后检查调用方、测试、构建和文档；没有实际使用者的旧兼容字段不继续保留。
- RAII 表达资源和锁的作用域；保持锁内只复制有界状态，不能执行 ROS、网络或动态分配。

## 格式化与检查

使用 clang-format 18（本轮验证版本 18.1.8）或更新版本：

```bash
python3 tools/format_cpp.py
python3 tools/format_cpp.py --check
# 指定已安装的格式工具：
CLANG_FORMAT=clang-format-18 python3 tools/format_cpp.py --check
git diff --check
```

系统只有 clang-format 14 等旧版本时，可安装独立工具环境：

```bash
python3 -m venv .cache/format-venv
.cache/format-venv/bin/pip install clang-format==18.1.8
CLANG_FORMAT="$PWD/.cache/format-venv/bin/clang-format" python3 tools/format_cpp.py --check
```

脚本仅遍历 `src/`、`tests/` 中的 C++ 源码，
不会格式化 `build/`、外部依赖或生成的构建产物。
`.clang-format-ignore` 统一列出以下例外：

- `src/hardware/usb/protocol/usb_protocol.hpp`、`usb_protocol_codec.hpp`、`usb_session_policy.hpp` 是 MCU 共享源码，保持逐字一致；主机专用 `usb_report_decoder.hpp` 正常参与格式检查。
- 测试 `bridge_layout.hpp` 标记为生成文件；公共模型头文件由本仓库直接维护并参与格式检查。

格式工具保证一致性，人工仍需检查命名、定义位置、分组和阅读节奏。
格式检查不能替代编译及现有合约测试；离线回归不能代替硬件验收。
整改不包含刷板、启动机器人、使能电机、提交或推送。

## 本仓库的验证边界

全仓检查覆盖 `src/` 和 `tests/` 的自有 C++，包括独立 benchmark；三个 MCU 共享
文件、生成消息/布局及 LQR 系数保持原样。独立基准使用历史协议，不改为 USB v4。
控制公式、默认参数、遥控优先级、使能门控和 wire 布局不因风格整改而改变。
结构调整需运行开发回归、独立 benchmark 合约以及平衡控制同输入数值对照。
工具目录的生成器须与生成内容一并审查，不为排版直接修改生成结果。
