# CanEvo SDK

CanEvo SDK 是面向 MODI CanEvo 关节的 Linux C++ SDK，支持 x86_64 和 ARM 架构，基于 SocketCAN 提供关节发现、参数读写、状态查询和运动控制能力。

SDK 发布包包含预编译动态库、C++ 头文件、CMake 配置、示例程序源码和配套文档。用户无需编译 SDK 源码，只需下载发布包、配置 CAN-FD 接口并编译示例程序即可开始使用。

## 主要功能

- 支持通过 SDO 扫描关节、读取设备信息和配置关节参数
- 支持 PP、PV、CSV、CSP、CST 等运动模式
- 支持单关节与多关节控制
- 提供阻塞式 NRT 接口和面向实时循环的 RT 接口
- 提供 CAN-FD 抓包与日志轮转脚本
- 提供 CMake 包配置，便于集成到用户工程

## 支持环境

- 操作系统：Linux
- 处理器架构：x86_64、ARM64（aarch64）
- CAN 接口：SocketCAN
- 推荐 CAN-FD 配置：仲裁段 1 Mbps、数据段 5 Mbps
- 推荐适配器：KH-UCANFD
- 构建工具：CMake、支持 C++17 的编译器

对实时控制有严格要求时，建议使用 PREEMPT_RT 内核并完成 CPU 隔离和 IRQ 亲和性配置，具体方法参见[实时系统配置指南](docs/CanEvo实时系统配置指南.md)。

## 下载 SDK

请从 [GitHub Releases](https://github.com/modi-robot/modi_canevo_sdk/releases) 下载最新发布包。

发布包名称格式：

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>.zip
```

例如，x86_64 和 ARM64 发布包可能分别为：

```text
modi_sdk_0.0.1_linux_x86_64_8f73bfb.zip
modi_sdk_0.0.1_linux_aarch64_8f73bfb.zip
```

请根据目标机器的处理器架构选择对应发布包，可通过 `uname -m` 查看当前架构。

## 快速开始

### 1. 安装基础工具

```bash
sudo apt update
sudo apt install -y build-essential cmake wget unzip can-utils
```

KH-UCANFD 驱动的安装方法请参见[使用说明书：安装 KH-UCANFD 驱动](docs/CanEvo%20SDK%20使用说明书.md#22-安装-kh-ucanfd-驱动)。

### 2. 下载并解压发布包

```bash
uname -m
unzip modi_sdk_*_linux_*.zip
cd modi_sdk_*_linux_*
export SDK_ROOT="$PWD"
```

如果当前目录中有多个架构或版本的压缩包，请将通配符替换为实际下载的完整文件名和解压目录名。

### 3. 配置 CAN-FD 接口

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 txqueuelen 1
sudo ip link set can0 up
ip -details link show can0
```

如果实际接口不是 `can0`，请将命令中的接口名替换为 `can1` 等实际名称。

### 4. 编译示例程序

```bash
cd "$SDK_ROOT/example"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel "$(nproc)"
```

首次连接关节时，建议先运行 SDO 示例扫描设备并读取状态，再进行运动控制测试。运行前请确保关节固定可靠、急停可用且运动范围安全。

完整操作步骤参见 [CanEvo SDK 使用说明书](docs/CanEvo%20SDK%20使用说明书.md)。

## 文档导航

| 文档 | 内容 |
| --- | --- |
| [CanEvo SDK 使用说明书](docs/CanEvo%20SDK%20使用说明书.md) | 环境准备、驱动安装、示例编译运行、抓包和常见问题 |
| [ModiJointCanevo 接口文档](docs/ModiJointCanevo%20接口文档.md) | 数据类型、C++ API、PDO 映射、对象字典和典型用法 |
| [CanEvo 关节通信协议](docs/CanEvo关节通信协议_V1.2.4.md) | CAN-FD 通信协议和报文定义 |
| [CanEvo 实时系统配置指南](docs/CanEvo实时系统配置指南.md) | PREEMPT_RT、CPU 隔离和 IRQ 亲和性配置 |

## 发布包结构

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>/
├── include/        # SDK 头文件
├── lib/            # SDK 动态库、运行时库和 CMake 配置
├── example/        # 单关节与多关节示例源码
├── script/         # CAN 日志工具
└── docs/           # 使用说明、接口文档、通信协议和实时配置指南
```

## 使用提示

- 首次测试请使用较小的速度、加速度和运动范围。
- 运动前确认关节无故障、CAN-FD 波特率正确且设备 ID 不冲突。
- `Nrt` 接口为阻塞调用，不应放入实时控制循环。
- CSP 等同步控制模式对系统和 CAN 总线抖动敏感。
- 遇到设备无法发现、SDO 超时或总线错误时，先查看使用说明书中的“常见问题与处理”。

## 问题反馈

使用中发现问题时，请在仓库的 [Issues](https://github.com/modi-robot/modi_canevo_sdk/issues) 页面提交，并尽量附上以下信息：

- SDK 版本和发布包文件名
- Linux 发行版与内核版本
- CAN 适配器型号及接口状态
- 可复现的操作步骤
- 完整错误输出和相关 CAN 抓包日志
