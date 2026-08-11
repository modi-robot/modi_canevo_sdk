# CanEvo SDK 使用说明书

## 目录

- [1. 概述](#1-概述)
- [2. 运行环境准备](#2-运行环境准备)
- [3. 软件包获取与解压](#3-软件包获取与解压)
- [4. 编译示例程序](#4-编译示例程序)
- [5. 示例程序说明](#5-示例程序说明)
- [6. 示例运行流程](#6-示例运行流程)
- [7. 抓包与日志分析](#7-抓包与日志分析)
- [8. 集成到用户项目](#8-集成到用户项目)
- [9. 注意事项](#9-注意事项)
- [10. 常见问题与处理](#10-常见问题与处理)

---

## 1. 概述

CanEvo SDK 是基于 CanEvo CAN-FD 协议的 Linux/SocketCAN C++ SDK，用于控制 CanEvo 关节。

本文面向获取正式发布包的 SDK 用户，说明环境准备、示例编译运行、业务工程集成和常见故障处理。用户不需要获取或编译 SDK 库源码。

SDK 采用 Bus + Joint 两级结构：

- `modi_bus_canevo`：管理一条 CAN-FD 总线，例如 `can0`。
- `modi_joint_canevo`：管理一个关节节点，例如 ID=1 的关节。

接口按使用场景分为两类：

| 类型 | 前缀 | 通信方式 | 用途 |
| --- | --- | --- | --- |
| 实时接口 | `Rt` | PDO | 控制循环中发送目标值、读取 PDO 缓存状态 |
| 非实时接口 | `Nrt` | SDO | 初始化、扫描、模式切换、参数读取、故障处理 |

> 约定：控制循环内不要调用 `Nrt`/SDO 接口，避免阻塞实时线程。

---

## 2. 运行环境准备

### 2.1 基础依赖

Ubuntu/Debian 系统建议安装：

```bash
sudo apt update
sudo apt install -y build-essential cmake wget unzip can-utils
```
### 2.2 安装 KH-UCANFD 驱动

使用昆宏 KH-UCANFD 适配器时，需要先安装对应的 Linux 驱动：

```bash
wget https://gitee.com/ChengDu-KunHong/KH-UCANFD_Linux_SDK/releases/download/latest/KH-UCANFD_Linux_SDK.zip
unzip KH-UCANFD_Linux_SDK.zip
cd KH-UCANFD_Linux_SDK-*/
sudo ./build.sh
```

安装完成后，确认 `kcan` 内核模块已经加载：

```bash
lsmod | grep kcan
```

如果没有输出，请检查 `build.sh` 的编译日志、当前内核版本，以及 `/lib/modules/$(uname -r)/build` 是否存在。驱动安装和实时系统配置的完整说明参见[《CanEvo 实时系统安装、配置与验证指南》](CanEvo实时系统配置指南.md)。

### 2.3 确认 CAN 设备

先查看 CAN 接口是否存在：

```bash
ip -br link
```

配置 CAN-FD，仲裁段波特率为 1 Mbps、数据段波特率为 5 Mbps：

```bash
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can0 txqueuelen 1
sudo ip link set can0 up
```

查看配置结果：

```bash
ip -details link show can0
```

正常情况下应能看到 `can0`，配置完成后应为 `UP`，CAN 状态应为 `ERROR-ACTIVE`。

降低 `txqueuelen` 可以减少内核发送队列积压，使报文发送时间更接近控制线程的调度点。多关节报文较多时，可以对比 `txqueuelen 1`、`2`、`4` 的总线实际抖动，最终以示波器或逻辑分析仪的测量结果为准。更完整的配置参见[《CanEvo 实时系统安装、配置与验证指南》](CanEvo实时系统配置指南.md)。

---

## 3. 软件包获取与解压

SDK 软件包可从 GitHub Releases 下载：

[https://github.com/modi-robot/modi_canevo_sdk/releases](https://github.com/modi-robot/modi_canevo_sdk/releases)


在 Releases 页面选择所需版本，并下载与目标机器平台、架构匹配的 ZIP 文件。文件名格式为：

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>.zip
```

各字段之间使用下划线 `_` 分隔。例如版本 `0.0.1`、Linux x86_64 平台的产物为：

```text
modi_sdk_0.0.1_linux_x86_64_8f73bfb.zip
```

将 ZIP 文件复制到目标机器的工作目录。以下命令查找当前目录中的 Linux x86_64 SDK 包，解压后进入对应目录：

```bash
SDK_ARCHIVE="$(find . -maxdepth 1 -type f \
  -name 'modi_sdk_*_linux_x86_64_*.zip' -print -quit)"

if [[ -z "${SDK_ARCHIVE}" ]]; then
  echo "未找到 Linux x86_64 SDK 压缩包" >&2
  exit 1
fi

unzip "${SDK_ARCHIVE}"
SDK_ROOT="${SDK_ARCHIVE%.zip}"
cd "${SDK_ROOT}"
```

解压后的目录名与 ZIP 文件名一致（不含 `.zip` 后缀），典型结构如下：

```text
modi_sdk_<版本>_<平台>_<架构>_<Git短提交>/
├── include/                 # SDK 头文件
├── lib/                     # SDK 动态库
│   ├── libcanevo_sdk_x86.so
│   ├── runtime/             # 随包运行时库
│   └── cmake/
├── docs/                    # 文档
├── example/                 # 示例源码和示例 CMakeLists.txt
└── script/
    └── candump_rotate.sh    # CAN 抓包与日志轮转脚本
```

本文后续使用 `SDK_ROOT` 表示这个解压后的顶层目录。例如将 `0.0.1` 发布包解压到用户主目录后，可执行：

```bash
export SDK_ROOT="$HOME/modi_sdk_0.0.1_linux_x86_64_8f73bfb"
```

如果下载的版本或 Git 短提交不同，请按实际解压目录名修改。可用下面的命令确认路径正确：

```bash
ls "${SDK_ROOT}/include" "${SDK_ROOT}/lib" "${SDK_ROOT}/example"
```

---

## 4. 编译示例程序

如果已经按照第 3 章进入 SDK 解压目录，执行：

```bash
cd example
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j"$(nproc)"
```

编译完成后，示例可执行文件位于：

```text
example/build/
```
---

## 5. 示例程序说明

### 5.1 单关节示例

| 目标名 | 源文件 | 模式 | 控制方式 | 说明 |
| --- | --- | --- | --- | --- |
| `example_single_joint_sdo` | `example_single_joint/example_sdo.cpp` | 无运动 | SDO | 扫描关节、读取设备信息和状态 |
| `example_single_joint_csv` | `example_single_joint/example_csv.cpp` | CSV | PDO 目标速度 | 单关节在位置范围内按速度往复 |
| `example_single_joint_pv` | `example_single_joint/example_pv.cpp` | PV | PDO4 目标速度 | 单关节通过轮廓速度模式往复 |
| `example_single_joint_csp` | `example_single_joint/example_csp.cpp` | CSP | PDO 目标位置 | 单关节周期同步位置控制 |
| `example_single_joint_cst` | `example_single_joint/example_cst.cpp` | CST | PDO 目标电流 | 单关节电流模式测试，默认 0 A |
| `example_single_joint_nrt_pp` | `example_single_joint/example_nrt_pp.cpp` | PP | SDO 目标位置 | 通过 SDO 下发 PP 目标，适合验证非实时 PP |
| `example_single_joint_rt_pp` | `example_single_joint/example_rt_pp.cpp` | PP | PDO3 目标位置 | 通过 RxPDO3 下发 PP 目标，适合验证 PP PDO |

部分单关节示例支持命令行指定关节 ID：

```bash
sudo ./example_single_joint_csv 2
sudo ./example_single_joint_pv 2
sudo ./example_single_joint_rt_pp 1
sudo ./example_single_joint_nrt_pp 1
```

不指定 ID 时，程序通常选择扫描到的第一个关节。

### 5.2 多关节示例

| 目标名 | 源文件 | 模式 | 控制方式 | 说明 |
| --- | --- | --- | --- | --- |
| `example_multi_joints_sdo` | `example_multi_joints/example_sdo.cpp` | 无运动 | SDO | 扫描并初始化总线上全部关节 |
| `example_multi_joints_csv` | `example_multi_joints/example_csv.cpp` | CSV | PDO 目标速度 | 多关节 CSV 往复运动 |
| `example_multi_joints_csp` | `example_multi_joints/example_csp.cpp` | CSP | PDO 目标位置 | 多关节 CSP 周期同步位置控制 |
| `example_multi_joints_cst` | `example_multi_joints/example_cst.cpp` | CST | PDO 目标电流 | 多关节 CST 电流控制 |
| `example_multi_joints_pp` | `example_multi_joints/example_pp.cpp` | PP | SDO 目标位置 | 多关节 PP SDO 目标位置示例 |
| `example_multi_joints_pp_csv` | `example_multi_joints/example_pp_csv.cpp` | PP + CSV | PP 用 PDO3，CSV 用 PDO1 | 1/3/5/7 跑 PP，2/4/6/8 跑 CSV |

### 5.3 NRT 与 RT 示例区别

| 项目 | NRT/SDO 示例 | RT/PDO 示例 |
| --- | --- | --- |
| 典型目标 | `example_single_joint_sdo`、`example_single_joint_nrt_pp` | `example_single_joint_csv`、`example_single_joint_csp`、`example_single_joint_rt_pp` |
| 通信方式 | SDO 阻塞读写 | PDO 非阻塞收发 |
| 适合场景 | 参数读取、配置、诊断、低频目标 | 高频控制、周期运动、实时反馈 |
| 是否放进实时循环 | 不建议 | 建议 |

> 现有多关节 RT 示例的初始化、清故障、模式切换仍使用 `Nrt`/SDO；运动目标在实时循环中使用 `Rt`/PDO。

---

## 6. 示例运行流程

### 6.1 推荐安全流程

1. 确认机械结构无卡滞，关节固定可靠。
2. 确认 CAN_H/CAN_L/GND 接线正确，终端电阻正确。
3. 配置 CAN-FD 总线。
4. 先运行 SDO 示例确认能扫描到关节。
5. 读取故障码为 0 后，再运行运动示例。
6. 首次运动时降低速度、加速度和目标角度。
7. 观察电流、温度、故障码和 CAN 状态。

### 6.2 扫描与读取信息

以下命令进入第 4 章生成示例可执行文件的目录。这里假设已经按第 3 章设置了 `SDK_ROOT`：

```bash
cd "${SDK_ROOT}/example/build"
sudo ./example_single_joint_sdo
```

期望输出：

```text
打开 CAN 总线 can0 成功
发现 N 个关节, ID: ...
初始化关节 (ID=...) 成功
故障代码: 0
```

### 6.3 单关节 CSV 测试

```bash
sudo ./example_single_joint_csv 2
```

运行时程序会切换到 CSV 模式，并通过 RxPDO1 下发目标速度。

### 6.4 单关节 PP PDO 测试

```bash
sudo ./example_single_joint_rt_pp 1
```

该示例通过 RxPDO3 下发 PP 目标位置和轮廓参数。

### 6.5 多关节 PP + CSV 混合测试

```bash
sudo ./example_multi_joints_pp_csv
```

默认逻辑：

- 1/3/5/7：PP 模式，RxPDO3 下发目标位置。
- 2/4/6/8：CSV 模式，RxPDO1 下发目标速度。

---

## 7. 抓包与日志分析

### 7.1 直接抓包

```bash
candump -tz -x can0
```

按节点过滤常用 ID：

```bash
# 2号 CSV
candump -tz -x can0,242:7FF,402:7FF,782:7FF,7C2:7FF,03F:7FF

# 1号 PP PDO
candump -tz -x can0,2C1:7FF,401:7FF,781:7FF,7C1:7FF,03F:7FF
```

### 7.2 保存抓包日志

发布包提供了 CAN 日志脚本。它默认从 `can0` 抓包，单个日志达到 20 MB 时自动切换，并只保留最近 20 个文件：

```bash
"${SDK_ROOT}/script/candump_rotate.sh" can0
```

抓取其他 CAN 接口时，将接口名作为第一个参数。例如抓取 `can1`：

```bash
"${SDK_ROOT}/script/candump_rotate.sh" can1
```

不传参数时默认使用 `can0`：

```bash
"${SDK_ROOT}/script/candump_rotate.sh"
```

运行前可以查看系统中实际存在的 CAN 接口：

```bash
ip -br link | grep -E '(^|\s)(can|vcan)[0-9]+'
```

如果指定的接口不存在，脚本会报错退出；如果接口存在但没有处于 `UP` 状态，脚本会给出警告，应先完成对应接口的 CAN-FD 配置。

按 `Ctrl+C` 停止抓包。日志默认保存在：

```text
<SDK_ROOT>/build/logs/
```

该脚本依赖 `candump` 和 `rotatelogs`；如果尚未安装：

```bash
sudo apt install -y can-utils apache2-utils
```

### 7.3 常用 COB-ID 对照

| 类型 | COB-ID | 例：ID=1 | 例：ID=2 |
| --- | --- | ---: | ---: |
| SYNC | `0x03F` | `0x03F` | `0x03F` |
| EMCY | `0x080 + node_id` | `0x081` | `0x082` |
| RxPDO0 / CSP 目标位置 | `0x200 + node_id` | `0x201` | `0x202` |
| RxPDO1 / CSV 目标速度 | `0x240 + node_id` | `0x241` | `0x242` |
| RxPDO2 / CST 目标电流 | `0x280 + node_id` | `0x281` | `0x282` |
| RxPDO3 / PP 目标位置 | `0x2C0 + node_id` | `0x2C1` | `0x2C2` |
| RxPDO4 / PV 目标速度 | `0x300 + node_id` | `0x301` | `0x302` |
| TxPDO0 / 状态反馈 | `0x400 + node_id` | `0x401` | `0x402` |
| SDO 请求 | `0x780 + node_id` | `0x781` | `0x782` |
| SDO 响应 | `0x7C0 + node_id` | `0x7C1` | `0x7C2` |

### 7.4 CAN-FD 日志格式说明

紧凑日志中 `##` 后第一个十六进制字符是 CAN-FD flags，不是 payload。

例如：

```text
242##102200000003F
```

含义：

```text
1                  # CAN-FD flags，BRS 开启
02200000003F       # payload
02 20              # controlword = 0x2002
00 00 00 3F        # float32 0.5 rpm
```

---

## 8. 集成到用户项目

### 8.1 CMake 集成

在业务工程配置阶段，将 SDK 解压目录传给 `CMAKE_PREFIX_PATH`：

```bash
cmake -S . -B build \
  -DCMAKE_PREFIX_PATH="${SDK_ROOT}"
cmake --build build -j$(nproc)
```

业务工程的 `CMakeLists.txt` 使用：

```cmake
find_package(canevo_sdk_x86 REQUIRED)
target_link_libraries(your_target PRIVATE canevo_sdk_x86::canevo_sdk_x86)
```

---

## 9. 注意事项

1. 运动前必须确认故障码为 0。
2. 首次测试不要直接使用大角度、大速度、大加速度。
3. CSV/CSP/CST/PP PDO 示例中，初始化和模式切换使用 SDO，实时目标使用 PDO。
4. CSP 对同步周期抖动敏感，应优先使用实时内核、板载 CAN 或 PCIe CAN。
5. USB-CAN 适合 SDO 和低实时要求测试，不建议用于严格 CSP 控制。
6. `Nrt` 接口为阻塞调用，不应放入实时线程循环。
7. `Rt` 接口依赖 `RtStepOnce()` 更新接收缓存，实时循环中应周期调用。
8. 退出运动程序时优先按 Ctrl+C，让程序执行失能和资源清理。
9. 抓包时同时看控制帧和反馈帧，不能只看目标帧。
10. 出现 `ERROR-PASSIVE`、`BUS-OFF` 时先处理 CAN 物理层和波特率问题，不要继续运动。

---

## 10. 常见问题与处理

### 10.1 `Device "can0" does not exist`

原因：

- CAN 驱动未加载。
- 设备名不是 `can0`。
- USB-CAN 或板卡未识别。

处理：

```bash
ip -br link
dmesg | grep -i can
```

确认实际接口名后修改程序或命令中的 CAN 名称。

### 10.2 打开 CAN 成功，但未扫描到在线关节

可能原因：

- 关节未上电。
- CAN_H/CAN_L 接反。
- 缺少共地。
- 终端电阻不正确。
- 仲裁段/数据段波特率不一致。
- CAN-FD/BRS 配置不一致。

处理步骤：

```bash
ip -details link show can0
candump -tz -x can0
cansend can0 781##101000C0000000000
```

如果 1 号在线，应看到类似：

```text
can0  TX B -  781  [08]  01 00 0C 00 00 00 00 00
can0  RX - -  7C1  [05]  01 00 0C 01 00
```

### 10.3 `ERROR-PASSIVE` 或 `tx 128 rx 0`

含义：

- 主站发出了 CAN 帧，但总线上没有正确 ACK。

常见原因：

- 线接错或没有接从站。
- 没有终端电阻。
- 波特率或 CAN-FD 参数不匹配。
- 只接了 CAN_H/CAN_L，没有共地。

处理：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 1000 berr-reporting on
sudo ip link set can0 txqueuelen 1
sudo ip link set can0 up
ip -details link show can0
```

### 10.4 SDO 超时或 `errno=-3`

可能原因：

- 发送失败。
- 从站没有响应。
- CAN 发送队列堵塞。
- CAN 状态已经 ERROR-PASSIVE/BUS-OFF。

处理：

1. 先用 `ip -details link show can0` 看 CAN 状态。
2. 用 `candump -tz -x can0` 确认是否有 TX 和 RX。
3. 降低总线负载，确认没有多个程序同时控制同一关节。
4. 确认 `txqueuelen` 配置合理。

### 10.5 `0xFF02` 关节峰值电流过载

含义：

- 关节峰值电流过载。

协议文档建议：

- 排查硬件短路。
- 关节断电复位重启。
- 若为实际过载，需要等待约 1 分钟冷却，在此期间可能无法直接使能。

现场处理：

1. 立即停止运动程序。
2. 断电检查机械是否卡住。
3. 检查负载是否过大、抱闸是否释放。
4. 等待冷却后重新上电。
5. 用 SDO 示例确认故障码清零后再低速测试。

### 10.6 程序显示进入 Running，但关节不动

排查顺序：

1. 抓包确认是否有对应 RxPDO 目标帧。
2. 抓包确认 TxPDO0 反馈中状态是否为 Running。
3. 看目标值是否太小，例如 CSV 速度过低。
4. 看当前位置是否已经触发限位。
5. 查故障码、警告码、抱闸状态。

CSV 2 号关节应看到：

```text
242##...
402##...
```

PP 2 号关节应看到：

```text
2C2##...
402##...
```

### 10.7 实时线程创建失败或实时性差

可能原因：

- 没有 root 权限。
- 系统不允许 `SCHED_FIFO`。
- CPU 亲和性配置的 CPU 不存在。
- 当前系统不是实时内核。

处理：

```bash
sudo ./example_single_joint_csv 1
ulimit -r
uname -a
```

如果当前系统不是 PREEMPT_RT 内核，示例仍可用于基本通信功能测试，但不应据此评估实时控制性能。
