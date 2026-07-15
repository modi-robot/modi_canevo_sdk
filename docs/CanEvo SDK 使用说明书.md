# CanEvo SDK 使用说明书

## 目录

- [1. 概述](#1-概述)
- [2. 运行环境准备](#2-运行环境准备)
- [3. 软件包获取与解压](#3-软件包获取与解压)
- [4. 编译与安装](#4-编译与安装)
- [5. CAN-FD 总线配置](#5-can-fd-总线配置)
- [6. 示例程序说明](#6-示例程序说明)
- [7. 示例运行流程](#7-示例运行流程)
- [8. 抓包与日志分析](#8-抓包与日志分析)
- [9. 开发者集成方式](#9-开发者集成方式)
- [10. 注意事项](#10-注意事项)
- [11. 常见问题与处理](#11-常见问题与处理)

---

## 1. 概述

CanEvo SDK 是基于 CanEvo CAN-FD 协议的 Linux/SocketCAN C++ SDK，用于控制 CanEvo 关节。

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
sudo apt install -y build-essential cmake unzip can-utils
```

如果需要使用日志轮转脚本：

```bash
sudo apt install -y apache2-utils
```

### 2.2 确认 CAN 设备

查看 CAN 接口：

```bash
ip -br link
ip -details link show can0
```

正常情况下应能看到 `can0`，配置完成后应为 `UP`，CAN 状态应为 `ERROR-ACTIVE`。

---

## 3. 软件包获取与解压

### 3.1 从打包文件使用

打包后的文件名通常类似：

```text
canevo_sdk-0.1.0-<git短哈希>-Linux-x86_64.zip
```

将压缩包复制到目标机器后解压：

```bash
unzip canevo_sdk-0.1.0-*-Linux-*.zip
cd canevo_sdk-0.1.0-*-Linux-*
```

解压后的典型目录：

```text
canevo_sdk-.../
├── include/                 # SDK 头文件
├── lib/                     # SDK 动态库
│   ├── libcanevo_sdk_x86.so
│   ├── runtime/             # 随包运行时库
│   └── cmake/
├── docs/                    # 文档
└── example/                 # 示例源码和示例 CMakeLists.txt
```

### 3.2 从源码打包

在源码目录执行：

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
cmake --build build --target package
```

生成的 zip 包位于 `build/` 目录。

---

## 4. 编译与安装

### 4.1 从源码编译整个 SDK

```bash
cd canevo_sdk_x86
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

编译指定示例：

```bash
cmake --build build --target example_single_joint_sdo -j4
cmake --build build --target example_single_joint_csv -j4
cmake --build build --target example_single_joint_rt_pp -j4
cmake --build build --target example_multi_joints_pp_csv -j4
```

安装到默认目录 `build/install`：

```bash
cmake --build build --target install
```

### 4.2 从已解压软件包编译示例

软件包中已经包含 SDK 库和示例源码。进入包内 `example` 目录单独编译：

```bash
cd canevo_sdk-0.1.0-*-Linux-*/example
cmake -S . -B build
cmake --build build -j$(nproc)
```

示例可执行文件位于：

```text
example/build/
```

---

## 5. CAN-FD 总线配置

### 5.1 通用 SocketCAN 配置

常用配置为仲裁段 1 Mbps、数据段 5 Mbps、CAN-FD 开启：

```bash
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 1000 berr-reporting on
sudo ip link set can0 txqueuelen 1
sudo ip link set can0 up

ip -details link show can0
```

确认输出中包含：

```text
can <BERR-REPORTING,FD> state ERROR-ACTIVE
bitrate 1000000
dbitrate 5000000
```

### 5.2 NIIC / mttcan 板卡参考配置

部分 mttcan 板卡需要先配置引脚复用和 TDC：

```bash
sudo busybox devmem 0x0243d008 32 0x400
sudo busybox devmem 0x0243d018 32 0x458
sudo busybox devmem 0x0243d028 32 0x400
sudo busybox devmem 0x0243d040 32 0x400

sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 1000 berr-reporting on
sudo ip link set can0 txqueuelen 1

sudo chmod 666 /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset
sudo bash -c "echo 0x600 > /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset"

sudo ip link set can0 up
ip -details link show can0
cat /sys/devices/platform/bus@0/c310000.mttcan/net/can0/tdc_offset
```

如果 `tdc_offset` 路径不存在，说明当前 CAN 控制器不是该 mttcan 设备，按实际板卡路径配置。

---

## 6. 示例程序说明

### 6.1 单关节示例

| 目标名 | 源文件 | 模式 | 控制方式 | 说明 |
| --- | --- | --- | --- | --- |
| `example_single_joint_sdo` | `example_single_joint/example_sdo.cpp` | 无运动 | SDO | 扫描关节、读取设备信息和状态 |
| `example_single_joint_csv` | `example_single_joint/example_csv.cpp` | CSV | PDO 目标速度 | 单关节在位置范围内按速度往复 |
| `example_single_joint_pv` | `example_single_joint/example_pv.cpp` | PV | PDO4 目标速度 | 单关节通过轮廓速度模式往复 |
| `example_single_joint_csp` | `example_single_joint/example_csp.cpp` | CSP | PDO 目标位置 | 单关节周期同步位置控制 |
| `example_single_joint_cst` | `example_single_joint/example_cst.cpp` | CST | PDO 目标电流 | 单关节电流模式测试，默认 0 A |
| `example_single_joint_nrt_pp` | `example_single_joint/example_nrt_pp.cpp` | PP | SDO 目标位置 | 通过 SDO 下发 PP 目标，适合验证非实时 PP |
| `example_single_joint_rt_pp` | `example_single_joint/example_rt_pp.cpp` | PP | PDO3 目标位置 | 通过 RxPDO3 下发 PP 目标，适合验证 PP PDO |
| `example_single_joint_rt_thread` | `example_single_joint/example_rt_thread.cpp` | 无 CAN | 无 | 实时线程时间戳采样，不访问 CAN |

部分单关节示例支持命令行指定关节 ID：

```bash
sudo ./example_single_joint_csv 2
sudo ./example_single_joint_pv 2
sudo ./example_single_joint_rt_pp 1
sudo ./example_single_joint_nrt_pp 1
```

不指定 ID 时，程序通常选择扫描到的第一个关节。

### 6.2 多关节示例

| 目标名 | 源文件 | 模式 | 控制方式 | 说明 |
| --- | --- | --- | --- | --- |
| `example_multi_joints_sdo` | `example_multi_joints/example_sdo.cpp` | 无运动 | SDO | 扫描并初始化总线上全部关节 |
| `example_multi_joints_csv` | `example_multi_joints/example_csv.cpp` | CSV | PDO 目标速度 | 多关节 CSV 往复运动 |
| `example_multi_joints_csp` | `example_multi_joints/example_csp.cpp` | CSP | PDO 目标位置 | 多关节 CSP 周期同步位置控制 |
| `example_multi_joints_cst` | `example_multi_joints/example_cst.cpp` | CST | PDO 目标电流 | 多关节 CST 电流控制 |
| `example_multi_joints_pp` | `example_multi_joints/example_pp.cpp` | PP | SDO 目标位置 | 多关节 PP SDO 目标位置示例 |
| `example_multi_joints_pp_csv` | `example_multi_joints/example_pp_csv.cpp` | PP + CSV | PP 用 PDO3，CSV 用 PDO1 | 1/3/5/7 跑 PP，2/4/6/8 跑 CSV |

### 6.3 NRT 与 RT 示例区别

| 项目 | NRT/SDO 示例 | RT/PDO 示例 |
| --- | --- | --- |
| 典型目标 | `example_single_joint_sdo`、`example_single_joint_nrt_pp` | `example_single_joint_csv`、`example_single_joint_csp`、`example_single_joint_rt_pp` |
| 通信方式 | SDO 阻塞读写 | PDO 非阻塞收发 |
| 适合场景 | 参数读取、配置、诊断、低频目标 | 高频控制、周期运动、实时反馈 |
| 是否放进实时循环 | 不建议 | 建议 |

> 现有多关节 RT 示例的初始化、清故障、模式切换仍使用 `Nrt`/SDO；运动目标在实时循环中使用 `Rt`/PDO。

---

## 7. 示例运行流程

### 7.1 推荐安全流程

1. 确认机械结构无卡滞，关节固定可靠。
2. 确认 CAN_H/CAN_L/GND 接线正确，终端电阻正确。
3. 配置 CAN-FD 总线。
4. 先运行 SDO 示例确认能扫描到关节。
5. 读取故障码为 0 后，再运行运动示例。
6. 首次运动时降低速度、加速度和目标角度。
7. 观察电流、温度、故障码和 CAN 状态。

### 7.2 扫描与读取信息

```bash
cd canevo_sdk_x86/build/example
sudo ./example_single_joint_sdo
```

期望输出：

```text
打开 CAN 总线 can0 成功
发现 N 个关节, ID: ...
初始化关节 (ID=...) 成功
故障代码: 0
```

### 7.3 单关节 CSV 测试

```bash
sudo ./example_single_joint_csv 2
```

运行时程序会切换到 CSV 模式，并通过 RxPDO1 下发目标速度。

### 7.4 单关节 PP PDO 测试

```bash
sudo ./example_single_joint_rt_pp 1
```

该示例通过 RxPDO3 下发 PP 目标位置和轮廓参数。

### 7.5 多关节 PP + CSV 混合测试

```bash
sudo ./example_multi_joints_pp_csv
```

默认逻辑：

- 1/3/5/7：PP 模式，RxPDO3 下发目标位置。
- 2/4/6/8：CSV 模式，RxPDO1 下发目标速度。

---

## 8. 抓包与日志分析

### 8.1 直接抓包

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

### 8.2 使用日志轮转脚本

```bash
cd canevo_sdk_x86
./script/candump_rotate.sh can0
```

日志默认保存到：

```text
build/logs/canevo_can.YYYYMMDD_HHMMSS.log
```

### 8.3 常用 COB-ID 对照

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

### 8.4 CAN-FD 日志格式说明

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

## 9. 开发者集成方式

### 9.1 CMake 集成

安装 SDK 后，在业务工程中使用：

```cmake
find_package(canevo_sdk_x86 REQUIRED)
target_link_libraries(your_target PRIVATE canevo_sdk_x86::canevo_sdk_x86)
```

### 9.2 直接编译链接

```bash
g++ your_app.cpp \
    -I/path/to/canevo_sdk/include \
    -L/path/to/canevo_sdk/lib \
    -lcanevo_sdk_x86 \
    -lpthread \
    -o your_app
```

运行时如果找不到动态库：

```bash
export LD_LIBRARY_PATH=/path/to/canevo_sdk/lib:$LD_LIBRARY_PATH
```

---

## 10. 注意事项

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

## 11. 常见问题与处理

### 11.1 `Device "can0" does not exist`

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

### 11.2 打开 CAN 成功，但未扫描到在线关节

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

### 11.3 `ERROR-PASSIVE` 或 `tx 128 rx 0`

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

### 11.4 SDO 超时或 `errno=-3`

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

### 11.5 `0xFF02` 关节峰值电流过载

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

### 11.6 程序显示进入 Running，但关节不动

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

### 11.7 实时线程创建失败或实时性差

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

如果没有硬实时环境，程序会显示 POSIX fallback，仍可用于功能测试，但实时抖动可能增大。

### 11.8 运行 example 时找不到动态库

错误类似：

```text
error while loading shared libraries: libcanevo_sdk_x86.so
```

处理：

```bash
export LD_LIBRARY_PATH=/path/to/canevo_sdk/lib:$LD_LIBRARY_PATH
```

如果使用包内 `example/CMakeLists.txt` 编译示例，通常会自动链接包内 SDK。
