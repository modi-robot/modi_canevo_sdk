# ModiToolIoCanevo 使用文档

## 目录

- [1. 概述](#1-概述)
- [2. 接口能力](#2-接口能力)
  - [2.1 通道编号](#21-通道编号)
  - [2.2 对象字典](#22-对象字典)
- [3. modi_tool_io_canevo 接口列表](#3-modi_tool_io_canevo-接口列表)
  - [3.1 构造与生命周期](#31-构造与生命周期)
  - [3.2 设备信息](#32-设备信息)
  - [3.3 工具 IO 读取与写入](#33-工具-io-读取与写入)
- [4. 示例程序](#4-示例程序)
  - [4.1 编译](#41-编译)
  - [4.2 扫描工具 IO 节点](#42-扫描工具-io-节点)
  - [4.3 读取工具 IO](#43-读取工具-io)
  - [4.4 设置 DO 输出](#44-设置-do-输出)
- [5. 典型用法](#5-典型用法)
- [6. 注意事项](#6-注意事项)

---

## 1. 概述

`modi_tool_io_canevo` 是 CanEvo SDK 中用于访问工具 IO 板的 C++ 类。
它继承自 `modi_node_canevo`，复用总线绑定、SDO 请求/响应等待、
设备信息读取和 SDO Abort 码查询能力。

工具 IO 节点只提供非实时 SDO 接口，不提供关节位置、速度、控制字、
PDO 实时控制等接口。它和 `modi_joint_canevo` 的区别是：

- `modi_joint_canevo` 表示关节节点，初始化时会写入关节同步周期 `0x00/0x0F`；
- `modi_tool_io_canevo` 表示工具 IO 节点，初始化时不会写入关节同步周期；
- 工具 IO 常用节点 ID 为 `46`，具体以现场配置为准。

---

## 2. 接口能力

工具 IO 当前支持以下能力：

- 读取工具板供电电压；
- 读取两路 AI 模拟输入；
- 读取两路 DI 数字输入；
- 读取两路 DO 数字输出状态；
- 设置单路 DO 数字输出。

所有工具 IO 接口都是 `Nrt` 非实时接口，会通过 SDO 阻塞等待从站应答或超时。
不要在高频实时控制循环中调用这些接口。

### 2.1 通道编号

公开接口统一使用 0-based `index`：

| `index` | 程序显示通道 | 说明 |
|--------:|------------|------|
| 0 | AI0 / DI0 / DO0 | 第 1 路 |
| 1 | AI1 / DI1 / DO1 | 第 2 路 |

`index >= 2` 会返回 `CanEvoError::kInvalidParam`。

### 2.2 对象字典

| 功能 | 对象字典 | 数据类型 | 单位/含义 |
|------|----------|----------|-----------|
| 工具板供电电压 | `0xE0/0x00` | `uint16_t` | 原始值乘 `0.01` 得到 V |
| AI0 模拟输入 | `0xE0/0x03` | `uint16_t` | 原始值乘 `0.01` 得到 V |
| AI1 模拟输入 | `0xE0/0x04` | `uint16_t` | 原始值乘 `0.01` 得到 V |
| DI 数字输入 | `0xE0/0x05` | `uint16_t` | bit0=DI0，bit1=DI1 |
| DO 数字输出 | `0xE0/0x06` | `uint16_t` | bit0=DO0，bit1=DO1，可读写 |

---

## 3. modi_tool_io_canevo 接口列表

### 3.1 构造与生命周期

| 序号 | 接口名字 | 参数 | 返回值 | 说明 |
|------|----------|------|--------|------|
| 1 | `modi_tool_io_canevo()` | 无 | — | 构造工具 IO 节点对象 |
| 2 | `~modi_tool_io_canevo()` | 无 | — | 析构并释放节点资源 |
| 3 | `NrtInit(bus, node_id)` | `bus`: 已打开的总线引用<br>`node_id`: 工具 IO 节点 ID | `int` | 初始化工具 IO 节点，绑定到总线；不会写入关节同步周期 |
| 4 | `NrtDestroy()` | 无 | `void` | 释放节点资源，从总线注销 |

> 禁止拷贝构造和拷贝赋值。

### 3.2 设备信息

以下接口来自公共基类 `modi_node_canevo`：

| 序号 | 接口名字 | 参数 | 返回值 | 说明 |
|------|----------|------|--------|------|
| 1 | `NrtGetProtocolVersion()` | 无 | `uint16_t` | 读取协议版本，来自 `0x00/0x01` |
| 2 | `NrtGetModuleType()` | 无 | `uint32_t` | 读取模块类型，来自 `0x00/0x02` |
| 3 | `NrtGetVendorCode()` | 无 | `uint32_t` | 读取厂商代号，来自 `0x00/0x03` |
| 4 | `NrtGetFwVersion()` | 无 | `uint16_t` | 读取固件版本，来自 `0x00/0x0D` |
| 5 | `NrtGetHwVersion()` | 无 | `uint16_t` | 读取硬件版本，来自 `0x00/0x0E` |
| 6 | `NrtGetLastSdoAbortCode()` | 无 | `uint16_t` | 获取最近一次 SDO Abort 码 |

### 3.3 工具 IO 读取与写入

| 序号 | 接口名字 | 参数 | 返回值 | 说明 |
|------|----------|------|--------|------|
| 1 | `RtGetToolSupplyVoltage(voltage_v)` | `voltage_v`: 输出电压，单位 V | `int` | 读取工具板供电电压 |
| 2 | `RtGetToolAnalogInput(index, voltage_v)` | `index`: AI 通道 0/1<br>`voltage_v`: 输出电压，单位 V | `int` | 读取单路 AI 电压 |
| 3 | `RtGetToolDigitalInput(index, value)` | `index`: DI 通道 0/1<br>`value`: 输出状态 | `int` | 读取单路 DI 状态 |
| 4 | `RtGetToolDigitalOutput(index, value)` | `index`: DO 通道 0/1<br>`value`: 输出状态 | `int` | 读取单路 DO 状态 |
| 5 | `RtSetToolDigitalOutput(index, value)` | `index`: DO 通道 0/1<br>`value`: 写入状态 | `int` | 设置单路 DO 状态 |

`RtSetToolDigitalOutput()` 内部会先读取当前 DO 位图，只修改指定 `index`
对应的一路，再写回 `0xE0/0x06`，因此不会主动覆盖另一条 DO 通道。

---

## 4. 示例程序

示例程序路径：

```text
example/example_tool_io/example_tool_io.cpp
```

编译后可执行文件：

```text
build/example/example_tool_io
```

### 4.1 编译

在 SDK 根目录执行：

```bash
cd /home/niic/hao/modi_canevo_sdk
cmake --build build --target example_tool_io -j"$(nproc)"
```

或全量编译：

```bash
cd /home/niic/hao/modi_canevo_sdk
cmake --build build -j"$(nproc)"
```

### 4.2 扫描工具 IO 节点

```bash
sudo ./build/example/example_tool_io --can can0 --scan
```

该命令会扫描当前总线上的在线节点，并尝试读取 `0xE0/0x00`
判断节点是否支持工具 IO。注意当前 `NrtScanJoints()` 的实现会返回所有在线节点，
包括关节节点和工具 IO 节点；示例程序会逐个尝试判断。

### 4.3 读取工具 IO

读取一次：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46
```

持续读取：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46 --count 0 --interval-ms 500
```

读取 20 次：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46 --count 20 --interval-ms 500
```

输出示例：

```text
supply=24.000 V  AI0=0.000 V  AI1=0.000 V  DI0=0  DI1=1  DO0=0  DO1=0
```

含义：

- `supply`：工具板供电电压；
- `AI0/AI1`：两路模拟输入电压；
- `DI0/DI1`：两路数字输入状态；
- `DO0/DO1`：两路数字输出状态。

### 4.4 设置 DO 输出

打开 DO0：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46 --do-index 0 --do-value 1
```

关闭 DO0：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46 --do-index 0 --do-value 0
```

打开 DO1：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46 --do-index 1 --do-value 1
```

关闭 DO1：

```bash
sudo ./build/example/example_tool_io --can can0 --node 46 --do-index 1 --do-value 0
```

`--do-index` 和 `--do-value` 必须同时提供。`--scan` 不能和 DO 设置参数同时使用。

---

## 5. 典型用法

```cpp
#include <iostream>

#include "modi_tool_io_canevo.h"

int main() {
  modi_bus_canevo bus;
  if (bus.Open("can0") != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 总线失败" << std::endl;
    return -1;
  }

  modi_tool_io_canevo tool_io;
  if (tool_io.NrtInit(bus, 46) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化工具 IO 节点失败" << std::endl;
    bus.Close();
    return -1;
  }

  float supply = 0.0f;
  float ai1 = 0.0f;
  bool di1 = false;
  bool do1 = false;

  tool_io.RtGetToolSupplyVoltage(supply);
  tool_io.RtGetToolAnalogInput(0, ai1);
  tool_io.RtGetToolDigitalInput(0, di1);
  tool_io.RtGetToolDigitalOutput(0, do1);

  std::cout << "supply=" << supply << " V"
            << " ai1=" << ai1
            << " di1=" << di1
            << " do1=" << do1 << std::endl;

  // 打开 DO0。实际接负载前必须确认电气安全。
  const int ret = tool_io.RtSetToolDigitalOutput(0, true);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "设置 DO0 失败, ret=" << ret
              << " abort=0x" << std::hex
              << tool_io.NrtGetLastSdoAbortCode() << std::dec << std::endl;
  }

  tool_io.NrtDestroy();
  bus.Close();
  return 0;
}
```

---

## 6. 注意事项

1. **工具 IO 是 SDO 非实时访问**：读取 AI/DI/DO 和设置 DO 都会阻塞等待应答，不要放到高频实时控制循环中。

2. **工具 IO 不等于关节**：不要用 `modi_joint_canevo::NrtInit()` 初始化工具 IO 节点，否则关节初始化流程会尝试写入同步周期，工具 IO 可能超时或拒绝。

3. **扫描结果需要判断类型**：当前 `NrtScanJoints()` 会返回所有在线节点，不保证只返回关节。工具 IO 示例通过读取 `0xE0/0x00` 判断节点是否支持工具 IO。

4. **DO 会真实输出**：`RtSetToolDigitalOutput()` 会改变 8P 口输出状态，可能驱动外部负载。测试前应确认电源、电流、负载和接线安全。

5. **AI 量程**：AI 通常为 0~10V 模拟输入，不要把 24V 或 48V 直接接入 AI。

6. **DI 输入**：DI 为数字输入，现场测试时按工具板规格接入有效电平，不要超过输入允许范围。

7. **错误判断**：返回 `CanEvoError::kSdoTimeout` 通常表示没有等到节点应答；返回 `CanEvoError::kSdoAbort` 时可通过 `NrtGetLastSdoAbortCode()` 查看从站拒绝原因。
