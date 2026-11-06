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
它通过内部 NodeCore 复用总线绑定、SDO 请求/响应等待、设备信息读取和 SDO Abort 码查询能力。

工具 IO 使用 SDO 查询设备配置，使用 PDO 读取工具状态和控制 DO，
使用独立 CAN-FD 实时透传访问夹爪；不提供关节位置、速度、控制字接口。
它和 `modi_joint_canevo` 的区别是：

- `modi_joint_canevo` 表示关节节点，初始化时会写入关节同步周期 `0x00/0x0F`；
- `modi_tool_io_canevo` 表示工具 IO 节点，初始化时不会写入关节同步周期；
- CoArm 左臂工具 IO 节点为 `can0/40`，右臂为 `can1/41`，具体以现场配置为准。

---

## 2. 接口能力

工具 IO 当前支持以下能力：

- 读取工具板供电电压；
- 读取两路 AI 模拟输入；
- 读取两路 DI 数字输入；
- 读取两路 DO 数字输出状态；
- 设置单路 DO 数字输出。
- 内部通过 `0x600 + node_id` 双向实时透传，一帧最多 60 字节串口数据，另加四字节封装头。
- 使用 `RtSendToolRs485()` 发送、`RtReceiveToolRs485()` 接收原始 RS485 数据，不限制夹爪型号和内层协议。

工具 IO 的版本、模块类型等配置查询是 `Nrt` SDO 接口。工具板状态和通用透传使用 `Rt` 接口。
SDK 仅处理工具板外层 `80 00 00 Len`，不解析设备地址、指令、校验和、电流、电压或运行状态。
夹爪协议、查询调度、反馈缓存和运动队列在 Robot HAL 中实现。

普通 PDO 的更新规则与关节、再生制动板一致：初始化时明确节点类型，接收线程按
该类型解析一次 PDO 并发布工程量快照；Get 只复制快照中的值，不重复解码或换算单位。
不再使用关节解析开关或依靠报文长度区分节点；长度校验用于拒绝已知协议的不完整帧。
供电电压、AI、DI、POD 读取使用初始化时的总线 PDO 有效期（默认 50 ms）；
未收到、过期或快照正在更新时返回 `kPdoTimeout`，输出保持原值。
DO 读取本地命令缓存；RS485 透传仍保持独立的原始接收缓存和上层超时管理。

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

以下为工具板设备信息接口：

| 序号 | 接口名字 | 参数 | 返回值 | 说明 |
|------|----------|------|--------|------|
| 1 | `NrtGetProtocolVersion()` | 无 | `std::string` | 读取协议版本，来自 `0x00/0x01` |
| 2 | `NrtGetModuleType()` | 无 | `uint32_t` | 读取模块类型，来自 `0x00/0x02` |
| 3 | `NrtGetVendorCode()` | 无 | `uint32_t` | 读取厂商代号，来自 `0x00/0x03` |
| 4 | `NrtGetFwVersion()` | 无 | `std::string` | 读取固件版本，来自 `0x00/0x0D` |
| 5 | `NrtGetHwVersion()` | 无 | `std::string` | 读取硬件版本，来自 `0x00/0x0E` |
| 6 | `NrtGetLastSdoAbortCode()` | 无 | `uint16_t` | 获取最近一次 SDO Abort 码 |

### 3.3 工具 IO 读取与写入

| 序号 | 接口名字 | 参数 | 返回值 | 说明 |
|------|----------|------|--------|------|
| 1 | `RtGetToolSupplyVoltage(voltage_v)` | `voltage_v`: 输出电压，单位 V | `int` | 读取工具板供电电压 |
| 2 | `RtGetToolAnalogInput(index, voltage_v)` | `index`: AI 通道 0/1<br>`voltage_v`: 输出电压，单位 V | `int` | 读取单路 AI 电压 |
| 3 | `RtGetToolDigitalInput(index, value)` | `index`: DI 通道 0/1<br>`value`: 输出状态 | `int` | 读取单路 DI 状态 |
| 4 | `RtGetToolDigitalOutput(index, value)` | `index`: DO 通道 0/1<br>`value`: 输出状态 | `int` | 读取单路 DO 状态 |
| 5 | `RtSetToolDigitalOutput(index, value)` | `index`: DO 通道 0/1<br>`value`: 写入状态 | `int` | 设置单路 DO 状态 |

### 3.4 通用 RS485 透传

SDK 分别提供发送和接收接口：

```cpp
int RtSendToolRs485(const uint8_t* data, std::size_t size);
int RtReceiveToolRs485(uint8_t* data, std::size_t capacity, std::size_t& size, uint32_t* sequence = nullptr, int64_t* timestamp_ns = nullptr);
```

- 发送：调用者提供 `data` 和 `size`，SDK 添加外层头并立即发送，长度范围 1~60 字节。
- 接收：读取最近收到的原始内层数据、长度、序号和接收单调时间；成功才更新输出。
- `capacity` 是接收缓冲区容量；缓冲区不足返回参数错误，所有输出保持原值。
- `sequence` 和 `timestamp_ns` 为可选接收输出指针，传入 `nullptr` 时不读取对应信息。
- 不等待设备应答，不主动查询，不检查内层地址、指令、长度或校验和。
- 接收不消费缓存；多次读取可返回同一序号。使用 `sequence` 判断新帧，`timestamp_ns` 判断时效。
- 缓存保留最新一帧；上层需及时读取，并负责事务匹配、超时、运动队列及反馈快照。
- 上层应协调同一工具板的发送，不让多个协议控制者同时操作同一 RS485 通道。
- 原始收发无动态内存分配；初始化和销毁不得与运行调用并发。

| 项目 | 格式 |
|------|------|
| 请求与应答 CAN ID | `0x600 + node_id`，左臂 40 为 `0x628`，右臂 41 为 `0x629` |
| 外层载荷 | `80 00 00 Len` + 原始 RS485 字节 + CAN-FD 补零 |
| Len | 1~60，只包含原始数据，不包含四字节头和补零 |

发送任意原始字节的示例：

```cpp
uint8_t tx[] = {0x01, 0xAA, 0x55};
int ret = tool_io.RtSendToolRs485(tx, sizeof(tx));

uint8_t rx[60]{};
std::size_t rx_size = 0;
uint32_t sequence = 0;
int64_t timestamp_ns = 0;
ret = tool_io.RtReceiveToolRs485(rx, sizeof(rx), rx_size, &sequence, &timestamp_ns);
```

这些字节只是展示透传接口，不代表任何设备的有效指令。

### 3.5 Robot HAL 的夹爪协议

EG2 的 `EB 90` 请求、`EE 16` 应答、累加和校验、0x41 完整反馈、50 ms 查询、
200 ms 等待期限、500 ms 快照有效期及 16 项写队列均由 Robot HAL 实现。
`ArmHal::RtStepOnce()` 先推进总线，再读取原始应答并协调夹爪查询/写入。
HAL get 读取同一笔完整快照，set 成功表示入队；停止优先清除未发送写指令。

SDK 不再提供 `ToolGripperStatus`、`ToolGripperFeedback` 或 `RtGet/SetToolGripper*` 接口。
协议实现位于 Robot HAL 的 `canevo_hal/src/tool_gripper_eg2.hpp`。
更换设备协议时应在 HAL 编写相应实现，SDK 继续使用这两个透传接口。

EG2 普通串口手册未定义实际电流和夹爪内部电压读取；HAL 电流接口返回不支持，
不构造手册中不存在的夹爪电压指令。
工具板供电电压仍从工具板 PDO 读取，不能冒充夹爪内部电压。

夹爪功能示例已移到 `canevo_hal/example/example_tool_gripper.cpp`，
对应目标 `example_tool_gripper_hal`。SDK 的 `example_tool_rs485` 只打印原始回包。

《CanEvo工具IO通信协议_V1.0.1》将 `0x600 + node_id` 列为“禁用”，
没有记录上述实际固件透传扩展；不能按文档的默认 RxPDO1/TxPDO1 映射实现该通道。
依据该文档表 11，下面的配置项仍缺少编码解释，SDK 不根据猜测修改它们：

| 协议项 | 已明确的定义 | 文档缺少的定义 |
|--------|--------------|----------------|
| `0xE0/0x11` | 串口模式选择，RW，int16 | 透传模式的取值及切换生效条件 |
| `0xE0/0x12` | 串口配置，RW | 数据长度、波特率、校验位和停止位的编码 |
| `0xE3/0x00~0x06` | 默认 TxPDO1 映射六维力 Fx/Fy/Fz/Mx/My/Mz，共 24 字节 | 未记录独立 `0x600 + node_id` 透传应答路径 |
| `0xE7/0x00~0x08` | RxPDO1 映射表，默认映射数量为 0 | 未记录独立透传请求入口和四字节封装头 |

因此不能将默认 TxPDO1 的 24 字节数据当作夹爪普通串口应答，也不能仅根据寄存器名称
推定串口模式或配置取值。实际透传通道及其缓存与 TxPDO1 分开处理。

2026-10-10 在 can0/40、can1/41 只读核对结果相同：串口模式读回 `0x0310`，
串口配置读回两个数据字节 `02 06`；TxPDO1 映射为 `0xE0080020` 至 `0xE00D0020`，
与协议中的六维力映射一致。读取 `0xE7/0x00` 返回未定义索引错误 `0xFFFF`，
说明当前固件与文档在此处也不一致。现有资料不足以解释两个串口配置值的含义。

2026-10-12 恢复 can0/can1 为 CAN-FD（仲裁 1 Mbps、数据 5 Mbps）后，
两侧工具板均在线，CAN 错误计数为 0。早期向错误的 `0x240 + node_id` 无头发送时，
读取及地址扫描均未收到回包；这些结果不能用来判断实际透传通道的设备地址。
按截图修正为 `0x600 + node_id`、添加四字节头并接收同 ID 应答后，
can0/40 的 `--get all --count 3` 连续三轮成功，开度 994、温度 34℃、故障 0、状态 3（停止）。
can1/41 工具板在线，但同样只读探测暂未得到夹爪应答；不能视为右侧夹爪已验证。

`RtSetToolDigitalOutput()` 只修改本地 DO 位图的指定 `index`，再通过 RxPDO0 下发，
不会主动覆盖本地缓存中的另一条 DO 通道；不使用 SDO 读写。

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
cd /home/linux/haozecong/src/modi_robot_server/modi_robot_hardware/robot_joint_hal/driver/modi_canevo_sdk
cmake --build build --target example_tool_io example_tool_rs485 -j1
```

或全量编译：

```bash
cd /home/linux/haozecong/src/modi_robot_server/modi_robot_hardware/robot_joint_hal/driver/modi_canevo_sdk
cmake --build build -j1
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
sudo ./build/example/example_tool_io --can can0 --node 40
```

持续读取：

```bash
sudo ./build/example/example_tool_io --can can0 --node 40 --count 0 --interval-ms 500
```

读取 20 次：

```bash
sudo ./build/example/example_tool_io --can can0 --node 40 --count 20 --interval-ms 500
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
sudo ./build/example/example_tool_io --can can0 --node 40 --do-index 0 --do-value 1
```

关闭 DO0：

```bash
sudo ./build/example/example_tool_io --can can0 --node 40 --do-index 0 --do-value 0
```

打开 DO1：

```bash
sudo ./build/example/example_tool_io --can can0 --node 40 --do-index 1 --do-value 1
```

关闭 DO1：

```bash
sudo ./build/example/example_tool_io --can can0 --node 40 --do-index 1 --do-value 0
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
  if (tool_io.NrtInit(bus, 40) != static_cast<int>(CanEvoError::kOk)) {
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

1. **工具 IO 功能使用实时通道**：AI/DI/DO、工具供电电压使用 PDO，EG2 夹爪使用独立 CAN-FD 透传；均为非阻塞 `Rt` 接口。只有版本、模块类型等配置查询使用 SDO。

2. **工具 IO 不等于关节**：不要用 `modi_joint_canevo::NrtInit()` 初始化工具 IO 节点，否则关节初始化流程会尝试写入同步周期，工具 IO 可能超时或拒绝。

3. **扫描结果需要判断类型**：当前 `NrtScanJoints()` 会返回所有在线节点，不保证只返回关节。工具 IO 示例通过读取 `0xE0/0x00` 判断节点是否支持工具 IO。

4. **DO 会真实输出**：`RtSetToolDigitalOutput()` 会改变 8P 口输出状态，可能驱动外部负载。测试前应确认电源、电流、负载和接线安全。

5. **AI 量程**：AI 通常为 0~10V 模拟输入，不要把 24V 或 48V 直接接入 AI。

6. **DI 输入**：DI 为数字输入，现场测试时按工具板规格接入有效电平，不要超过输入允许范围。

7. **错误判断**：`Rt` 读取返回 `CanEvoError::kPdoTimeout` 表示尚无有效工具 PDO 缓存或夹爪应答尚未就绪；SDO 配置查询返回 `CanEvoError::kSdoAbort` 时，可通过 `NrtGetLastSdoAbortCode()` 查看从站拒绝原因。

8. **RS485 分层**：NodeCore 提供通用 CAN 发送和完整帧转交；
   ToolIO 仅添加/去掉 `80 00 00 Len` 外层头；设备内层协议及查询/写队列由 Robot HAL 实现。

9. **透传排查**：工具板在线及 TxPDO0 正常不代表 RS485 通道已经收到设备数据。
   使用 `example_tool_rs485` 打印原始回包，再由相应 HAL 协议解析器判断设备应答是否有效。
   接收超时不能仅凭 CAN 侧区分板端未转发、串口配置不匹配和设备未应答。
