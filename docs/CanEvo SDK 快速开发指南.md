# CanEvo SDK 快速开发指南

本文面向第一次使用 CanEvo SDK 的开发者，从创建工程开始，完成 CAN 总线打开、关节扫描、状态读取和 PP 位置控制。

在开始前，请先按照 [CanEvo SDK 使用说明书](CanEvo%20SDK%20使用说明书.md) 安装驱动、配置 CAN-FD 接口并解压对应架构的 SDK 发布包。

> 运动控制具有风险。首次运行时应固定关节、准备急停，并使用较小的速度、加速度和运动范围。

## 1. SDK 对象关系

CanEvo SDK 的基本调用关系如下：

```text
modi_bus_canevo
  ├── 打开一个 SocketCAN 接口
  ├── 扫描在线关节
  └── 管理通信线程和超时
        │
        └── modi_joint_canevo
              ├── 绑定一个关节 ID
              ├── 读取状态和参数
              └── 使能、失能并下发运动目标
```

一个程序通常只创建一个总线对象，每个在线关节分别创建一个关节对象。

## 2. 创建最小工程

假设 SDK 已解压，并设置了 `SDK_ROOT`：

```bash
export SDK_ROOT=/实际路径/modi_canevo_sdk_<版本>_<平台>_<架构>_<Git短提交>
mkdir -p ~/canevo_first_app
cd ~/canevo_first_app
```

工程目录：

```text
canevo_first_app/
├── CMakeLists.txt
└── main.cpp
```

创建 `CMakeLists.txt`：

```cmake
cmake_minimum_required(VERSION 3.14)
project(canevo_first_app LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(canevo_sdk_x86 CONFIG REQUIRED)
find_package(Threads REQUIRED)

add_executable(canevo_first_app main.cpp)
target_link_libraries(canevo_first_app PRIVATE
    canevo_sdk_x86::canevo_sdk_x86
    Threads::Threads
)
```

`canevo_sdk_x86` 是当前 SDK 的 CMake 包名，在 x86_64 和 ARM64 发布包中使用方式相同。

## 3. 第一个程序：扫描并读取关节状态

创建 `main.cpp`：

```cpp
#include <cstdint>
#include <iomanip>
#include <iostream>

#include "modi_joint_canevo.h"

int main(int argc, char* argv[]) {
  const char* can_ifname = argc > 1 ? argv[1] : "can0";

  // 1. 创建总线对象并打开已经配置好的 SocketCAN 接口。
  modi_bus_canevo bus;
  TaskConfig task_config;
  task_config.sync_period_us = 5000;

  if (bus.Open(can_ifname, task_config) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 接口失败: " << can_ifname << std::endl;
    return 1;
  }

  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  // 2. 扫描在线关节。
  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return 1;
  }

  std::cout << "在线关节 ID:";
  for (const uint8_t id : joint_ids) {
    std::cout << " " << static_cast<int>(id);
  }
  std::cout << std::endl;

  // 3. 绑定扫描到的第一个关节。
  modi_joint_canevo joint;
  const uint8_t node_id = joint_ids.front();
  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化关节失败, ID=" << static_cast<int>(node_id)
              << std::endl;
    bus.Close();
    return 1;
  }

  // 4. 读取关节信息和当前状态。
  std::cout << "固件版本: " << joint.NrtGetFwVersion() << std::endl;
  std::cout << "实际位置(rad): " << joint.NrtGetActualPosition() << std::endl;
  std::cout << "实际速度(rad/s): " << joint.NrtGetActualVelocity() << std::endl;
  std::cout << "故障码: 0x" << std::hex
            << static_cast<uint16_t>(joint.NrtGetFaultCode()) << std::dec
            << std::endl;

  // 5. 按照“关节后总线”的顺序释放资源。
  joint.NrtDestroy();
  bus.Close();
  return 0;
}
```

这个程序只读取信息，不会使能关节或下发运动目标，适合作为首次通信测试。

## 4. 编译与运行

在工程目录执行：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_PREFIX_PATH="${SDK_ROOT}"
cmake --build build --parallel "$(nproc)"
```

使用 `can0` 运行：

```bash
sudo ./build/canevo_first_app can0
```

如果使用其他 CAN 接口，例如 `can1`：

```bash
sudo ./build/canevo_first_app can1
```

## 5. 增加 PP 位置控制

确认关节无故障且机械环境安全后，可以将 `main.cpp` 替换为下面的完整 PP 位置控制程序。程序默认使用 `can0` 和扫描到的第一个关节，也可以通过命令行指定 CAN 接口。

```cpp
#include <cmath>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>

#include "modi_joint_canevo.h"

namespace {

constexpr float kTargetPositionRad = 0.1f;
constexpr float kProfileVelocityRadS = 0.2f;
constexpr float kProfileAccelerationRadSS = 0.3f;
constexpr float kPositionToleranceRad = 0.01f;
constexpr auto kStateTimeout = std::chrono::seconds(3);
constexpr auto kMotionTimeout = std::chrono::seconds(10);

bool WaitUntilRunning(modi_joint_canevo& joint) {
  const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (joint.NrtGetServoState() == CanEvoServoState::kRunning &&
        joint.NrtGetCurrentMode() == CanEvoMode::kPp) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

}  // namespace

int main(int argc, char* argv[]) {
  const char* can_ifname = argc > 1 ? argv[1] : "can0";

  // 1. 打开已经配置并启动的 CAN-FD 接口。
  modi_bus_canevo bus;
  TaskConfig task_config;
  task_config.sync_period_us = 5000;

  if (bus.Open(can_ifname, task_config) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 接口失败: " << can_ifname << std::endl;
    return 1;
  }

  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  // 2. 扫描并选择第一个在线关节。
  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return 1;
  }

  const uint8_t node_id = joint_ids.front();
  std::cout << "选择关节 ID: " << static_cast<int>(node_id) << std::endl;

  // 3. 初始化关节对象。
  modi_joint_canevo joint;
  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化关节失败" << std::endl;
    bus.Close();
    return 1;
  }

  bool enabled = false;
  const auto cleanup = [&]() {
    if (enabled) {
      joint.NrtDisable();
    }
    joint.NrtDestroy();
    bus.Close();
  };

  // 4. 检查并尝试清除故障。
  const auto fault = joint.NrtGetFaultCode();
  if (fault != CanEvoFault::kNone) {
    std::cout << "检测到故障，尝试清除，故障码: 0x" << std::hex
              << static_cast<uint16_t>(fault) << std::dec << std::endl;
    if (joint.NrtClearFault() != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "清除故障失败" << std::endl;
      cleanup();
      return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // 5. 切换到 PP 模式并使能关节。
  if (joint.NrtEnable(CanEvoMode::kPp) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "使能 PP 模式失败" << std::endl;
    cleanup();
    return 1;
  }
  enabled = true;

  if (!WaitUntilRunning(joint)) {
    std::cerr << "等待关节进入 PP Running 状态超时" << std::endl;
    cleanup();
    return 1;
  }

  // 6. 下发位置、速度和加速度目标。
  if (joint.NrtSetPpTargetPosition(kTargetPositionRad,
                                   kProfileVelocityRadS,
                                   kProfileAccelerationRadSS) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "下发 PP 目标失败" << std::endl;
    cleanup();
    return 1;
  }

  std::cout << "目标位置: " << kTargetPositionRad << " rad" << std::endl;

  // 7. 等待关节到达目标位置。
  bool reached = false;
  const auto deadline = std::chrono::steady_clock::now() + kMotionTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (joint.NrtGetServoState() == CanEvoServoState::kFault) {
      std::cerr << "运动过程中检测到关节故障" << std::endl;
      break;
    }

    const float actual_position = joint.NrtGetActualPosition();
    std::cout << "当前位置: " << actual_position << " rad\r" << std::flush;
    if (std::fabs(kTargetPositionRad - actual_position) <=
        kPositionToleranceRad) {
      reached = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  std::cout << std::endl;

  if (!reached) {
    std::cerr << "关节未在规定时间内到达目标位置" << std::endl;
  } else {
    std::cout << "关节已到达目标位置" << std::endl;
  }

  // 8. 无论是否到位，都先失能关节，再释放通信资源。
  cleanup();
  return reached ? 0 : 1;
}
```

重新编译并运行：

```bash
cmake --build build --parallel "$(nproc)"
sudo ./build/canevo_first_app can0
```

使用其他 CAN 接口时，将最后一个参数改为实际接口名，例如：

```bash
sudo ./build/canevo_first_app can1
```

程序在成功、超时和错误路径中都会执行清理逻辑：先调用 `NrtDisable()`，再调用 `NrtDestroy()`，最后调用 `bus.Close()`。

## 6. 创建实时线程

CSP、CSV 和 CST 等固定周期控制应在实时线程中执行。建议先运行包内的纯实时线程示例，确认线程能够使用 `SCHED_FIFO` 并绑定到指定 CPU：

```bash
cd "${SDK_ROOT}/example"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target example_single_joint_rt_thread \
  --parallel "$(nproc)"
sudo ./build/example_single_joint_rt_thread
```

完整源码位于：

```text
example/example_single_joint/example_rt_thread.cpp
```

### 6.1 实时线程的基本配置

创建 POSIX 实时线程时需要完成以下配置：

```cpp
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

constexpr int kRealtimeCpu = 1;
constexpr int kRealtimePriority = 99;

// 锁定当前和后续分配的内存，避免实时循环发生换页。
if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
  std::cerr << "mlockall 失败" << std::endl;
  return 1;
}

pthread_attr_t attr;
pthread_attr_init(&attr);
pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
pthread_attr_setschedpolicy(&attr, SCHED_FIFO);

sched_param param{};
param.sched_priority = kRealtimePriority;
pthread_attr_setschedparam(&attr, &param);

cpu_set_t cpuset;
CPU_ZERO(&cpuset);
CPU_SET(kRealtimeCpu, &cpuset);
pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);

pthread_t rt_thread{};
const int ret = pthread_create(&rt_thread, &attr, RtLoop, nullptr);
pthread_attr_destroy(&attr);
if (ret != 0) {
  std::cerr << "创建实时线程失败, errno=" << ret << std::endl;
  return 1;
}

pthread_join(rt_thread, nullptr);
```

其中 `RtLoop` 是实时线程入口函数。`CPU1` 只是本文采用的配置，运行前应确认该 CPU 在线，并且与系统实际隔离核一致：

```bash
cat /sys/devices/system/cpu/online
cat /sys/devices/system/cpu/isolated
```

`SCHED_FIFO 99` 和 `mlockall()` 通常需要 root 或相应实时调度权限，因此示例使用 `sudo` 运行。

### 6.2 固定周期循环

固定周期循环应使用 `CLOCK_MONOTONIC` 和绝对时间唤醒，避免每周期误差不断累积：

```cpp
timespec AddNs(timespec value, long ns) {
  value.tv_nsec += ns;
  while (value.tv_nsec >= 1000000000L) {
    value.tv_nsec -= 1000000000L;
    ++value.tv_sec;
  }
  return value;
}

void* RtLoop(void*) {
  constexpr long kPeriodNs = 5'000'000L;  // 5 ms
  timespec next_wakeup{};
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

  while (g_running.load(std::memory_order_acquire)) {
    next_wakeup = AddNs(next_wakeup, kPeriodNs);
    const int ret = clock_nanosleep(
        CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
    if (ret != 0) {
      break;
    }

    // 在这里执行一次固定周期控制。
  }
  return nullptr;
}
```

实时循环中不要打印日志、写文件、动态分配内存或调用可能阻塞的 `Nrt` 接口。状态记录可写入预先分配的内存，退出实时线程后再由普通线程输出。

## 7. CSP 实时位置控制

CSP 模式由应用程序按照固定周期持续发送目标位置。与 PP 不同，CSP 不能只发送一次目标后等待，必须在每个同步周期调用 `RtStepOnce()` 并更新目标。

完整可运行示例位于：

```text
example/example_single_joint/example_csp.cpp
```

编译并运行：

```bash
cd "${SDK_ROOT}/example"
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target example_single_joint_csp \
  --parallel "$(nproc)"
sudo ./build/example_single_joint_csp
```

### 7.1 主线程初始化顺序

主线程负责所有可能阻塞的初始化操作：

```cpp
TaskConfig task_config;
task_config.sync_period_us = 5000;
task_config.cpu_affinity = 3;   // SDK Rx 线程使用非隔离 CPU
task_config.sched_policy = SCHED_FIFO;
task_config.sched_priority = 90;

modi_bus_canevo bus;
modi_joint_canevo joint;

bus.Open("can0", task_config);
const auto joint_ids = bus.NrtScanJoints();
joint.NrtInit(bus, joint_ids.front());

if (joint.NrtGetFaultCode() != CanEvoFault::kNone) {
  joint.NrtClearFault();
}

joint.NrtEnable(CanEvoMode::kCsp);
```

实际程序必须检查每个接口的返回值，并等待：

```text
NrtGetServoState() == CanEvoServoState::kRunning
NrtGetCurrentMode() == CanEvoMode::kCsp
```

确认模式切换完成后再创建实时线程。完整的错误处理与等待逻辑请直接参考 `example_csp.cpp`。

### 7.2 CSP 实时循环

实时线程每周期执行以下顺序：

```cpp
void* RtLoop(void*) {
  timespec next_wakeup{};
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

  const long period_ns = g_period_us * 1000L;
  float target_position = g_joint->NrtGetActualPosition();
  float step = 0.02f * 3.1415926f / 180.0f;

  while (g_running.load(std::memory_order_acquire)) {
    next_wakeup = AddNs(next_wakeup, period_ns);
    if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                        &next_wakeup, nullptr) != 0) {
      break;
    }

    // 必须每周期调用一次，完成同步帧发送和实时缓存更新。
    if (g_bus->RtStepOnce() != static_cast<int>(CanEvoError::kOk)) {
      break;
    }

    if (g_joint->RtGetServoState() == CanEvoServoState::kFault) {
      break;
    }
    if (g_joint->RtGetCurrentMode() != CanEvoMode::kCsp) {
      break;
    }

    target_position += step;
    if (g_joint->RtSetCspTargetPosition(target_position) !=
        static_cast<int>(CanEvoError::kOk)) {
      break;
    }
  }
  return nullptr;
}
```

这里使用全局指针只是为了突出调用顺序。业务程序可以将总线、关节和退出状态封装到上下文结构体中，通过 `pthread_create()` 的参数传入实时线程。

### 7.3 CSP 退出顺序

退出时需要主线程和实时线程配合，推荐顺序：

1. 主线程收到 `SIGINT` 或业务停止请求。
2. 调用 `NrtDisable()` 失能关节。
3. 让实时循环短暂保持周期通信，完成失能状态切换。
4. 设置实时线程退出标志并调用 `pthread_join()`。
5. 调用 `NrtDestroy()`。
6. 调用 `bus.Close()`。

不要直接结束进程或先关闭 CAN 总线，否则关节可能来不及收到失能指令。

## 8. 常用接口调用顺序

| 阶段 | 接口 | 说明 |
| --- | --- | --- |
| 打开总线 | `bus.Open()` | 打开已配置并启动的 SocketCAN 接口 |
| 配置超时 | `SetSdoTimeoutMs()`、`SetPdoTimeoutMs()` | 设置 SDO/PDO 通信超时 |
| 扫描设备 | `bus.NrtScanJoints()` | 获取在线关节 ID 列表 |
| 初始化关节 | `joint.NrtInit()` | 将关节对象绑定到总线和节点 ID |
| 检查状态 | `NrtGetFaultCode()`、`NrtGetServoState()` | 运动前检查故障和伺服状态 |
| 清除故障 | `joint.NrtClearFault()` | 尝试清除当前关节故障 |
| 使能模式 | `joint.NrtEnable(mode)` | 选择控制模式并使能关节 |
| 下发目标 | `NrtSetPpTargetPosition()` 等 | 按控制模式发送目标值 |
| 安全停止 | `joint.NrtDisable()` | 停止运动并失能关节 |
| 释放资源 | `joint.NrtDestroy()`、`bus.Close()` | 注销关节并关闭 CAN 总线 |

## 9. NRT 与 RT 接口怎么选择

- `Nrt` 接口可能发生阻塞，适合初始化、参数配置、状态确认和低频控制。
- RT/PDO 接口适合固定周期控制循环，不应在实时循环中调用阻塞式 `Nrt` 接口。
- 初次开发建议先完成 SDO 与 NRT PP 测试，再参考 CSP、CSV、CST 示例编写实时控制程序。

## 10. 下一步参考

- [完整示例源码](../example/)
- [CanEvo SDK 使用说明书](CanEvo%20SDK%20使用说明书.md)
- [ModiJointCanevo 接口文档](ModiJointCanevo%20接口文档.md)
- [CanEvo 关节通信协议](CanEvo关节通信协议_V1.2.4.md)
- [CanEvo 实时系统配置指南](CanEvo实时系统配置指南.md)

建议按照以下顺序阅读示例：

1. `example_single_joint/example_sdo.cpp`
2. `example_single_joint/example_nrt_pp.cpp`
3. `example_single_joint/example_csv.cpp`
4. `example_single_joint/example_csp.cpp`
5. `example_multi_joints/` 下的多关节示例
