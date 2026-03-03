# CanEvo SDK x86

CanEvo CAN-FD 关节控制 C++ SDK，适用于 x86 (Linux/SocketCAN) 平台。

## 简介

本 SDK 提供了基于 CanEvo V1.2.3 CAN-FD 协议的关节控制接口，支持在 Linux 平台上通过 SocketCAN 进行实时关节控制。

### 主要特性

- **Bus + Joint 两级架构**：支持同一总线上挂载多个关节节点
- **实时控制**：PDO 接口采用 SPSC 无锁队列 + 专用 Tx 线程，保证非阻塞实时性
- **同步配置**：SDO 接口为同步阻塞调用，适合初始化/配置阶段
- **PIMPL 设计**：隐藏实现细节，公开头文件无平台依赖
- **单位自动转换**：SDK 内部自动完成 rad↔deg、rad/s↔rpm 等单位转换

### 支持的工作模式

- **CSP (Cyclic Synchronous Position)**：周期同步位置模式
- **CSV (Cyclic Synchronous Velocity)**：周期同步速度模式
- **CST (Cyclic Synchronous Torque)**：周期同步转矩模式
- **PP (Profile Position)**：轮廓位置模式

## 系统要求

- **操作系统**：Linux（支持 SocketCAN）
- **编译器**：支持 C++17 的编译器（GCC 7+ / Clang 5+）
- **构建工具**：CMake 3.14+
- **依赖库**：pthread

## 快速开始

### 1. 编译

```bash
# 创建构建目录
mkdir build && cd build

# 配置 CMake
cmake ..

# 编译
make -j$(nproc)

# 安装（默认安装到 build/install）
make install
```

### 2. 基本使用

```cpp
#include "modi_joint_canevo.h"

int main() {
    // 1. 打开 CAN 总线
    modi_bus_canevo bus;
    bus.Open("can0");
    
    // 2. 初始化关节
    modi_joint_canevo joint;
    joint.NrtInit(bus, 1);  // node_id = 1
    
    // 3. 设置工作模式并使能
    joint.RtSetWorkMode(CanEvoMode::kCsp);
    joint.RtEnable();
    
    // 4. 实时控制循环
    uint8_t sync_counter = 0;
    while (running) {
        // 发送 SYNC 帧
        bus.RtSendSync(sync_counter++);
        
        // 读取关节状态
        JointStatus status;
        joint.RtGetJointStatus(status);
        
        // 设置目标位置
        joint.RtSetCspTargetPosition(target_pos_rad);
    }
    
    // 5. 清理资源
    joint.RtDisable();
    joint.NrtDestroy();
    bus.Close();
    
    return 0;
}
```

### 3. 运行示例

```bash
# 编译示例
cd build
make example_csp

# 运行示例（需要 root 权限或配置 CAN 接口）
sudo ./example/example_csp
```

## 项目结构

```
canevo_sdk_x86/
├── CMakeLists.txt          # 主 CMake 配置文件
├── include/                # 公开头文件
│   ├── modi_joint_canevo.h # 主要 API 头文件
│   └── canevo_impl.h       # 内部实现头文件
├── src/                    # 源文件
│   ├── modi_joint_canevo.cpp
│   └── canevo_impl.cpp
├── example/                # 示例代码
│   ├── CMakeLists.txt
│   └── example_csp.cpp    # CSP 模式示例
├── docs/                   # 文档
│   ├── ModiJointCanevo 接口文档.md
│   └── ModiJointCanevo 接口文档.pdf
└── README.md               # 本文件
```

## 接口说明

### 接口命名规范

- **Rt 前缀**：Real-time（实时）接口，用于 PDO 相关操作，非阻塞，适合实时控制循环
  - 例如：`RtSetCspTargetPosition()`、`RtGetJointStatus()`、`RtEnable()`
  
- **Nrt 前缀**：Non-real-time（非实时）接口，用于 SDO 相关操作，阻塞调用，适合初始化/配置阶段
  - 例如：`NrtInit()`、`NrtDestroy()`、`NrtGetProtocolVersion()`

### 核心类

#### `modi_bus_canevo`

总线管理类，对应一条 SocketCAN 接口（如 can0）。

主要接口：
- `Open(can_ifname)`：打开 CAN 总线
- `Close()`：关闭总线
- `RtSendSync(counter)`：发送 SYNC 同步帧

#### `modi_joint_canevo`

关节控制类，对应总线上一个节点（node_id 1~62）。

主要接口：
- **生命周期**：`NrtInit()`、`NrtDestroy()`
- **实时控制**：`RtSetCspTargetPosition()`、`RtGetJointStatus()`、`RtEnable()`、`RtDisable()`
- **SDO 配置**：`NrtGetProtocolVersion()`、`NrtSetMaxSpeed()` 等

## 单位约定

SDK 公开接口统一使用工程单位，内部自动完成与协议层单位的转换：

- **位置**：rad（弧度）
- **速度**：rad/s
- **加速度/减速度**：rad/s²
- **电流**：A（安培）
- **温度**：℃
- **电压**：V

## 安装路径

默认安装到 `build/install` 目录：

```
build/install/
├── include/
│   └── modi_joint_canevo.h
├── lib/
│   ├── libcanevo_sdk_x86.so
│   └── cmake/
│       └── canevo_sdk_x86/
│           └── canevo_sdk_x86Targets.cmake
└── bin/
```

## 使用已安装的 SDK

### CMake 方式

```cmake
find_package(canevo_sdk_x86 REQUIRED)
target_link_libraries(your_target PRIVATE canevo_sdk_x86::canevo_sdk_x86)
```

### 直接链接方式

```bash
# 编译时指定包含目录和库路径
g++ your_app.cpp -I/path/to/install/include \
                 -L/path/to/install/lib \
                 -lcanevo_sdk_x86 \
                 -lpthread \
                 -o your_app
```

## 文档

详细的接口文档请参考：

- [ModiJointCanevo 接口文档.md](docs/ModiJointCanevo%20接口文档.md)
- [ModiJointCanevo 接口文档.pdf](docs/ModiJointCanevo%20接口文档.pdf)

## 代码风格

项目使用基于 [Google C++ 风格指南](https://zh-google-styleguide.readthedocs.io/en/latest/google-cpp-styleguide/) 的 clang-format 配置，代码格式化：

```bash
clang-format -i src/*.cpp include/*.h example/*.cpp
```

## 注意事项

1. **实时控制循环**：建议控制频率 200Hz~1000Hz，使用 Rt 接口进行实时控制
2. **SDO 调用**：Nrt 接口为阻塞调用，不应在实时控制循环中使用
3. **权限要求**：访问 SocketCAN 接口通常需要 root 权限，或配置适当的用户权限
4. **线程安全**：PDO 实时循环建议单线程调度，减少周期抖动