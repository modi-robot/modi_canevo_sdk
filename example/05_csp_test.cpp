/**
 * @file 05_csp_test.cpp
 * @brief CSP测试demo - 使用控制循环线程接口
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;

// 全局变量用于信号处理
static modi_bus_canevo* g_bus = nullptr;
static modi_joint_canevo* g_joint = nullptr;

// 信号处理函数
void SignalHandler(int signum) {
  g_joint->NrtDisable();

  // 清理资源
  if (g_joint) {
    g_joint->NrtDestroy();
  }
  if (g_bus) {
    g_bus->Close();
    std::cout << "can bus 资源已释放" << std::endl;
  }

  std::exit(signum);
}

int main() {
  std::cout << "========================================" << std::endl;
  std::cout << "CSP 测试程序 - 使用控制循环线程接口" << std::endl;
  std::cout << "========================================" << std::endl;

  // 1. 配置任务参数（控制周期、优先级、CPU亲和性）
  std::cout << "\n配置任务参数..." << std::endl;
  TaskConfig task_config;
  task_config.period_us = 5000;  // 5000 us = 5 ms 控制周期
  task_config.priority = 99;     // 实时优先级 99,最高优先级
  // 绑定到 CPU 2, 对实时性要求高,要绑定到隔离的CPU核心上
  task_config.cpu_affinity = 2;

  // 2. 创建总线和关节对象
  modi_bus_canevo bus;
  modi_joint_canevo joint;

  // 设置全局指针用于信号处理
  g_bus = &bus;
  g_joint = &joint;

  // 注册信号处理函数
  std::signal(SIGINT, SignalHandler);   // Ctrl+C
  std::signal(SIGTERM, SignalHandler);  // kill 命令

  // 3. 打开 CAN 总线（内部会配置 Rx 线程、控制循环线程和锁定内存）
  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 无法打开 CAN 总线" << std::endl;
    return -1;
  }
  std::cout << "✓ CAN 总线已打开 (can0，周期: "
            << (task_config.period_us / 1000.0) << " ms"
            << "，优先级: " << task_config.priority
            << "，CPU: " << task_config.cpu_affinity << ")" << std::endl;

  // 4. 配置超时
  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  // 5. 初始化关节
  if (joint.NrtInit(bus, 1) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节初始化失败" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "✓ Joint 初始化完成 (Node ID: 1" << std::endl;
  // 6. 设置机械零点
  joint.NrtDisable();
  while (joint.NrtGetServoState() != CanEvoServoState::kReady) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  
  joint.NrtSetMechZero();
  while (joint.NrtGetMechZeroOk() != 1) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << "✓ 机械零点已设置" << std::endl;
  if (joint.NrtEnable(CanEvoMode::kCsp) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节使能失败" << std::endl;
    bus.Close();
    return -1;
  }
  while (joint.NrtGetServoState() != CanEvoServoState::kRunning) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << "✓ 关节已使能" << std::endl;
  // 实时控制: 启动控制循环，定义控制循环回调（使用 Lambda 捕获关节）
  float target_pos_rad = 0.0f;  // 初始位置
  auto ret = bus.StartControlLoop([&joint, &target_pos_rad]() {
    // 每次步进+1度
    target_pos_rad += 1.0f * kDegToRad;
    joint.RtSetCspTargetPosition(target_pos_rad);
  });
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 启动控制循环失败" << std::endl;
    return -1;
  }

  std::cout << "控制循环运行中，按 Ctrl+C 终止..." << std::endl;
  bus.Join();
  return 0;
}
