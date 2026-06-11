/**
 * @file example_pp.cpp
 * @brief PP模式测试 - 回零后在 +90° / -90° 间往复运动
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 */

#include <chrono>
#include <cmath>
#include <csignal>
#include <iostream>
#include <thread>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = 3.1415926f / 180.0f;
constexpr float kRadToDeg = 180.0f / 3.1415926f;
constexpr float kTargetToleranceRad = 0.001f * kDegToRad;
constexpr float kProfileVelRadS = 1.0f;
constexpr float kProfileAccRadSS = 1.0f;
constexpr auto kPollInterval = std::chrono::milliseconds(50);
constexpr auto kMoveTimeout = std::chrono::seconds(60);
constexpr auto kStateTimeout = std::chrono::seconds(3);

// 全局变量用于信号处理
static modi_bus_canevo* g_bus = nullptr;
static modi_joint_canevo* g_joint = nullptr;

// 信号处理函数
void SignalHandler(int signum) {
  if (g_joint) {
    g_joint->NrtDisable();
    g_joint->NrtDestroy();
  }
  if (g_bus) {
    g_bus->Close();
    std::cout << "can bus 资源已释放" << std::endl;
  }

  std::exit(signum);
}

bool WaitServoState(modi_joint_canevo& joint, CanEvoServoState target,
                    const char* action) {
  const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (joint.NrtGetServoState() == target) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << action << "超时" << std::endl;
  return false;
}

bool WaitPositionReached(modi_joint_canevo& joint, float target_pos_rad) {
  const auto deadline = std::chrono::steady_clock::now() + kMoveTimeout;

  while (std::chrono::steady_clock::now() < deadline) {
    const float actual_pos_rad = joint.NrtGetActualPosition();
    const float error_rad = target_pos_rad - actual_pos_rad;

    std::cout << "目标: " << target_pos_rad * kRadToDeg
              << " deg, 当前: " << actual_pos_rad * kRadToDeg
              << " deg, 误差: " << error_rad * kRadToDeg << " deg" << std::endl;

    if (std::abs(error_rad) <= kTargetToleranceRad) {
      return true;
    }

    std::this_thread::sleep_for(kPollInterval);
  }

  std::cerr << "等待到位超时" << std::endl;
  return false;
}

bool MoveTo(modi_joint_canevo& joint, float target_pos_rad) {
  const float actual_pos_rad = joint.NrtGetActualPosition();
  if (std::abs(target_pos_rad - actual_pos_rad) <= kTargetToleranceRad) {
    std::cout << "当前位置已在目标附近: " << actual_pos_rad * kRadToDeg
              << " deg" << std::endl;
    return true;
  }

  int ret = joint.NrtSetPpTargetPosition(target_pos_rad, kProfileVelRadS,
                                         kProfileAccRadSS);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "下发 PP 目标位置失败, ret=" << ret << std::endl;
    return false;
  }
  return WaitPositionReached(joint, target_pos_rad);
}

int main() {
  std::cout << "\n========== 单关节 PP 示例 ==========\n" << std::endl;

  // 1. 配置任务参数（控制周期、CPU亲和性）
  std::cout << "\n配置任务参数..." << std::endl;
  TaskConfig task_config;
  task_config.sync_period_us = 5000;  // 5000 us = 5 ms 控制周期
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

  // 3. 打开已手动配置好的 CAN 总线（内部会配置 Rx
  // 线程和锁定内存）
  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 无法打开 CAN 总线" << std::endl;
    return -1;
  }
  std::cout << "✓ CAN 总线已打开 (can0，周期: "
            << (task_config.sync_period_us / 1000.0) << " ms"
            << "，CPU: " << task_config.cpu_affinity << ")" << std::endl;

  // 4. 配置超时
  bus.SetPdoTimeoutMs(50);

  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "✗ 未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "✓ 扫描到 " << joint_ids.size() << " 个关节, ID: ";
  for (const auto id : joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;
  const uint8_t node_id = joint_ids.front();
  std::cout << "✓ 默认选择第一个关节 ID: " << static_cast<int>(node_id)
            << std::endl;

  // 5. 初始化关节
  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节初始化失败" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "✓ Joint 初始化完成 (Node ID: " << static_cast<int>(node_id)
            << ")" << std::endl;

  if (joint.NrtEnable(CanEvoMode::kPp) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节使能失败" << std::endl;
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  while (joint.NrtGetServoState() != CanEvoServoState::kRunning) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << "✓ 关节已使能" << std::endl;

  auto ok = MoveTo(joint, 0.0f);
  if (!ok) {
    std::cerr << "✗ 关节回零失败" << std::endl;
  }

  for (int i = 0; ok && i < 5; ++i) {
    std::cout << "\n第 " << (i + 1) << " 次运动到 +90 deg" << std::endl;
    ok = MoveTo(joint, 90.0f * kDegToRad);
    if (!ok) break;

    std::cout << "\n第 " << (i + 1) << " 次运动到 -90 deg" << std::endl;
    ok = MoveTo(joint, -90.0f * kDegToRad);
  }

  joint.NrtDisable();
  joint.NrtDestroy();
  bus.Close();

  std::cout << "\n========== PP 示例" << (ok ? "完成" : "失败")
            << " ==========\n"
            << std::endl;
  return ok ? 0 : -1;
}
