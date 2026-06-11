/**
 * @file example_csv.cpp
 * @brief CSV速度模式测试 (10 rpm 正转3秒 → 反转3秒)
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 *
 * 测试内容：
 * 1. 使能CSV模式
 * 2. 9.55 rpm 正转3秒
 * 3. 暂停1秒
 * 4. 9.55 rpm 反转3秒
 * 5. 停止
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <vector>

#include "modi_joint_canevo.h"

int main() {
  TaskConfig task_config;
  task_config.sync_period_us = 5000;
  task_config.cpu_affinity = 2;

  modi_bus_canevo bus;
  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 总线失败" << std::endl;
    return -1;
  }

  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "扫描到 " << joint_ids.size() << " 个关节, ID: ";
  for (const auto id : joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  std::vector<modi_joint_canevo> joints(joint_ids.size());
  for (size_t i = 0; i < joint_ids.size(); ++i) {
    if (joints[i].NrtInit(bus, joint_ids[i]) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "初始化关节失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      for (size_t j = 0; j < i; ++j) {
        joints[j].NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].NrtEnable(CanEvoMode::kCsv) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "使能 CSV 模式失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      for (auto& joint : joints) {
        joint.NrtDisable();
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  auto run_velocity = [&bus, &joints, &task_config](
                          float target_rads,
                          std::chrono::milliseconds duration) {
    auto next_wakeup = std::chrono::steady_clock::now();
    const auto deadline = next_wakeup + duration;
    while (std::chrono::steady_clock::now() < deadline) {
      for (auto& joint : joints) {
        const int set_ret = joint.RtSetCsvTargetVelocity(target_rads);
        if (set_ret != static_cast<int>(CanEvoError::kOk)) return set_ret;
      }
      const int ret = bus.RtStepOnce();
      if (ret != static_cast<int>(CanEvoError::kOk)) return ret;
      next_wakeup += std::chrono::microseconds(task_config.sync_period_us);
      std::this_thread::sleep_until(next_wakeup);
    }
    return static_cast<int>(CanEvoError::kOk);
  };

  int ret = run_velocity(1.0f, std::chrono::seconds(3));
  if (ret == static_cast<int>(CanEvoError::kOk)) {
    ret = run_velocity(0.0f, std::chrono::seconds(1));
  }
  if (ret == static_cast<int>(CanEvoError::kOk)) {
    ret = run_velocity(-1.0f, std::chrono::seconds(3));
  }
  if (ret == static_cast<int>(CanEvoError::kOk)) {
    ret = run_velocity(0.0f, std::chrono::milliseconds(200));
  }
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "RtStepOnce 失败, ret=" << ret << std::endl;
  }

  for (auto& joint : joints) {
    joint.NrtDisable();
    joint.NrtDestroy();
  }
  bus.Close();

  return 0;
}
