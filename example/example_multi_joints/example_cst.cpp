/**
 * @file example_cst.cpp
 * @brief 多关节 CST 电流模式测试 - 全部扫描到的关节同时执行
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
    if (joints[i].NrtEnable(CanEvoMode::kCst) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "使能 CST 模式失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      for (auto& joint : joints) {
        joint.NrtDisable();
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  auto next_wakeup = std::chrono::steady_clock::now();
  const auto deadline = next_wakeup + std::chrono::seconds(3);
  bool run_ok = true;
  while (run_ok && std::chrono::steady_clock::now() < deadline) {
    for (auto& joint : joints) {
      const int set_ret = joint.RtSetCstTargetCurrent(0.0f);
      if (set_ret != static_cast<int>(CanEvoError::kOk)) {
        std::cerr << "RtSetCstTargetCurrent 失败, ret=" << set_ret
                  << std::endl;
        run_ok = false;
        break;
      }
    }
    if (!run_ok) break;
    const int ret = bus.RtStepOnce();
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "RtStepOnce 失败, ret=" << ret << std::endl;
      break;
    }
    next_wakeup += std::chrono::microseconds(task_config.sync_period_us);
    std::this_thread::sleep_until(next_wakeup);
  }

  for (auto& joint : joints) {
    joint.NrtDisable();
    joint.NrtDestroy();
  }
  bus.Close();
  return 0;
}
