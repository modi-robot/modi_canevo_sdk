/**
 * @file example_pp.cpp
 * @brief PP模式测试 - 通过 SDO 一次性下发轮廓参数，到达后退出
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 */

#include <time.h>

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = 3.1415926f / 180.0f;

int main() {
  modi_bus_canevo bus;
  if (bus.Open("can0") != 0) {
    std::cerr << "打开失败" << std::endl;
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
    if (joints[i].NrtInit(bus, joint_ids[i]) != 0) {
      std::cerr << "初始化失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      for (size_t j = 0; j < i; ++j) {
        joints[j].NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  for (auto& joint : joints) {
    joint.NrtDisable();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  for (auto& joint : joints) {
    joint.NrtSetMechZero();
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].NrtEnable(CanEvoMode::kPp) != 0) {
      std::cerr << "使能 PP 失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      for (auto& joint : joints) {
        joint.NrtDisable();
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    const int ret = joints[i].NrtSetPpTargetPosition(6.283f, 0.524f, 0.524f);
    if (ret != 0) {
      std::cerr << "下发PP目标失败, ID=" << static_cast<int>(joint_ids[i])
                << ", ret=" << ret << std::endl;
      for (auto& joint : joints) {
        joint.NrtDisable();
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  while (true) {
    bool all_reached = true;
    for (auto& joint : joints) {
      const float pos = joint.NrtGetActualPosition();
      if (std::abs(pos - 6.283f) >= 0.017f) {
        all_reached = false;
        break;
      }
    }
    if (all_reached) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  for (auto& joint : joints) {
    joint.NrtDisable();
    joint.NrtDestroy();
  }
  bus.Close();

  std::cout << "全部关节到达360°，完成" << std::endl;
  return 0;
}
