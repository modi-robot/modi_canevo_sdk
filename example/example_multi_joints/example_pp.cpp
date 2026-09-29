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

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = 3.1415926f / 180.0f;

void PrintUsage(const char* program) {
  std::cout << "用法:\n"
            << "  " << program << " [joint_id ...]\n\n"
            << "示例:\n"
            << "  " << program << "        # 不传参数，默认使用扫描到的第一个关节\n"
            << "  " << program << " 8      # 只下发 8 号关节\n"
            << "  " << program << " 8 9 10 # 只下发 8、9、10 号关节\n";
}

bool ParseNodeId(const char* text, uint8_t& node_id) {
  char* end = nullptr;
  const long value = std::strtol(text, &end, 0);
  if (end == text || *end != '\0' || value < 1 || value > 62) {
    return false;
  }
  node_id = static_cast<uint8_t>(value);
  return true;
}

bool ContainsNodeId(const std::vector<uint8_t>& ids, const uint8_t node_id) {
  return std::find(ids.begin(), ids.end(), node_id) != ids.end();
}

int main(int argc, char* argv[]) {
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--help" || arg == "-h") {
      PrintUsage(argv[0]);
      return 0;
    }
  }

  modi_bus_canevo bus;
  if (bus.Open("can0") != 0) {
    std::cerr << "打开失败" << std::endl;
    return -1;
  }

  const auto scanned_joint_ids = bus.NrtScanJoints();
  if (scanned_joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }

  std::vector<uint8_t> joint_ids;
  for (int i = 1; i < argc; ++i) {
    uint8_t node_id = 0;
    if (!ParseNodeId(argv[i], node_id)) {
      std::cerr << "无效关节号: " << argv[i] << std::endl;
      bus.Close();
      return -1;
    }
    if (!ContainsNodeId(scanned_joint_ids, node_id)) {
      std::cerr << "未扫描到请求的关节 ID=" << static_cast<int>(node_id)
                << std::endl;
      bus.Close();
      return -1;
    }
    if (!ContainsNodeId(joint_ids, node_id)) {
      joint_ids.push_back(node_id);
    }
  }
  if (joint_ids.empty()) {
    joint_ids.push_back(scanned_joint_ids.front());
    std::cout << "未传入关节号，默认使用扫描到的第一个关节: "
              << static_cast<int>(joint_ids.front()) << std::endl;
  }

  std::cout << "扫描到 " << scanned_joint_ids.size() << " 个关节, ID: ";
  for (const auto id : scanned_joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  std::cout << "本次参与 PP 测试的关节: ";
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
