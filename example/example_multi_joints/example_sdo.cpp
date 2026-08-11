/**
 * @file example_sdo.cpp
 * @brief 最简示例: 打开/关闭CAN总线，初始化/销毁关节
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 */

#include <iostream>
#include <vector>

#include "modi_joint_canevo.h"

int main() {
  std::cout << "\n========== 示例 01: 总线打开/关闭 ==========\n" << std::endl;

  // 1. 创建总线对象
  modi_bus_canevo bus;

  // 2. 打开已手动配置好的 CAN 总线
  std::cout << "正在打开 CAN 总线 can0 ... " << std::flush;
  TaskConfig cfg;
  cfg.sync_period_us = 5000;
  cfg.cpu_affinity = 2;
  if (bus.Open("can0", cfg) != 0) {
    std::cerr << "打开 CAN 总线 can0 失败" << std::endl;
    return -1;
  }
  std::cout << "打开 CAN 总线 can0 成功" << std::endl;

  std::cout << "正在扫描在线关节 ... " << std::flush;
  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "发现 " << joint_ids.size() << " 个关节: ";
  for (const auto id : joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  std::vector<modi_joint_canevo> joints(joint_ids.size());
  for (size_t i = 0; i < joint_ids.size(); ++i) {
    const uint8_t node_id = joint_ids[i];
    std::cout << "正在初始化关节 (ID=" << static_cast<int>(node_id)
              << ") ... " << std::flush;
    if (joints[i].NrtInit(bus, node_id) != 0) {
      std::cerr << "初始化关节 (ID=" << static_cast<int>(node_id) << ") 失败"
                << std::endl;
      for (size_t j = 0; j < i; ++j) {
        joints[j].NrtDestroy();
      }
      bus.Close();
      return -1;
    }
    std::cout << "初始化关节 (ID=" << static_cast<int>(node_id) << ") 成功"
              << std::endl;
  }

  // 4. 清理资源
  std::cout << "正在清理资源 ... " << std::flush;
  for (auto& joint : joints) {
    joint.NrtDestroy();
  }
  bus.Close();

  std::cout << "\n========== 示例执行完成 ==========\n" << std::endl;
  return 0;
}
