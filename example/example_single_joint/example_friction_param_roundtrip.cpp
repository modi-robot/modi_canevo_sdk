#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

#include "modi_joint_canevo.h"

namespace {
constexpr std::size_t kFrictionParamCount = 7;
constexpr float kMaxFrictionParam = 65.535f;
constexpr float kScale = 1000.0f;

void PrintParams(const char* label, const std::vector<float>& params) {
  std::cout << label << ":\n" << std::fixed << std::setprecision(3);
  for (std::size_t i = 0; i < params.size(); ++i)
    std::cout << "  p" << i << " = " << params[i] << '\n';
}

bool ParseParam(const char* text, float& value) {
  char* end = nullptr;
  const float parsed = std::strtof(text, &end);
  if (end == text || *end != '\0' || !std::isfinite(parsed) ||
      parsed < 0.0f || parsed > kMaxFrictionParam)
    return false;
  value = parsed;
  return true;
}

long WireValue(float value) {
  return std::lround(value * kScale);
}
}  // namespace

int main(int argc, char* argv[]) {
  if (argc != static_cast<int>(kFrictionParamCount) + 3) {
    std::cerr << "用法: " << argv[0]
              << " <CAN接口名称> <关节Node ID> <p0> <p1> <p2> <p3> <p4> <p5> <p6>\n"
              << "示例: " << argv[0]
              << " can0 1 1.234 0.100 0.200 0.300 0.400 0.000 0.000\n"
              << "警告: 此程序会覆盖指定关节的全部7个摩擦参数，"
                 "并写入 flash 保存（掉电保持）。\n"
              << "参数范围为[0, 65.535]，固件量化步进为0.001。"
              << std::endl;
    return 2;
  }

  const std::string can_interface = argv[1];
  char* node_end = nullptr;
  const long node_value = std::strtol(argv[2], &node_end, 10);
  if (node_end == argv[2] || *node_end != '\0' || node_value < 1 ||
      node_value > std::numeric_limits<uint8_t>::max()) {
    std::cerr << "关节 Node ID 必须是 1~255 的整数。\n";
    return 2;
  }
  const auto node_id = static_cast<uint8_t>(node_value);

  std::vector<float> requested(kFrictionParamCount);
  for (std::size_t i = 0; i < requested.size(); ++i) {
    if (!ParseParam(argv[i + 3], requested[i])) {
      std::cerr << "参数 p" << i
                << " 无效，必须是范围[0, 65.535]内的有限数值。\n";
      return 2;
    }
  }

  modi_bus_canevo bus;
  TaskConfig config;
  config.sync_period_us = 5000;
  if (bus.Open(can_interface, config) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 总线 " << can_interface << " 失败。\n";
    return 1;
  }

  const auto node_ids = bus.NrtScanJoints();
  bool found = false;
  for (const auto id : node_ids) found = found || id == node_id;
  if (!found) {
    std::cerr << "未扫描到关节 Node ID " << node_value << "。\n";
    bus.Close();
    return 1;
  }

  modi_joint_canevo joint;
  int ret = joint.NrtInit(bus, node_id);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化关节 Node ID " << node_value << " 失败，错误码="
              << ret << '\n';
    bus.Close();
    return 1;
  }

  // Read -> write -> read. Keep the original values visible for review.
  std::vector<float> before(kFrictionParamCount);
  for (std::size_t i = 0; i < kFrictionParamCount; ++i)
    before[i] = joint.NrtGetFrictionParam(static_cast<uint8_t>(i));
  PrintParams("写入前参数", before);
  PrintParams("请求写入参数", requested);

  for (std::size_t i = 0; i < kFrictionParamCount; ++i) {
    ret = joint.NrtSetFrictionParam(static_cast<uint8_t>(i), requested[i]);
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "写入 p" << i << " 失败，错误码=" << ret
                << "。SDK 按 index 顺序写入，失败时已写入项不会回滚。\n";
      joint.NrtDestroy();
      bus.Close();
      return 1;
    }
  }

  // 写入的只是关节 RAM，掉电即失；触发 flash 保存并轮询保存成功标志。
  ret = joint.NrtSaveParams();
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "触发参数保存失败，错误码=" << ret << "。\n";
    joint.NrtDestroy();
    bus.Close();
    return 1;
  }
  bool saved = false;
  const auto save_deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < save_deadline) {
    if (joint.NrtGetSaveParamsOk() == 1) {
      saved = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!saved) {
    std::cerr << "3 秒内未收到参数保存成功标志，参数重启后可能丢失。\n";
    joint.NrtDestroy();
    bus.Close();
    return 1;
  }
  std::cout << "参数已保存到 flash（掉电保持）。\n";

  std::vector<float> after(kFrictionParamCount);
  for (std::size_t i = 0; i < kFrictionParamCount; ++i)
    after[i] = joint.NrtGetFrictionParam(static_cast<uint8_t>(i));
  PrintParams("写入后回读参数", after);

  bool matched = true;
  for (std::size_t i = 0; i < requested.size(); ++i) {
    const bool equal = WireValue(requested[i]) == WireValue(after[i]);
    matched = matched && equal;
    if (!equal)
      std::cerr << "p" << i << " 不一致：请求量化值="
                << static_cast<float>(WireValue(requested[i])) / kScale
                << "，回读值=" << after[i] << '\n';
  }

  joint.NrtDestroy();
  bus.Close();
  if (!matched) {
    std::cerr << "摩擦参数读-写-读校验失败。\n";
    return 1;
  }
  std::cout << "摩擦参数读-写-读校验通过。\n";
  return 0;
}
