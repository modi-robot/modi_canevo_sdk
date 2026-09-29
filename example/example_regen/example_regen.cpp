/**
 * @file example_regen.cpp
 * @brief 读取和测试 CanEvo 再生制动板 PDO 快照接口。
 *
 * 用法：
 *   ./example_regen --can can0 --node 46
 *   ./example_regen --can can0 --node 46 --count 20 --interval-ms 500
 *   ./example_regen --can can0 --scan
 */

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

#include "modi_bus_canevo.h"
#include "modi_regen_canevo.h"

namespace {

void PrintUsage(const char* program) {
  std::cout
      << "用法:\n"
      << "  " << program
      << " --can <can接口> --node <节点ID> [选项]\n\n"
      << "选项:\n"
      << "  --count <次数>       读取次数，默认 1；0 表示持续读取\n"
      << "  --interval-ms <毫秒> 持续读取时的间隔，默认 500\n"
      << "  --scan               扫描所有在线节点\n"
      << "  --help               显示帮助\n";
}

bool ReadUnsigned(const std::string& text, unsigned long& value) {
  try {
    std::size_t parsed = 0;
    value = std::stoul(text, &parsed, 0);
    return parsed == text.size();
  } catch (const std::exception&) {
    return false;
  }
}

bool ReadOptionValue(int& index, int argc, char** argv, std::string& value) {
  if (index + 1 >= argc) return false;
  value = argv[++index];
  return true;
}

void PrintRegenStatus(const RegenStatus& st) {
  std::cout << std::fixed << std::setprecision(3)
            << "sw=0x" << std::hex << st.statusword << std::dec
            << " vin=" << st.input_voltage_v
            << " vout=" << st.output_voltage_v
            << " cur=" << st.system_current_a
            << " temp=" << st.brake_temp_c
            << " ax=" << st.mpu_accel_x_g
            << " ay=" << st.mpu_accel_y_g
            << " az=" << st.mpu_accel_z_g << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  std::string can_interface;
  std::optional<uint8_t> node_id;
  unsigned long count_value = 1;
  unsigned long interval_value = 500;
  bool scan = false;

  for (int i = 1; i < argc; ++i) {
    const std::string option = argv[i];
    std::string value;
    unsigned long parsed = 0;

    if (option == "--help") {
      PrintUsage(argv[0]);
      return 0;
    }
    if (option == "--scan") {
      scan = true;
      continue;
    }
    if (option == "--can") {
      if (!ReadOptionValue(i, argc, argv, can_interface)) {
        std::cerr << "--can 需要接口名\n";
        return 1;
      }
      continue;
    }
    if (option == "--node") {
      if (!ReadOptionValue(i, argc, argv, value) || !ReadUnsigned(value, parsed) ||
          parsed > 62) {
        std::cerr << "--node 需要 1~62 的节点 ID\n";
        return 1;
      }
      node_id = static_cast<uint8_t>(parsed);
      continue;
    }
    if (option == "--count") {
      if (!ReadOptionValue(i, argc, argv, value) || !ReadUnsigned(value, parsed)) {
        std::cerr << "--count 参数非法\n";
        return 1;
      }
      count_value = parsed;
      continue;
    }
    if (option == "--interval-ms") {
      if (!ReadOptionValue(i, argc, argv, value) || !ReadUnsigned(value, parsed)) {
        std::cerr << "--interval-ms 参数非法\n";
        return 1;
      }
      interval_value = parsed;
      continue;
    }

    std::cerr << "未知参数: " << option << '\n';
    PrintUsage(argv[0]);
    return 1;
  }

  if (can_interface.empty()) {
    std::cerr << "缺少 --can 参数\n";
    PrintUsage(argv[0]);
    return 1;
  }

  modi_bus_canevo bus;
  if (bus.Open(can_interface) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开总线失败: " << can_interface << '\n';
    return 1;
  }

  if (scan) {
    const auto ids = bus.NrtScanJoints();
    if (ids.empty()) {
      std::cout << "未扫描到节点。\n";
    } else {
      std::cout << "扫描到节点: ";
      for (const auto id : ids) {
        std::cout << static_cast<int>(id) << ' ';
      }
      std::cout << '\n';
    }
    bus.Close();
    return 0;
  }

  if (!node_id.has_value()) {
    std::cerr << "缺少 --node 参数\n";
    PrintUsage(argv[0]);
    bus.Close();
    return 1;
  }

  modi_regen_canevo regen;
  const int init_ret = regen.NrtInit(bus, *node_id);
  if (init_ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化再生板失败 ret=" << init_ret << '\n';
    bus.Close();
    return 1;
  }

  for (int i = 0; i < 3; ++i) {
    (void)bus.RtStepOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::size_t loop = 0;
  do {
    (void)bus.RtStepOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    RegenStatus st;
    const int ret = regen.RtGetRegenStatus(st);
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "读取快照失败 ret=" << ret << '\n';
      break;
    }

    std::cout << "[" << loop << "] ";
    PrintRegenStatus(st);

    ++loop;
    if (count_value != 0 && loop >= count_value) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(interval_value));
  } while (true);

  regen.NrtDestroy();
  bus.Close();
  return 0;
}
