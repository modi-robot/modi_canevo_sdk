/**
 * @file example_tool_io.cpp
 * @brief 读取和测试 CanEvo 工具 IO 8P 接口。
 *
 * 默认只读工具板状态，不会改变 DO 输出。
 *
 * 示例：
 *   ./example_tool_io --can can0 --node 46
 *   ./example_tool_io --can can0 --node 46 --count 20 --interval-ms 500
 *   ./example_tool_io --can can0 --node 46 --do-index 0 --do-value 1
 *   ./example_tool_io --can can0 --scan
 *
 * 8P 口的硬件功能来自工具 IO 协议和工具板原理图：
 *   - DI0/DI1：24V 数字输入
 *   - DO0/DO1：24V 数字输出
 *   - AI0/AI1：0~10V 模拟输入
 *   - 另外两根为电源正负
 */

#include <chrono>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "modi_tool_io_canevo.h"

namespace {

void PrintUsage(const char* program) {
  std::cout
      << "用法:\n"
      << "  " << program
      << " --can <can接口> --node <节点ID> [选项]\n\n"
      << "选项:\n"
      << "  --count <次数>       读取次数，默认 1；0 表示持续读取\n"
      << "  --interval-ms <毫秒> 持续读取时的间隔，默认 500\n"
      << "  --do-index <0|1>     要设置的 DO 通道，0=DO0，1=DO1\n"
      << "  --do-value <0|1>     DO 输出状态，0=关闭，1=打开\n"
      << "  --scan               扫描所有在线节点，并判断哪个节点支持工具 IO\n"
      << "  --help               显示帮助\n\n"
      << "安全提示：不同时传 --do-index 和 --do-value 时程序只读，不会改变 DO 输出。\n";
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

const char* BoolText(const bool value) { return value ? "1" : "0"; }

void PrintToolIo(const float supply_voltage, const float analog_input_1,
                 const float analog_input_2, const bool digital_input_1,
                 const bool digital_input_2, const bool digital_output_1,
                 const bool digital_output_2, const uint8_t pod_state) {
  std::cout << std::fixed << std::setprecision(3)
            << "supply=" << supply_voltage << " V"
            << "  AI0=" << analog_input_1 << " V"
            << "  AI1=" << analog_input_2 << " V"
            << "  DI0=" << BoolText(digital_input_1)
            << "  DI1=" << BoolText(digital_input_2)
            << "  DO0=" << BoolText(digital_output_1)
            << "  DO1=" << BoolText(digital_output_2)
            << "  POD=" << static_cast<int>(pod_state) << '\n';
}

void PrintRetWithAbort(const char* prefix, modi_tool_io_canevo& node, int ret) {
  std::cerr << prefix << "，ret=" << ret;
  if (ret == static_cast<int>(CanEvoError::kSdoAbort)) {
    std::cerr << ", abort=0x" << std::hex << node.NrtGetLastSdoAbortCode()
              << std::dec;
  }
  std::cerr << '\n';
}

bool ReadToolIoWithTrace(modi_tool_io_canevo& node, float& supply_voltage,
                         float& analog_input_1, float& analog_input_2,
                         bool& digital_input_1, bool& digital_input_2,
                         bool& digital_output_1, bool& digital_output_2,
                         uint8_t& pod_state) {
  int ret = node.RtGetToolSupplyVoltage(supply_voltage);
  if (ret != 0) {
    PrintRetWithAbort("读取工具板供电电压失败 (0xE0/0x00)", node, ret);
    return false;
  }
  ret = node.RtGetToolAnalogInput(0, analog_input_1);
  if (ret != 0) {
    PrintRetWithAbort("读取 AI0 失败 (0xE0/0x03 index=0)", node, ret);
    return false;
  }
  ret = node.RtGetToolAnalogInput(1, analog_input_2);
  if (ret != 0) {
    PrintRetWithAbort("读取 AI1 失败 (0xE0/0x04 index=1)", node, ret);
    return false;
  }
  ret = node.RtGetToolDigitalInput(0, digital_input_1);
  if (ret != 0) {
    PrintRetWithAbort("读取 DI0 失败 (0xE0/0x05 index=0)", node, ret);
    return false;
  }
  ret = node.RtGetToolDigitalInput(1, digital_input_2);
  if (ret != 0) {
    PrintRetWithAbort("读取 DI1 失败 (0xE0/0x05 index=1)", node, ret);
    return false;
  }
  ret = node.RtGetToolDigitalOutput(0, digital_output_1);
  if (ret != 0) {
    PrintRetWithAbort("读取 DO0 失败 (0xE0/0x06 index=0)", node, ret);
    return false;
  }
  ret = node.RtGetToolDigitalOutput(1, digital_output_2);
  if (ret != 0) {
    PrintRetWithAbort("读取 DO1 失败 (0xE0/0x06 index=1)", node, ret);
    return false;
  }
  ret = node.RtGetToolPodState(pod_state);
  if (ret != 0) {
    PrintRetWithAbort("读取 POD 失败 (0xE0/0x07)", node, ret);
    return false;
  }
  return true;
}

void WarmupToolIoPdo(modi_bus_canevo& bus) {
  for (int i = 0; i < 3; ++i) {
    (void)bus.RtStepOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

int ScanToolIoNodes(modi_bus_canevo& bus) {
  const auto ids = bus.NrtScanJoints();
  if (ids.empty()) {
    std::cerr << "未扫描到 CanEvo 节点。\n";
    return 1;
  }

  std::cout << "扫描到节点: ";
  for (const auto id : ids) {
    std::cout << static_cast<int>(id) << ' ';
  }
  std::cout << "\n\n";

  for (const auto id : ids) {
    modi_tool_io_canevo node;
    const int init_ret = node.NrtInit(bus, id);
    if (init_ret != 0) {
      std::cout << "node " << static_cast<int>(id)
                << ": 初始化失败 ret=" << init_ret << '\n';
      continue;
    }

    const std::string protocol = node.NrtGetProtocolVersion();
    const uint32_t module_type = node.NrtGetModuleType();
    const uint32_t vendor = node.NrtGetVendorCode();

    WarmupToolIoPdo(bus);

    float supply = 0.0f;
    const int tool_ret = node.RtGetToolSupplyVoltage(supply);

    std::cout << "node " << static_cast<int>(id)
              << ": protocol=" << protocol
              << " module=0x" << module_type << " vendor=0x" << vendor
              << std::dec;
    if (tool_ret == 0) {
      std::cout << "  支持工具IO  supply=" << std::fixed
                << std::setprecision(3) << supply << " V";
    } else {
      std::cout << "  不支持/未响应工具IO  ret=" << tool_ret;
      if (tool_ret == static_cast<int>(CanEvoError::kSdoAbort)) {
        std::cout << " abort=0x" << std::hex
                  << node.NrtGetLastSdoAbortCode() << std::dec;
      }
    }
    std::cout << '\n';

    node.NrtDestroy();
  }

  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  std::string can_interface;
  unsigned long node_id_value = 0;
  unsigned long count_value = 1;
  unsigned long interval_value = 500;
  std::optional<uint8_t> do_index;
  std::optional<bool> do_value;
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
        PrintUsage(argv[0]);
        return 2;
      }
      continue;
    }
    if (option == "--node" || option == "--count" ||
        option == "--interval-ms" || option == "--do-index" ||
        option == "--do-value") {
      if (!ReadOptionValue(i, argc, argv, value) ||
          !ReadUnsigned(value, parsed)) {
        std::cerr << "参数无效: " << option << '\n';
        return 2;
      }
      if (option == "--node") {
        node_id_value = parsed;
      } else if (option == "--count") {
        count_value = parsed;
      } else if (option == "--interval-ms") {
        interval_value = parsed;
      } else if (option == "--do-index") {
        if (parsed > 1) {
          std::cerr << "--do-index 只能使用 0 或 1，当前值为 " << parsed << '\n';
          return 2;
        }
        do_index = static_cast<uint8_t>(parsed);
      } else {
        if (parsed > 1) {
          std::cerr << "--do-value 只能使用 0 或 1，当前值为 " << parsed << '\n';
          return 2;
        }
        do_value = parsed != 0;
      }
      continue;
    }

    std::cerr << "未知参数: " << option << '\n';
    PrintUsage(argv[0]);
    return 2;
  }

  if (can_interface.empty() || (!scan && node_id_value == 0) ||
      node_id_value > 62 || interval_value > 3600000UL) {
    std::cerr << "--can 必须提供；非 --scan 模式还必须提供 --node，node 范围为 1~62。\n";
    PrintUsage(argv[0]);
    return 2;
  }

  if (do_index.has_value() != do_value.has_value()) {
    std::cerr << "--do-index 和 --do-value 必须同时提供。\n";
    PrintUsage(argv[0]);
    return 2;
  }
  if (scan && do_index.has_value()) {
    std::cerr << "--scan 不能和 --do-index/--do-value 同时使用。\n";
    PrintUsage(argv[0]);
    return 2;
  }

  modi_bus_canevo bus;
  bus.SetSdoTimeoutMs(200);
  TaskConfig config;
  if (bus.Open(can_interface, config) != 0) {
    std::cerr << "打开 CAN 接口失败: " << can_interface << '\n';
    return 1;
  }

  if (scan) {
    const int scan_ret = ScanToolIoNodes(bus);
    bus.Close();
    return scan_ret;
  }

  modi_tool_io_canevo node;
  const auto node_id = static_cast<uint8_t>(node_id_value);
  if (node.NrtInit(bus, node_id) != 0) {
    std::cerr << "初始化 CanEvo 节点失败，node_id="
              << static_cast<int>(node_id) << '\n';
    bus.Close();
    return 1;
  }

  WarmupToolIoPdo(bus);

  if (do_index.has_value()) {
    const int ret = node.RtSetToolDigitalOutput(*do_index, *do_value);
    if (ret != 0) {
      PrintRetWithAbort("设置 DO 失败 (0xE0/0x06)", node, ret);
      node.NrtDestroy();
      bus.Close();
      return 1;
    }
    std::cout << "已设置 DO" << static_cast<int>(*do_index)
              << "=" << BoolText(*do_value) << '\n';
  }

  std::cout << "开始读取工具 IO: can=" << can_interface
            << ", node_id=" << static_cast<int>(node_id)
            << ", count=" << count_value << '\n';

  unsigned long count = 0;
  while (count_value == 0 || count < count_value) {
    const int sync_ret = bus.RtStepOnce();
    if (sync_ret != 0) {
      std::cerr << "发送 SYNC 失败，ret=" << sync_ret << '\n';
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(2));

    float supply_voltage = 0.0f;
    float analog_input_1 = 0.0f;
    float analog_input_2 = 0.0f;
    bool digital_input_1 = false;
    bool digital_input_2 = false;
    bool digital_output_1 = false;
    bool digital_output_2 = false;
    uint8_t pod_state = 0;
    if (ReadToolIoWithTrace(node, supply_voltage, analog_input_1,
                            analog_input_2, digital_input_1, digital_input_2,
                            digital_output_1, digital_output_2,
                            pod_state)) {
      std::cout << '[' << count + 1 << "] ";
      PrintToolIo(supply_voltage, analog_input_1, analog_input_2,
                  digital_input_1, digital_input_2, digital_output_1,
                  digital_output_2, pod_state);
    }
    ++count;

    if (count_value != 0 && count >= count_value) break;
    std::this_thread::sleep_for(
        std::chrono::milliseconds(interval_value));
  }

  node.NrtDestroy();
  bus.Close();
  return 0;
}
