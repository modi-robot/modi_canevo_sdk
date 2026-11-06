/**
 * @brief 通用 RS485 透传示例：只发送原始字节并打印原始回包，不解析设备协议。
 * ./example_tool_rs485 --can can0 --node 40 --send "EB 90 01 01 41 43"
 * 不带 --send 时只监听回包；原始发送数据由调用者按设备手册提供。
 */
#include <array>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "modi_tool_io_canevo.h"

namespace {
volatile std::sig_atomic_t running = 1;
void Stop(int) { running = 0; }
}

int main(int argc, char** argv) {
  std::string can = "can0";
  unsigned node = 40;
  unsigned timeout_ms = 1000;
  std::string bytes;
  try {
    for (int i = 1; i < argc; ++i) {
      const std::string key = argv[i];
      if (key == "--help") {
        std::cout << "用法: example_tool_rs485 [--can can0] [--node 40] [--send \"十六进制字节\"] [--timeout-ms 1000]\n";
        return 0;
      }
      if (++i >= argc) return 2;
      if (key == "--can") can = argv[i];
      else if (key == "--send") bytes = argv[i];
      else if (key == "--node" || key == "--timeout-ms") {
        std::size_t used = 0;
        const std::string value = argv[i];
        const auto number = std::stoul(value, &used);
        if (used != value.size() || number > 60000) return 2;
        if (key == "--node") node = static_cast<unsigned>(number);
        else timeout_ms = static_cast<unsigned>(number);
      } else return 2;
    }
  } catch (const std::exception&) { return 2; }
  if (node < 1 || node > 62 || timeout_ms == 0 || can.empty()) return 2;
  std::array<uint8_t, 60> request{};
  std::size_t request_size = 0;
  std::istringstream input(bytes);
  std::string token;
  while (input >> token) {
    try {
      std::size_t used = 0;
      const auto value = std::stoul(token, &used, 16);
      if (used != token.size() || value > 255 || request_size == request.size()) return 2;
      request[request_size++] = static_cast<uint8_t>(value);
    } catch (const std::exception&) { return 2; }
  }
  if (!bytes.empty() && request_size == 0) return 2;
  std::signal(SIGINT, Stop);
  std::signal(SIGTERM, Stop);
  modi_bus_canevo bus;
  int ret = bus.Open(can);
  if (ret != 0) { std::cerr << "打开总线失败，ret=" << ret << '\n'; return 1; }
  modi_tool_io_canevo tool;
  ret = tool.NrtInit(bus, static_cast<uint8_t>(node));
  if (ret != 0) { std::cerr << "初始化失败，ret=" << ret << '\n'; return 1; }
  if (request_size != 0) {
    ret = tool.RtSendToolRs485(request.data(), request_size);
    if (ret != 0) { std::cerr << "发送失败，ret=" << ret << '\n'; return 1; }
    std::cout << "原始数据已发送，字节数=" << request_size << "；尚未确认设备执行结果。\n";
  }
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
  uint32_t sequence = 0;
  bool received = false;
  while (running && std::chrono::steady_clock::now() < deadline) {
    std::array<uint8_t, 60> response{};
    std::size_t response_size = 0;
    uint32_t response_sequence = 0;
    ret = tool.RtReceiveToolRs485(response.data(), response.size(), response_size, &response_sequence);
    if (ret == 0 && response_sequence != sequence) {
      sequence = response_sequence;
      received = true;
      std::cout << "sequence=" << sequence << ", size=" << response_size << ", data=";
      for (std::size_t i = 0; i < response_size; ++i) {
        std::cout << std::hex << std::uppercase << std::setfill('0') << std::setw(2)
                  << static_cast<unsigned>(response[i]) << ' ';
      }
      std::cout << std::dec << '\n';
    } else if (ret != 0 && ret != static_cast<int>(CanEvoError::kPdoTimeout)) {
      std::cerr << "读取失败，ret=" << ret << '\n'; return 1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  if (!received) std::cerr << "等待超时：没有收到完整的外层透传帧。\n";
  return received ? 0 : 1;
}
