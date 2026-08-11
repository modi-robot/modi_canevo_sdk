/**
 * @file example_nrtpp.cpp
 * @brief PP 位置模式测试 - 通过 NRT/SDO 接口在 0/+30/-30 deg 间运动
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <thread>

#include "modi_joint_canevo.h"

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kTargetToleranceRad = 0.2f * kDegToRad;
constexpr float kProfileVelRadS = 0.3f;
constexpr float kProfileAccRadSS = 0.5f;
constexpr float kPpPositiveTargetRad = 30.0f * kDegToRad;
constexpr float kPpNegativeTargetRad = -30.0f * kDegToRad;
constexpr int kMaxRoundTrips = 5;
constexpr auto kStateTimeout = std::chrono::seconds(3);

enum class RunState {
  kRunning,
  kStopRequested,
};

enum class PpTargetState {
  kMoveZero,
  kMovePositive,
  kMoveNegative,
};

static std::atomic<RunState> g_run_state{RunState::kRunning};

void SignalHandler(int signum) {
  (void)signum;
  g_run_state.store(RunState::kStopRequested, std::memory_order_release);
}

void PrintJointDiag(modi_joint_canevo& joint) {
  const auto state = joint.NrtGetServoState();
  const auto mode = joint.NrtGetCurrentMode();
  const auto fault = joint.NrtGetFaultCode();
  const auto warn = joint.NrtGetWarnCode();

  std::cerr << "诊断: servo_state=" << static_cast<int>(state)
            << ", mode=" << static_cast<int>(mode) << ", fault=0x" << std::hex
            << static_cast<uint16_t>(fault) << ", warn=0x"
            << static_cast<uint16_t>(warn) << std::dec << std::endl;
}

void PrintEndTime() {
  const auto now = std::chrono::system_clock::now();
  const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
  std::tm local_time{};
  localtime_r(&now_time, &local_time);
  std::cout << "结束时间: " << std::put_time(&local_time, "%Y-%m-%d %H:%M:%S")
            << std::endl;
}

bool WaitServoState(modi_joint_canevo& joint, CanEvoServoState target,
                    const char* action) {
  const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (joint.NrtGetServoState() == target) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << action << "超时" << std::endl;
  PrintJointDiag(joint);
  return false;
}

bool WaitControlMode(modi_joint_canevo& joint, CanEvoMode target,
                     const char* action) {
  const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
  CanEvoMode current = joint.NrtGetCurrentMode();
  while (std::chrono::steady_clock::now() < deadline) {
    current = joint.NrtGetCurrentMode();
    if (current == target) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << action << "超时, 当前模式=" << static_cast<int>(current)
            << ", 目标模式=" << static_cast<int>(target) << std::endl;
  PrintJointDiag(joint);
  return false;
}

int SendPpTarget(modi_joint_canevo& joint, float target_pos_rad) {
  return joint.NrtSetPpTargetPosition(target_pos_rad, kProfileVelRadS,
                                      kProfileAccRadSS);
}

int main(int argc, char* argv[]) {
  std::cout << "========================================" << std::endl;
  std::cout << "PP NRT 测试程序 - 通过 SDO 在 0/+30/-30 deg 间运动"
            << std::endl;
  std::cout << "========================================" << std::endl;

  modi_bus_canevo bus;
  if (bus.Open("can0") != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 总线 can0 失败" << std::endl;
    return -1;
  }
  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

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

  uint8_t node_id = joint_ids.front();
  if (argc >= 2) {
    const int requested_id = std::atoi(argv[1]);
    if (requested_id < 1 || requested_id > 127) {
      std::cerr << "无效关节 ID: " << argv[1] << std::endl;
      bus.Close();
      return -1;
    }

    const auto it = std::find(joint_ids.begin(), joint_ids.end(),
                              static_cast<uint8_t>(requested_id));
    if (it == joint_ids.end()) {
      std::cerr << "指定关节不在线, ID=" << requested_id << std::endl;
      bus.Close();
      return -1;
    }

    node_id = static_cast<uint8_t>(requested_id);
    std::cout << "指定选择关节 ID: " << requested_id << std::endl;
  } else {
    std::cout << "默认选择第一个关节 ID: " << static_cast<int>(node_id)
              << std::endl;
  }

  modi_joint_canevo joint;
  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化关节失败, ID=" << static_cast<int>(node_id)
              << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "Joint 初始化完成 (Node ID: " << static_cast<int>(node_id)
            << ")" << std::endl;

  const auto fault_code = joint.NrtGetFaultCode();
  std::cerr << "fault_code: 0x" << std::hex << static_cast<uint16_t>(fault_code)
            << std::dec << std::endl;
  if (fault_code != CanEvoFault::kNone) {
    std::cerr << "检测到故障，先清除故障..." << std::endl;
    PrintJointDiag(joint);
    const int clear_ret = joint.NrtClearFault();
    if (clear_ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "清除故障失败, ret=" << clear_ret << std::endl;
      joint.NrtDestroy();
      bus.Close();
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (joint.NrtEnable(CanEvoMode::kPp) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "使能 PP 失败, ID=" << static_cast<int>(node_id)
              << std::endl;
    PrintJointDiag(joint);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitServoState(joint, CanEvoServoState::kRunning,
                      "等待 PP Running 状态")) {
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitControlMode(joint, CanEvoMode::kPp, "等待 PP 模式切换")) {
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  PpTargetState target_state = PpTargetState::kMoveZero;
  float target_pos_rad = 0.0f;
  int round_trips = 0;

  int ret = SendPpTarget(joint, target_pos_rad);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "下发 PP 初始目标失败, ret=" << ret << std::endl;
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  std::cout << "PP NRT 位置轨迹运行中：0/+30/-30 deg，按 Ctrl+C 终止..."
            << std::endl;

  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning) {
    if (joint.NrtGetServoState() == CanEvoServoState::kFault) {
      std::cerr << "检测到关节故障" << std::endl;
      PrintJointDiag(joint);
      ret = -1;
      break;
    }

    const float actual_pos = joint.NrtGetActualPosition();
    if (std::fabs(target_pos_rad - actual_pos) > kTargetToleranceRad) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
      continue;
    }

    if (target_state == PpTargetState::kMoveZero) {
      target_state = PpTargetState::kMovePositive;
      target_pos_rad = kPpPositiveTargetRad;
      std::cout << "切换目标: +30 deg" << std::endl;
    } else if (target_state == PpTargetState::kMovePositive) {
      target_state = PpTargetState::kMoveNegative;
      target_pos_rad = kPpNegativeTargetRad;
      std::cout << "切换目标: -30 deg" << std::endl;
    } else {
      ++round_trips;
      if (round_trips >= kMaxRoundTrips) {
        g_run_state.store(RunState::kStopRequested, std::memory_order_release);
        break;
      }

      target_state = PpTargetState::kMovePositive;
      target_pos_rad = kPpPositiveTargetRad;
      std::cout << "第 " << (round_trips + 1) << " 次切换目标: +30 deg"
                << std::endl;
    }

    ret = SendPpTarget(joint, target_pos_rad);
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "下发 PP 目标失败, ret=" << ret << std::endl;
      break;
    }
  }

  std::cout << "PP NRT 轨迹结束或收到退出信号，准备失能关节..." << std::endl;
  const int disable_ret = joint.NrtDisable();
  if (disable_ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "失能失败, ret=" << disable_ret << std::endl;
    PrintJointDiag(joint);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  PrintEndTime();
  joint.NrtDestroy();
  bus.Close();
  return ret == static_cast<int>(CanEvoError::kOk) ? 0 : -1;
}
