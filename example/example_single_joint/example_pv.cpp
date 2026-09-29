/**
 * @file example_pv.cpp
 * @brief 轮毂 PV 单次运动测试 - 转向关节先以 PP 模式回零
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 txqueuelen 1
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 *
 * 用法：
 *   example_single_joint_pv --degrees 90                 # 默认轮毂 ID 2
 *   example_single_joint_pv --degrees -90 2              # 选择一个 ID
 *   example_single_joint_pv --degrees 180 2 4 6 8        # 选择多个 ID
 *   example_single_joint_pv --degrees 180 2,4,6,8        # 逗号分隔
 *   example_single_joint_pv --degrees 360 --all          # 全部在线轮毂
 *
 * 程序先将转向关节 1/3/5/7 以 PP 模式使能并回到绝对零位，确认全部
 * 到位后，再让所选轮毂 2/4/6/8 从当前实际位置开始，以 11.0 rad/s
 * 按命令行指定的相对角度只运动一次。
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "modi_joint_canevo.h"

#ifndef CANEVO_HAVE_NECRO
#define CANEVO_HAVE_NECRO 0
#endif

#if CANEVO_HAVE_NECRO
#include <qiuniu/init.h>
#include <qiuniu/wrappers.h>
#else
#ifndef __RT
#define __RT(expr) (expr)
#endif
inline void qiuniu_init() {}
#endif

namespace {

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRpmToRadS = 2.0f * kPi / 60.0f;
constexpr float kPvSpeedRadS = 6.0f;
constexpr std::array<uint8_t, 4> kSteeringIds{{1, 3, 5, 7}};
constexpr std::array<uint8_t, 4> kWheelIds{{2, 4, 6, 8}};
constexpr std::array<uint8_t, 2> kReversedWheelIds{{4, 8}};
constexpr float kSteeringHomePositionToleranceRad = 0.2f * kDegToRad;
constexpr float kSteeringHomeVelocityToleranceRadS = 0.02f;
constexpr float kSteeringProfileVelocityRadS = 0.3f;
constexpr float kSteeringProfileAccRadSS = 0.5f;
constexpr int kSteeringHomeStableCycles = 10;
constexpr int kRtStateStartupGraceCycles = 20;
constexpr int kRtStatusReadGraceCycles = 20;
// CanEvo V1.2.4 specifies a 1~300 rpm/s range for PV profile acceleration
// and deceleration. Use the maximum for a near-step PV command. For a true
// unprofiled velocity step, use CSV rather than PV.
constexpr float kPvProfileAccRadSS = 300.0f * kRpmToRadS;
constexpr float kPvProfileDecRadSS = 300.0f * kRpmToRadS;
constexpr float kStopVelocityToleranceRadS = 0.02f;
constexpr int kStopStableCycles = 10;
constexpr auto kStateTimeout = std::chrono::seconds(3);
constexpr auto kSteeringHomeTimeout = std::chrono::seconds(30);
constexpr auto kStopTimeout = std::chrono::seconds(3);
constexpr int kRtFault = -1000;
constexpr int kRtUnexpectedModeOrState = -1001;

modi_bus_canevo* g_bus = nullptr;
std::vector<modi_joint_canevo>* g_steering_joints = nullptr;
std::vector<modi_joint_canevo>* g_wheel_joints = nullptr;
std::vector<float> g_target_positions;
std::vector<float> g_move_directions;
std::vector<uint8_t> g_motion_phases;
std::vector<int> g_stopped_stable_cycles;
std::vector<uint8_t> g_steering_home_target_sent;
std::vector<int> g_steering_home_stable_cycles;
std::vector<uint8_t> g_steering_rt_state_seen;
std::vector<int> g_steering_rt_transient_cycles;
std::vector<uint8_t> g_wheel_rt_state_seen;
std::vector<int> g_wheel_rt_transient_cycles;
int g_period_us = 5000;
std::atomic<int> g_rt_ret{0};
std::atomic<int> g_rt_failed_joint_index{-1};
std::atomic<bool> g_steering_home_complete{false};
std::atomic<bool> g_wheel_motion_started{false};
std::atomic<bool> g_stop_complete{false};
std::atomic<bool> g_motion_complete{false};

enum class FailedJointGroup : uint8_t {
  kNone,
  kSteering,
  kWheel,
};

std::atomic<FailedJointGroup> g_rt_failed_joint_group{
    FailedJointGroup::kNone};

enum class RunState {
  kRunning,
  kStopRequested,
  kRtExit,
};

std::atomic<RunState> g_run_state{RunState::kRunning};

struct CommandLineOptions {
  bool select_all{false};
  bool has_travel_degrees{false};
  float travel_degrees{0.0f};
  std::vector<uint8_t> requested_ids;
};

enum class MotionPhase : uint8_t {
  kMoving,
  kStopping,
  kComplete,
};

float WheelDirectionSign(uint8_t id) {
  return std::find(kReversedWheelIds.begin(), kReversedWheelIds.end(), id) !=
                 kReversedWheelIds.end()
             ? -1.0f
             : 1.0f;
}

timespec AddNs(timespec current, long ns) {
  current.tv_nsec += ns;
  if (current.tv_nsec >= 1000000000L) {
    current.tv_sec += current.tv_nsec / 1000000000L;
    current.tv_nsec %= 1000000000L;
  } else if (current.tv_nsec < 0) {
    const long borrow = (-current.tv_nsec + 999999999L) / 1000000000L;
    current.tv_sec -= borrow;
    current.tv_nsec += borrow * 1000000000L;
  }
  return current;
}

void PrintUsage(const char* program) {
  std::cout
      << "用法:\n"
      << "  " << program << " --degrees DEG [ID ...]\n"
      << "  " << program << " --degrees DEG ID,ID,...\n"
      << "  " << program << " --degrees DEG --all\n\n"
      << "示例:\n"
      << "  " << program
      << " --degrees 90             # 默认轮毂 ID 2 正向 90°\n"
      << "  " << program
      << " --degrees -90 2          # ID 2 反向 90°\n"
      << "  " << program
      << " --degrees 180 2 4 6 8    # 多个 ID 各正向 180°\n"
      << "  " << program
      << " --degrees 180 2,4,6,8    # 逗号分隔的多 ID\n"
      << "  " << program
      << " --degrees 360 --all      # 全部在线轮毂正向 360°\n\n"
      << "可选轮毂 ID 只有 2/4/6/8；--all 表示全部在线轮毂。\n"
      << "转向关节 1/3/5/7 必须全部在线，程序会先将其 PP 使能并回零。\n"
      << "CAN 接口固定为 can0。\n"
      << "DEG 是相对运动角度，必须非零；正数正转，负数反转。\n"
      << "轮毂 4/8 的关节方向相对命令取反，轮毂 2/6 保持原方向。\n"
      << "PV 目标速度固定为 11.0 rad/s，加减速度为协议上限 300 rpm/s。\n"
      << "程序到达指定相对角度后停止并自动退出，不再往复。\n";
}

float ParseTravelDegrees(const std::string& text) {
  std::size_t parsed = 0;
  float degrees = 0.0f;
  try {
    degrees = std::stof(text, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument("无效运动角度: " + text);
  }
  if (parsed != text.size() || !std::isfinite(degrees) ||
      std::fabs(degrees) < 1e-6f) {
    throw std::invalid_argument("运动角度必须是非零有限数值: " + text);
  }
  return degrees;
}

void AppendRequestedId(const std::string& text,
                       std::vector<uint8_t>* requested_ids) {
  if (text.empty()) {
    throw std::invalid_argument("关节 ID 不能为空");
  }

  std::size_t parsed = 0;
  int requested_id = 0;
  try {
    requested_id = std::stoi(text, &parsed);
  } catch (const std::exception&) {
    throw std::invalid_argument("无效关节 ID: " + text);
  }
  if (parsed != text.size() || requested_id < 1 || requested_id > 62) {
    throw std::invalid_argument("无效关节 ID: " + text);
  }

  const auto id = static_cast<uint8_t>(requested_id);
  if (std::find(kWheelIds.begin(), kWheelIds.end(), id) == kWheelIds.end()) {
    throw std::invalid_argument("无效轮毂 ID: " + text +
                                "，只能选择 2/4/6/8；1/3/5/7 "
                                "由程序用于 PP 回零");
  }
  if (std::find(requested_ids->begin(), requested_ids->end(), id) !=
      requested_ids->end()) {
    throw std::invalid_argument("重复的关节 ID: " + text);
  }
  requested_ids->push_back(id);
}

CommandLineOptions ParseOptions(int argc, char* argv[]) {
  CommandLineOptions options;
  for (int arg_index = 1; arg_index < argc; ++arg_index) {
    const std::string argument(argv[arg_index]);
    if (argument == "--degrees") {
      if (options.has_travel_degrees) {
        throw std::invalid_argument("--degrees 只能指定一次");
      }
      if (arg_index + 1 >= argc) {
        throw std::invalid_argument("--degrees 缺少角度值");
      }
      options.travel_degrees = ParseTravelDegrees(argv[++arg_index]);
      options.has_travel_degrees = true;
    } else if (argument.rfind("--degrees=", 0) == 0) {
      if (options.has_travel_degrees) {
        throw std::invalid_argument("--degrees 只能指定一次");
      }
      options.travel_degrees = ParseTravelDegrees(argument.substr(10));
      options.has_travel_degrees = true;
    } else if (argument == "--all" || argument == "all") {
      if (options.select_all || !options.requested_ids.empty()) {
        throw std::invalid_argument("--all 不能和指定 ID 同时使用");
      }
      options.select_all = true;
    } else if (!argument.empty() && argument.front() == '-') {
      throw std::invalid_argument("未知选项: " + argument);
    } else {
      if (options.select_all) {
        throw std::invalid_argument("--all 不能和指定 ID 同时使用");
      }

      std::size_t begin = 0;
      while (begin <= argument.size()) {
        const std::size_t comma = argument.find(',', begin);
        AppendRequestedId(
            argument.substr(begin, comma == std::string::npos
                                       ? std::string::npos
                                       : comma - begin),
            &options.requested_ids);
        if (comma == std::string::npos) {
          break;
        }
        begin = comma + 1;
      }
    }
  }
  if (!options.has_travel_degrees) {
    throw std::invalid_argument("必须通过 --degrees 指定相对运动角度");
  }
  return options;
}

std::vector<uint8_t> ResolveSelectedIds(
    const CommandLineOptions& options,
    const std::vector<uint8_t>& scanned_ids) {
  if (scanned_ids.empty()) {
    throw std::runtime_error("未扫描到在线关节");
  }
  if (options.select_all) {
    std::vector<uint8_t> online_wheel_ids;
    for (const uint8_t id : kWheelIds) {
      if (std::find(scanned_ids.begin(), scanned_ids.end(), id) !=
          scanned_ids.end()) {
        online_wheel_ids.push_back(id);
      }
    }
    if (online_wheel_ids.empty()) {
      throw std::runtime_error("没有扫描到在线轮毂 2/4/6/8");
    }
    return online_wheel_ids;
  }
  if (options.requested_ids.empty()) {
    for (const uint8_t id : kWheelIds) {
      if (std::find(scanned_ids.begin(), scanned_ids.end(), id) !=
          scanned_ids.end()) {
        return {id};
      }
    }
    throw std::runtime_error("没有扫描到在线轮毂 2/4/6/8");
  }

  for (const uint8_t id : options.requested_ids) {
    if (std::find(scanned_ids.begin(), scanned_ids.end(), id) ==
        scanned_ids.end()) {
      throw std::runtime_error("指定关节不在线, ID=" +
                               std::to_string(static_cast<int>(id)));
    }
  }
  return options.requested_ids;
}

void RequireAllSteeringJointsOnline(
    const std::vector<uint8_t>& scanned_ids) {
  for (const uint8_t id : kSteeringIds) {
    if (std::find(scanned_ids.begin(), scanned_ids.end(), id) ==
        scanned_ids.end()) {
      throw std::runtime_error(
          "转向关节必须全部在线，未扫描到 ID=" +
          std::to_string(static_cast<int>(id)));
    }
  }
}

void SignalHandler(int) {
  const RunState current = g_run_state.load(std::memory_order_relaxed);
  g_run_state.store(current == RunState::kRunning
                        ? RunState::kStopRequested
                        : RunState::kRtExit,
                    std::memory_order_release);
}

void PrintJointDiag(modi_joint_canevo& joint, uint8_t node_id) {
  const auto state = joint.NrtGetServoState();
  const auto mode = joint.NrtGetCurrentMode();
  const auto fault = joint.NrtGetFaultCode();
  const auto warn = joint.NrtGetWarnCode();

  std::cerr << "关节 " << static_cast<int>(node_id)
            << " 诊断: servo_state=" << static_cast<int>(state)
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

bool WaitServoState(modi_joint_canevo& joint,
                    CanEvoServoState target,
                    const char* action) {
  const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (joint.NrtGetServoState() == target) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << action << "超时" << std::endl;
  return false;
}

bool WaitControlMode(modi_joint_canevo& joint,
                     CanEvoMode target,
                     const char* action) {
  const auto deadline = std::chrono::steady_clock::now() + kStateTimeout;
  CanEvoMode current = joint.NrtGetCurrentMode();
  while (std::chrono::steady_clock::now() < deadline) {
    current = joint.NrtGetCurrentMode();
    if (current == target) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << action << "超时, 当前模式=" << static_cast<int>(current)
            << ", 目标模式=" << static_cast<int>(target) << std::endl;
  return false;
}

void DisableAndDestroyAll(
    std::vector<modi_joint_canevo>& steering_joints,
    std::vector<modi_joint_canevo>& wheel_joints,
    modi_bus_canevo& bus) {
  for (auto& joint : wheel_joints) {
    joint.NrtDisable();
  }
  for (auto& joint : steering_joints) {
    joint.NrtDisable();
  }
  for (auto& joint : wheel_joints) {
    joint.NrtDestroy();
  }
  for (auto& joint : steering_joints) {
    joint.NrtDestroy();
  }
  bus.Close();
}

float BrakingDistance(float velocity_rad_s) {
  return velocity_rad_s * velocity_rad_s / (2.0f * kPvProfileDecRadSS);
}

void SetRtFailure(int error,
                  FailedJointGroup group,
                  std::size_t joint_index) {
  g_rt_failed_joint_group.store(group, std::memory_order_relaxed);
  g_rt_failed_joint_index.store(static_cast<int>(joint_index),
                                std::memory_order_relaxed);
  g_rt_ret.store(error, std::memory_order_release);
}

void PrintRtFailureDetails(
    int rt_ret,
    std::vector<modi_joint_canevo>& steering_joints,
    std::vector<modi_joint_canevo>& wheel_joints,
    const std::vector<uint8_t>& selected_wheel_ids) {
  const int failed_index =
      g_rt_failed_joint_index.load(std::memory_order_acquire);
  const auto failed_group =
      g_rt_failed_joint_group.load(std::memory_order_acquire);

  modi_joint_canevo* failed_joint = nullptr;
  uint8_t failed_id = 0;
  const char* group_name = "未知";

  if (failed_index >= 0 && failed_group == FailedJointGroup::kSteering &&
      static_cast<std::size_t>(failed_index) < steering_joints.size()) {
    const auto index = static_cast<std::size_t>(failed_index);
    failed_joint = &steering_joints[index];
    failed_id = kSteeringIds[index];
    group_name = "转向";
  } else if (failed_index >= 0 &&
             failed_group == FailedJointGroup::kWheel &&
             static_cast<std::size_t>(failed_index) < wheel_joints.size()) {
    const auto index = static_cast<std::size_t>(failed_index);
    failed_joint = &wheel_joints[index];
    failed_id = selected_wheel_ids[index];
    group_name = "轮毂";
  }

  if (failed_joint != nullptr) {
    std::cerr << "实时线程失败" << group_name
              << "关节 ID=" << static_cast<int>(failed_id) << std::endl;
  }

  if (rt_ret == kRtFault) {
    std::cerr << "实时线程出错: CanEvoServoState::kFault" << std::endl;
  } else if (rt_ret == kRtUnexpectedModeOrState) {
    if (failed_group == FailedJointGroup::kSteering) {
      std::cerr << "实时线程出错: 转向关节不在 PP Running 状态"
                << std::endl;
    } else if (failed_group == FailedJointGroup::kWheel) {
      std::cerr << "实时线程出错: 轮毂不在 PV Running 状态"
                << std::endl;
    } else {
      std::cerr << "实时线程出错: 关节模式或伺服状态异常" << std::endl;
    }
  } else {
    std::cerr << "实时线程出错, ret=" << rt_ret << std::endl;
  }

  if (failed_joint != nullptr) {
    PrintJointDiag(*failed_joint, failed_id);
  }
}

void* RtLoop(void*) {
  timespec next_wakeup{};
  __RT(clock_gettime(CLOCK_MONOTONIC, &next_wakeup));

  const long period_ns = g_period_us * 1000L;
  const float period_s = static_cast<float>(g_period_us) / 1000000.0f;
  const int ok = static_cast<int>(CanEvoError::kOk);

  while (g_run_state.load(std::memory_order_acquire) != RunState::kRtExit) {
    next_wakeup = AddNs(next_wakeup, period_ns);
    const int sleep_ret = __RT(
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr));
    if (sleep_ret != 0) {
      g_rt_ret.store(sleep_ret, std::memory_order_release);
      break;
    }

    const int step_ret = g_bus->RtStepOnce();
    if (step_ret != ok) {
      g_rt_ret.store(step_ret, std::memory_order_release);
      break;
    }

    const bool steering_was_homed =
        g_steering_home_complete.load(std::memory_order_acquire);
    bool all_steering_homed = true;

    for (std::size_t index = 0; index < g_steering_joints->size(); ++index) {
      auto& joint = (*g_steering_joints)[index];
      const auto servo_state = joint.RtGetServoState();
      const auto mode = joint.RtGetCurrentMode();
      const bool in_pp = servo_state == CanEvoServoState::kRunning &&
                         mode == CanEvoMode::kPp;
      if (!in_pp) {
        if (!steering_was_homed) {
          all_steering_homed = false;
        }

        ++g_steering_rt_transient_cycles[index];
        const bool startup_grace_expired =
            g_steering_rt_transient_cycles[index] >=
            kRtStateStartupGraceCycles;
        if (g_steering_rt_state_seen[index] != 0 ||
            startup_grace_expired) {
          SetRtFailure(servo_state == CanEvoServoState::kFault
                           ? kRtFault
                           : kRtUnexpectedModeOrState,
                       FailedJointGroup::kSteering, index);
          break;
        }
        continue;
      }

      JointStatus status{};
      const int status_ret = joint.RtGetJointStatus(status);
      if (status_ret != ok) {
        if (!steering_was_homed) {
          all_steering_homed = false;
        }
        ++g_steering_rt_transient_cycles[index];
        if (g_steering_rt_transient_cycles[index] >=
            kRtStatusReadGraceCycles) {
          SetRtFailure(status_ret, FailedJointGroup::kSteering, index);
          break;
        }
        continue;
      }
      g_steering_rt_state_seen[index] = 1;
      g_steering_rt_transient_cycles[index] = 0;

      if (g_steering_home_target_sent[index] == 0) {
        const int set_ret = joint.RtSetPpTargetPosition(
            0.0f, kSteeringProfileVelocityRadS,
            kSteeringProfileAccRadSS, kSteeringProfileAccRadSS);
        if (set_ret != ok) {
          SetRtFailure(set_ret, FailedJointGroup::kSteering, index);
          break;
        }
        g_steering_home_target_sent[index] = 1;
        g_steering_home_stable_cycles[index] = 0;
        all_steering_homed = false;
        continue;
      }

      if (!steering_was_homed) {
        if (std::fabs(status.actual_pos) <=
                kSteeringHomePositionToleranceRad &&
            std::fabs(status.actual_vel) <=
                kSteeringHomeVelocityToleranceRadS) {
          ++g_steering_home_stable_cycles[index];
        } else {
          g_steering_home_stable_cycles[index] = 0;
        }

        if (g_steering_home_stable_cycles[index] <
            kSteeringHomeStableCycles) {
          all_steering_homed = false;
        }
      }
    }

    if (g_rt_ret.load(std::memory_order_acquire) != ok) {
      break;
    }

    if (!steering_was_homed && all_steering_homed) {
      g_steering_home_complete.store(true, std::memory_order_release);
    }

    if (!g_wheel_motion_started.load(std::memory_order_acquire)) {
      continue;
    }

    const bool stopping =
        g_run_state.load(std::memory_order_acquire) ==
        RunState::kStopRequested;
    bool all_stopped = stopping;
    bool all_motion_complete = !stopping;

    for (std::size_t index = 0; index < g_wheel_joints->size(); ++index) {
      auto& joint = (*g_wheel_joints)[index];
      const auto servo_state = joint.RtGetServoState();
      const auto mode = joint.RtGetCurrentMode();
      const bool in_pv = servo_state == CanEvoServoState::kRunning &&
                         mode == CanEvoMode::kPv;
      if (!in_pv) {
        if (stopping) {
          all_stopped = false;
        } else {
          all_motion_complete = false;
        }

        ++g_wheel_rt_transient_cycles[index];
        const bool startup_grace_expired =
            g_wheel_rt_transient_cycles[index] >=
            kRtStateStartupGraceCycles;
        if (g_wheel_rt_state_seen[index] != 0 ||
            startup_grace_expired) {
          SetRtFailure(servo_state == CanEvoServoState::kFault
                           ? kRtFault
                           : kRtUnexpectedModeOrState,
                       FailedJointGroup::kWheel, index);
          break;
        }
        continue;
      }

      JointStatus status{};
      const int status_ret = joint.RtGetJointStatus(status);
      if (status_ret != ok) {
        if (stopping) {
          all_stopped = false;
        } else {
          all_motion_complete = false;
        }
        ++g_wheel_rt_transient_cycles[index];
        if (g_wheel_rt_transient_cycles[index] >=
            kRtStatusReadGraceCycles) {
          SetRtFailure(status_ret, FailedJointGroup::kWheel, index);
          break;
        }
        continue;
      }
      g_wheel_rt_state_seen[index] = 1;
      g_wheel_rt_transient_cycles[index] = 0;

      float target_velocity = 0.0f;
      if (stopping) {
        if (std::fabs(status.actual_vel) <= kStopVelocityToleranceRadS) {
          ++g_stopped_stable_cycles[index];
        } else {
          g_stopped_stable_cycles[index] = 0;
        }
        if (g_stopped_stable_cycles[index] < kStopStableCycles) {
          all_stopped = false;
        }
      } else {
        auto phase = static_cast<MotionPhase>(g_motion_phases[index]);
        if (phase == MotionPhase::kMoving) {
          const float direction = g_move_directions[index];
          const float remaining_rad =
              direction * (g_target_positions[index] - status.actual_pos);
          const float velocity_toward_target =
              std::max(direction * status.actual_vel, 0.0f);
          const float stopping_distance =
              BrakingDistance(velocity_toward_target) +
              velocity_toward_target * period_s;

          if (remaining_rad <= stopping_distance) {
            phase = MotionPhase::kStopping;
            g_motion_phases[index] = static_cast<uint8_t>(phase);
          } else {
            target_velocity = direction * kPvSpeedRadS;
          }
        }

        if (phase == MotionPhase::kStopping) {
          if (std::fabs(status.actual_vel) <= kStopVelocityToleranceRadS) {
            ++g_stopped_stable_cycles[index];
          } else {
            g_stopped_stable_cycles[index] = 0;
          }
          if (g_stopped_stable_cycles[index] >= kStopStableCycles) {
            phase = MotionPhase::kComplete;
            g_motion_phases[index] = static_cast<uint8_t>(phase);
          }
        }

        if (phase != MotionPhase::kComplete) {
          all_motion_complete = false;
        }
      }

      const int set_ret = joint.RtSetPvTargetVelocity(
          target_velocity, kPvProfileAccRadSS, kPvProfileDecRadSS);
      if (set_ret != ok) {
        SetRtFailure(set_ret, FailedJointGroup::kWheel, index);
        break;
      }
    }

    if (g_rt_ret.load(std::memory_order_acquire) != ok) {
      break;
    }

    if (stopping && all_stopped) {
      g_stop_complete.store(true, std::memory_order_release);
    }
    if (!stopping && all_motion_complete) {
      g_motion_complete.store(true, std::memory_order_release);
    }
  }

  return nullptr;
}

}  // namespace

int main(int argc, char* argv[]) {
  for (int index = 1; index < argc; ++index) {
    const std::string argument(argv[index]);
    if (argument == "-h" || argument == "--help") {
      PrintUsage(argv[0]);
      return 0;
    }
  }

  CommandLineOptions options;
  try {
    options = ParseOptions(argc, argv);
  } catch (const std::exception& error) {
    std::cerr << "✗ " << error.what() << "\n\n";
    PrintUsage(argv[0]);
    return -1;
  }

  const float travel_rad = options.travel_degrees * kDegToRad;
  if (!std::isfinite(travel_rad)) {
    std::cerr << "✗ 运动角度换算为弧度后超出有效范围" << std::endl;
    return -1;
  }

  std::cout << "========================================" << std::endl;
  std::cout << "PV 单次相对角度测试 - 速度 11.0 rad/s"
            << std::endl;
  std::cout << "相对角度: " << options.travel_degrees << "°" << std::endl;
  std::cout << "========================================" << std::endl;

  qiuniu_init();
  std::cout << "NIIC hard realtime: "
            << (CANEVO_HAVE_NECRO ? "ON (__RT -> qiuniu)" : "OFF (POSIX)")
            << std::endl;

  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "警告: mlockall 失败，实时抖动可能增大" << std::endl;
    return -1;
  }

  pthread_attr_t attr;
  pthread_attr_init(&attr);

  int attr_ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setinheritsched 失败, errno=" << attr_ret
              << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  const int sched_policy = SCHED_FIFO;
  attr_ret = pthread_attr_setschedpolicy(&attr, sched_policy);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedpolicy 失败, errno=" << attr_ret
              << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  sched_param param{};
  param.sched_priority = 99;
  attr_ret = pthread_attr_setschedparam(&attr, &param);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedparam 失败, errno=" << attr_ret
              << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  const int cpu = 1;
  CPU_SET(cpu, &cpuset);
  attr_ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setaffinity_np 失败, errno=" << attr_ret
              << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }
  std::cout << "✓ 实时线程绑定 CPU" << cpu << std::endl;

  TaskConfig task_config;
  task_config.sync_period_us = 5000;
  task_config.cpu_affinity = 3;
  task_config.sched_policy = sched_policy;
  task_config.sched_priority = 90;
  g_period_us = task_config.sync_period_us;
  std::cout << "✓ SDK Rx 线程绑定 CPU" << task_config.cpu_affinity
            << ", 优先级 " << task_config.sched_priority << std::endl;

  modi_bus_canevo bus;
  const std::vector<uint8_t> steering_ids(kSteeringIds.begin(),
                                          kSteeringIds.end());
  std::vector<modi_joint_canevo> steering_joints(kSteeringIds.size());
  std::vector<modi_joint_canevo> wheel_joints;
  std::vector<uint8_t> selected_wheel_ids;
  g_bus = &bus;
  g_steering_joints = &steering_joints;
  g_wheel_joints = &wheel_joints;

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  const int ok = static_cast<int>(CanEvoError::kOk);
  if (bus.Open("can0", task_config) != ok) {
    std::cerr << "✗ 无法打开 CAN 总线 can0" << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  const auto scanned_ids = bus.NrtScanJoints();
  std::cout << "✓ 扫描到 " << scanned_ids.size() << " 个关节, ID: ";
  for (const uint8_t id : scanned_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  try {
    RequireAllSteeringJointsOnline(scanned_ids);
    selected_wheel_ids = ResolveSelectedIds(options, scanned_ids);
  } catch (const std::exception& error) {
    std::cerr << "✗ " << error.what() << std::endl;
    pthread_attr_destroy(&attr);
    bus.Close();
    return -1;
  }

  std::cout << "✓ 固定转向关节 ID: 1 3 5 7" << std::endl;
  std::cout << "✓ 本次选择轮毂 ID: ";
  for (const uint8_t id : selected_wheel_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  std::vector<modi_joint_canevo> new_wheel_joints(
      selected_wheel_ids.size());
  wheel_joints.swap(new_wheel_joints);

  const auto initialize_joints =
      [&](std::vector<modi_joint_canevo>& joints,
          const std::vector<uint8_t>& ids,
          const char* group_name) {
        for (std::size_t index = 0; index < ids.size(); ++index) {
          if (joints[index].NrtInit(bus, ids[index]) != ok) {
            std::cerr << "✗ " << group_name << "关节初始化失败, ID="
                      << static_cast<int>(ids[index]) << std::endl;
            return false;
          }
          std::cout << "✓ " << group_name << " Joint 初始化完成 (Node ID: "
                    << static_cast<int>(ids[index]) << ")" << std::endl;
        }
        return true;
      };

  if (!initialize_joints(steering_joints, steering_ids, "转向") ||
      !initialize_joints(wheel_joints, selected_wheel_ids, "轮毂")) {
    pthread_attr_destroy(&attr);
    DisableAndDestroyAll(steering_joints, wheel_joints, bus);
    return -1;
  }

  const auto clear_faults =
      [&](std::vector<modi_joint_canevo>& joints,
          const std::vector<uint8_t>& ids) {
        for (std::size_t index = 0; index < joints.size(); ++index) {
          const auto fault_code = joints[index].NrtGetFaultCode();
          std::cout << "ID=" << static_cast<int>(ids[index])
                    << " fault_code=0x" << std::hex
                    << static_cast<uint16_t>(fault_code) << std::dec
                    << std::endl;
          if (fault_code == CanEvoFault::kNone) {
            continue;
          }

          std::cerr << "检测到故障，先清除故障, ID="
                    << static_cast<int>(ids[index]) << std::endl;
          PrintJointDiag(joints[index], ids[index]);
          const int clear_ret = joints[index].NrtClearFault();
          if (clear_ret != ok) {
            std::cerr << "✗ 清除故障失败, ID="
                      << static_cast<int>(ids[index])
                      << ", ret=" << clear_ret << std::endl;
            return false;
          }
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          if (joints[index].NrtGetFaultCode() != CanEvoFault::kNone) {
            std::cerr << "✗ 故障仍存在，不允许运动, ID="
                      << static_cast<int>(ids[index]) << std::endl;
            PrintJointDiag(joints[index], ids[index]);
            return false;
          }
        }
        return true;
      };

  if (!clear_faults(steering_joints, steering_ids) ||
      !clear_faults(wheel_joints, selected_wheel_ids)) {
    pthread_attr_destroy(&attr);
    DisableAndDestroyAll(steering_joints, wheel_joints, bus);
    return -1;
  }

  for (std::size_t index = 0; index < wheel_joints.size(); ++index) {
    const int disable_ret = wheel_joints[index].NrtDisable();
    if (disable_ret != ok) {
      std::cerr << "✗ 回零前失能轮毂失败, ID="
                << static_cast<int>(selected_wheel_ids[index])
                << ", ret=" << disable_ret << std::endl;
      pthread_attr_destroy(&attr);
      DisableAndDestroyAll(steering_joints, wheel_joints, bus);
      return -1;
    }
  }

  for (std::size_t index = 0; index < steering_joints.size(); ++index) {
    if (steering_joints[index].NrtEnable(CanEvoMode::kPp) != ok) {
      std::cerr << "✗ 使能转向关节 PP 模式失败, ID="
                << static_cast<int>(steering_ids[index]) << std::endl;
      PrintJointDiag(steering_joints[index], steering_ids[index]);
      pthread_attr_destroy(&attr);
      DisableAndDestroyAll(steering_joints, wheel_joints, bus);
      return -1;
    }
  }

  for (std::size_t index = 0; index < steering_joints.size(); ++index) {
    if (!WaitServoState(steering_joints[index],
                        CanEvoServoState::kRunning,
                        "等待转向关节 PP Running 状态") ||
        !WaitControlMode(steering_joints[index], CanEvoMode::kPp,
                         "等待转向关节 PP 模式切换")) {
      PrintJointDiag(steering_joints[index], steering_ids[index]);
      pthread_attr_destroy(&attr);
      DisableAndDestroyAll(steering_joints, wheel_joints, bus);
      return -1;
    }
  }

  g_steering_home_target_sent.assign(steering_joints.size(), 0);
  g_steering_home_stable_cycles.assign(steering_joints.size(), 0);
  g_steering_rt_state_seen.assign(steering_joints.size(), 0);
  g_steering_rt_transient_cycles.assign(steering_joints.size(), 0);
  g_wheel_rt_state_seen.assign(wheel_joints.size(), 0);
  g_wheel_rt_transient_cycles.assign(wheel_joints.size(), 0);

  pthread_t rt_thread{};
  const int create_ret =
      __RT(pthread_create(&rt_thread, &attr, RtLoop, nullptr));
  if (create_ret == 0) {
    __RT(pthread_setname_np(rt_thread, "canevo_pv_rt"));
  }
  pthread_attr_destroy(&attr);
  if (create_ret != 0) {
    std::cerr << "✗ 创建实时线程失败, errno=" << create_ret << std::endl;
    DisableAndDestroyAll(steering_joints, wheel_joints, bus);
    return -1;
  }

  std::cout << "✓ 转向关节 1/3/5/7 已进入 PP Running 状态" << std::endl;
  std::cout << "正在将转向关节 1/3/5/7 回到绝对零位..." << std::endl;

  const auto steering_home_deadline =
      std::chrono::steady_clock::now() + kSteeringHomeTimeout;
  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         !g_steering_home_complete.load(std::memory_order_acquire) &&
         g_rt_ret.load(std::memory_order_acquire) == ok &&
         std::chrono::steady_clock::now() < steering_home_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  const bool steering_home_timed_out =
      !g_steering_home_complete.load(std::memory_order_acquire) &&
      g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
      g_rt_ret.load(std::memory_order_acquire) == ok;

  if (!g_steering_home_complete.load(std::memory_order_acquire)) {
    if (steering_home_timed_out) {
      std::cerr << "✗ 等待转向关节 1/3/5/7 回零超时，"
                   "不会启动轮毂"
                << std::endl;
    } else if (g_run_state.load(std::memory_order_acquire) !=
               RunState::kRunning) {
      std::cerr << "收到退出信号，回零中止，不会启动轮毂" << std::endl;
    }

    g_run_state.store(RunState::kRtExit, std::memory_order_release);
    __RT(pthread_join(rt_thread, nullptr));
    PrintEndTime();

    const int rt_ret = g_rt_ret.load(std::memory_order_acquire);
    if (rt_ret != ok) {
      PrintRtFailureDetails(rt_ret, steering_joints, wheel_joints,
                            selected_wheel_ids);
    } else if (steering_home_timed_out) {
      for (std::size_t index = 0; index < steering_joints.size(); ++index) {
        std::cerr << "转向 ID=" << static_cast<int>(steering_ids[index])
                  << ", 当前位置="
                  << steering_joints[index].NrtGetActualPosition() /
                         kDegToRad
                  << "°, 当前速度="
                  << steering_joints[index].NrtGetActualVelocity()
                  << " rad/s" << std::endl;
        PrintJointDiag(steering_joints[index], steering_ids[index]);
      }
    }

    DisableAndDestroyAll(steering_joints, wheel_joints, bus);
    return -1;
  }

  std::cout << "✓ 转向关节 1/3/5/7 已全部回到零位并保持 PP 使能"
            << std::endl;

  bool wheel_start_failed = false;
  for (std::size_t index = 0; index < wheel_joints.size(); ++index) {
    const int init_ret = wheel_joints[index].RtSetPvTargetVelocity(
        0.0f, kPvProfileAccRadSS, kPvProfileDecRadSS);
    if (init_ret != ok) {
      std::cerr << "✗ 预置轮毂 PV 零速度失败, ID="
                << static_cast<int>(selected_wheel_ids[index])
                << ", ret=" << init_ret << std::endl;
      wheel_start_failed = true;
      break;
    }
  }

  if (!wheel_start_failed &&
      g_run_state.load(std::memory_order_acquire) == RunState::kRunning) {
    for (std::size_t index = 0; index < wheel_joints.size(); ++index) {
      if (wheel_joints[index].NrtEnable(CanEvoMode::kPv) != ok) {
        std::cerr << "✗ 使能轮毂 PV 模式失败, ID="
                  << static_cast<int>(selected_wheel_ids[index])
                  << std::endl;
        PrintJointDiag(wheel_joints[index], selected_wheel_ids[index]);
        wheel_start_failed = true;
        break;
      }
    }
  } else if (!wheel_start_failed) {
    wheel_start_failed = true;
  }

  if (!wheel_start_failed) {
    for (std::size_t index = 0; index < wheel_joints.size(); ++index) {
      if (!WaitServoState(wheel_joints[index],
                          CanEvoServoState::kRunning,
                          "等待轮毂 PV Running 状态") ||
          !WaitControlMode(wheel_joints[index], CanEvoMode::kPv,
                           "等待轮毂 PV 模式切换")) {
        PrintJointDiag(wheel_joints[index], selected_wheel_ids[index]);
        wheel_start_failed = true;
        break;
      }
    }
  }

  g_target_positions.resize(wheel_joints.size());
  g_move_directions.resize(wheel_joints.size());
  g_motion_phases.assign(
      wheel_joints.size(), static_cast<uint8_t>(MotionPhase::kMoving));
  g_stopped_stable_cycles.assign(wheel_joints.size(), 0);

  if (!wheel_start_failed) {
    for (std::size_t index = 0; index < wheel_joints.size(); ++index) {
      const float initial_position =
          wheel_joints[index].NrtGetActualPosition();
      const float joint_travel_rad =
          travel_rad * WheelDirectionSign(selected_wheel_ids[index]);
      if (!std::isfinite(initial_position)) {
        std::cerr << "✗ 轮毂初始位置无效, ID="
                  << static_cast<int>(selected_wheel_ids[index])
                  << std::endl;
        wheel_start_failed = true;
        break;
      }

      g_move_directions[index] =
          joint_travel_rad > 0.0f ? 1.0f : -1.0f;
      g_target_positions[index] = initial_position + joint_travel_rad;
      if (!std::isfinite(g_target_positions[index])) {
        std::cerr << "✗ 轮毂目标位置超出有效范围, ID="
                  << static_cast<int>(selected_wheel_ids[index])
                  << std::endl;
        wheel_start_failed = true;
        break;
      }

      std::cout << "轮毂 ID="
                << static_cast<int>(selected_wheel_ids[index])
                << ", 起点=" << initial_position / kDegToRad
                << "°, 目标=" << g_target_positions[index] / kDegToRad
                << "°, 命令位移=" << options.travel_degrees
                << "°, 关节位移=" << joint_travel_rad / kDegToRad << "°"
                << std::endl;

      const int init_ret = wheel_joints[index].RtSetPvTargetVelocity(
          0.0f, kPvProfileAccRadSS, kPvProfileDecRadSS);
      if (init_ret != ok) {
        std::cerr << "✗ 初始 PV PDO 零速度发送失败, ID="
                  << static_cast<int>(selected_wheel_ids[index])
                  << ", ret=" << init_ret << std::endl;
        wheel_start_failed = true;
        break;
      }
    }
  }

  if (g_rt_ret.load(std::memory_order_acquire) != ok ||
      g_run_state.load(std::memory_order_acquire) != RunState::kRunning) {
    wheel_start_failed = true;
  }

  if (wheel_start_failed) {
    g_run_state.store(RunState::kRtExit, std::memory_order_release);
    __RT(pthread_join(rt_thread, nullptr));
    PrintEndTime();
    const int rt_ret = g_rt_ret.load(std::memory_order_acquire);
    if (rt_ret != ok) {
      PrintRtFailureDetails(rt_ret, steering_joints, wheel_joints,
                            selected_wheel_ids);
    }
    DisableAndDestroyAll(steering_joints, wheel_joints, bus);
    return -1;
  }

  g_wheel_motion_started.store(true, std::memory_order_release);

  std::cout << "✓ 所选轮毂已切换到 PV 模式，转向关节继续保持零位"
            << std::endl;
  std::cout << "轮毂以 11.0 rad/s 运动 " << options.travel_degrees
            << "° 一次；到达后自动停止、全部失能并退出。"
               "按 Ctrl+C 可提前停止，再次按 Ctrl+C 强制退出..."
            << std::endl;

  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         !g_motion_complete.load(std::memory_order_acquire) &&
         g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  bool stop_timed_out = false;
  if (g_motion_complete.load(std::memory_order_acquire)) {
    std::cout << "✓ 所选轮毂已完成指定相对角度并停止" << std::endl;
  } else if (g_run_state.load(std::memory_order_acquire) ==
                 RunState::kStopRequested &&
             g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::cout << "收到退出信号，向全部所选轮毂下发 PV 目标速度 0，"
                 "等待轮廓减速完成..."
              << std::endl;
    const auto stop_deadline =
        std::chrono::steady_clock::now() + kStopTimeout;
    while (!g_stop_complete.load(std::memory_order_acquire) &&
           g_rt_ret.load(std::memory_order_acquire) == ok &&
           g_run_state.load(std::memory_order_acquire) ==
               RunState::kStopRequested &&
           std::chrono::steady_clock::now() < stop_deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    stop_timed_out =
        !g_stop_complete.load(std::memory_order_acquire) &&
        g_run_state.load(std::memory_order_acquire) != RunState::kRtExit &&
        g_rt_ret.load(std::memory_order_acquire) == ok;
    if (g_stop_complete.load(std::memory_order_acquire)) {
      std::cout << "✓ 所选轮毂已减速至停止阈值" << std::endl;
    } else if (stop_timed_out) {
      std::cerr << "警告: 等待 PV 减速停止超时，将执行失能" << std::endl;
    } else if (g_run_state.load(std::memory_order_acquire) ==
               RunState::kRtExit) {
      std::cerr << "收到第二次退出信号，强制结束实时循环" << std::endl;
    }
  }

  g_run_state.store(RunState::kRtExit, std::memory_order_release);
  __RT(pthread_join(rt_thread, nullptr));
  PrintEndTime();

  const int rt_ret = g_rt_ret.load(std::memory_order_acquire);
  if (rt_ret != ok) {
    PrintRtFailureDetails(rt_ret, steering_joints, wheel_joints,
                          selected_wheel_ids);
  }

  DisableAndDestroyAll(steering_joints, wheel_joints, bus);
  return rt_ret == ok && !stop_timed_out ? 0 : -1;
}
