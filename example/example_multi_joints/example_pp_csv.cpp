/**
 * @file example_pp_csv.cpp
 * @brief 8 joint mixed-mode test: 1/3/5/7 use PP PDO3, 2/4/6/8 use CSV PDO1.
 *
 * Run after configuring SocketCAN CAN-FD externally:
 *   sudo ip link set can0 down
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on restart-ms 100
 *   sudo ip link set can0 txqueuelen 1
 *   sudo ip link set can0 up
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
#include <iostream>
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

constexpr float kPi = 3.14159265358979323846f;
constexpr float kDegToRad = kPi / 180.0f;

constexpr float kLimitMinRad = -30.0f * kDegToRad;
constexpr float kLimitMaxRad = 30.0f * kDegToRad;

constexpr float kPpTravelRad = 10.0f * kDegToRad;
constexpr float kPpVelRadS = 5.0f * kDegToRad;
constexpr float kPpAccRadSS = 20.0f * kDegToRad;
constexpr float kPpTolRad = 0.2f * kDegToRad;

constexpr float kCsvSpeedRadS = 3.0f * kDegToRad;

constexpr std::array<uint8_t, 4> kPpIds = {1, 3, 5, 7};
constexpr std::array<uint8_t, 4> kCsvIds = {2, 4, 6, 8};
constexpr std::array<uint8_t, 8> kExpectedIds = {1, 2, 3, 4, 5, 6, 7, 8};

static modi_bus_canevo* g_bus = nullptr;
static std::vector<modi_joint_canevo>* g_joints = nullptr;
static std::vector<uint8_t>* g_joint_ids = nullptr;

static std::vector<float> g_pp_target_rad;
static std::vector<float> g_pp_dir;
static std::vector<float> g_csv_vel_rad_s;

static int g_period_us = 5000;
static std::atomic<int> g_rt_ret{0};

enum class RunState {
  kRunning,
  kStopRequested,
  kRtExit,
};

static std::atomic<RunState> g_run_state{RunState::kRunning};

bool Contains(const std::array<uint8_t, 4>& ids, uint8_t id) {
  return std::find(ids.begin(), ids.end(), id) != ids.end();
}

bool IsPpId(uint8_t id) { return Contains(kPpIds, id); }

bool IsCsvId(uint8_t id) { return Contains(kCsvIds, id); }

float Clamp(float v, float lo, float hi) { return std::max(lo, std::min(v, hi)); }

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

void SignalHandler(int) {
  g_run_state.store(RunState::kStopRequested, std::memory_order_release);
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

bool WaitServoState(modi_joint_canevo& joint, CanEvoServoState target,
                    const char* action) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    if (joint.NrtGetServoState() == target) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cerr << action << "超时" << std::endl;
  return false;
}

bool WaitControlMode(modi_joint_canevo& joint, CanEvoMode target,
                     const char* action) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
  CanEvoMode current = joint.NrtGetCurrentMode();
  while (std::chrono::steady_clock::now() < deadline) {
    current = joint.NrtGetCurrentMode();
    if (current == target) return true;
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::cerr << action << "超时, 当前模式=" << static_cast<int>(current)
            << ", 目标模式=" << static_cast<int>(target) << std::endl;
  return false;
}

void DisableAndDestroyAll(std::vector<modi_joint_canevo>& joints,
                          modi_bus_canevo& bus) {
  for (auto& joint : joints) {
    joint.NrtDisable();
    joint.NrtDestroy();
  }
  bus.Close();
}

void* RtLoop(void*) {
  timespec next_wakeup{};
  __RT(clock_gettime(CLOCK_MONOTONIC, &next_wakeup));

  const long period_ns = g_period_us * 1000L;
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

    const bool stopping =
        g_run_state.load(std::memory_order_acquire) == RunState::kStopRequested;

    for (size_t i = 0; i < g_joints->size(); ++i) {
      auto& joint = (*g_joints)[i];
      const uint8_t id = (*g_joint_ids)[i];

      if (joint.RtGetServoState() == CanEvoServoState::kFault) {
        g_rt_ret.store(-1000, std::memory_order_release);
        break;
      }

      JointStatus st{};
      if (joint.RtGetJointStatus(st) != ok) continue;

      if (IsCsvId(id)) {
        if (stopping) {
          joint.RtSetCsvTargetVelocity(0.0f);
          continue;
        }

        if (st.actual_pos >= kLimitMaxRad) {
          g_csv_vel_rad_s[i] = -kCsvSpeedRadS;
        } else if (st.actual_pos <= kLimitMinRad) {
          g_csv_vel_rad_s[i] = kCsvSpeedRadS;
        }

        const int ret = joint.RtSetCsvTargetVelocity(g_csv_vel_rad_s[i]);
        if (ret != ok) {
          g_rt_ret.store(ret, std::memory_order_release);
          break;
        }
      }

      if (IsPpId(id) && !stopping) {
        if (std::fabs(st.actual_pos - g_pp_target_rad[i]) <= kPpTolRad) {
          g_pp_dir[i] = -g_pp_dir[i];
          g_pp_target_rad[i] =
              Clamp(st.actual_pos + g_pp_dir[i] * kPpTravelRad, kLimitMinRad,
                    kLimitMaxRad);

          const int ret = joint.RtSetPpTargetPosition(
              g_pp_target_rad[i], kPpVelRadS, kPpAccRadSS, kPpAccRadSS);
          if (ret != ok) {
            g_rt_ret.store(ret, std::memory_order_release);
            break;
          }
        }
      }
    }
  }

  return nullptr;
}

int main() {
  std::cout << "========================================" << std::endl;
  std::cout << "PP + CSV 混合测试: 1/3/5/7=PP, 2/4/6/8=CSV" << std::endl;
  std::cout << "PP: 当前位置附近 ±10°, CSV: ±30° 内 3°/s 往复" << std::endl;
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

  attr_ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
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
  CPU_SET(1, &cpuset);
  attr_ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setaffinity_np 失败, errno=" << attr_ret
              << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  TaskConfig task_config;
  task_config.sync_period_us = 5000;
  task_config.cpu_affinity = 3;
  task_config.sched_policy = SCHED_FIFO;
  task_config.sched_priority = 90;
  g_period_us = task_config.sync_period_us;

  modi_bus_canevo bus;
  std::vector<modi_joint_canevo> joints;
  std::vector<uint8_t> joint_ids;

  g_bus = &bus;
  g_joints = &joints;
  g_joint_ids = &joint_ids;

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 无法打开 CAN 总线 can0" << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  const auto scanned_ids = bus.NrtScanJoints();
  std::cout << "扫描到 " << scanned_ids.size() << " 个关节, ID: ";
  for (const auto id : scanned_ids) std::cout << static_cast<int>(id) << " ";
  std::cout << std::endl;

  for (const auto id : kExpectedIds) {
    if (std::find(scanned_ids.begin(), scanned_ids.end(), id) ==
        scanned_ids.end()) {
      std::cerr << "✗ 缺少关节 ID=" << static_cast<int>(id) << std::endl;
      bus.Close();
      pthread_attr_destroy(&attr);
      return -1;
    }
  }

  joint_ids.assign(kExpectedIds.begin(), kExpectedIds.end());
  std::vector<modi_joint_canevo> tmp_joints(joint_ids.size());
  joints.swap(tmp_joints);

  for (size_t i = 0; i < joint_ids.size(); ++i) {
    if (joints[i].NrtInit(bus, joint_ids[i]) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 初始化失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      DisableAndDestroyAll(joints, bus);
      pthread_attr_destroy(&attr);
      return -1;
    }
    std::cout << "✓ 初始化关节 ID=" << static_cast<int>(joint_ids[i])
              << std::endl;
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    const auto fault = joints[i].NrtGetFaultCode();
    std::cout << "ID=" << static_cast<int>(joint_ids[i]) << " fault=0x"
              << std::hex << static_cast<uint16_t>(fault) << std::dec
              << std::endl;

    if (fault != CanEvoFault::kNone) {
      std::cerr << "检测到故障，尝试清除: ID="
                << static_cast<int>(joint_ids[i]) << std::endl;
      PrintJointDiag(joints[i], joint_ids[i]);
      if (joints[i].NrtClearFault() != static_cast<int>(CanEvoError::kOk)) {
        std::cerr << "✗ 清故障失败" << std::endl;
        DisableAndDestroyAll(joints, bus);
        pthread_attr_destroy(&attr);
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      if (joints[i].NrtGetFaultCode() != CanEvoFault::kNone) {
        std::cerr << "✗ 故障仍存在，不允许运动: ID="
                  << static_cast<int>(joint_ids[i]) << std::endl;
        PrintJointDiag(joints[i], joint_ids[i]);
        DisableAndDestroyAll(joints, bus);
        pthread_attr_destroy(&attr);
        return -1;
      }
    }
  }

  g_pp_target_rad.assign(joints.size(), 0.0f);
  g_pp_dir.assign(joints.size(), 1.0f);
  g_csv_vel_rad_s.assign(joints.size(), 0.0f);

  for (size_t i = 0; i < joints.size(); ++i) {
    const uint8_t id = joint_ids[i];
    const float pos =
        Clamp(joints[i].NrtGetActualPosition(), kLimitMinRad, kLimitMaxRad);

    if (IsPpId(id)) {
      g_pp_dir[i] = (pos >= 0.0f) ? -1.0f : 1.0f;
      g_pp_target_rad[i] =
          Clamp(pos + g_pp_dir[i] * kPpTravelRad, kLimitMinRad, kLimitMaxRad);
      std::cout << "ID=" << static_cast<int>(id)
                << " PP 初始目标(rad)=" << g_pp_target_rad[i] << std::endl;
    } else {
      g_csv_vel_rad_s[i] = (pos >= kLimitMaxRad) ? -kCsvSpeedRadS
                                                  : kCsvSpeedRadS;
      std::cout << "ID=" << static_cast<int>(id)
                << " CSV 初始速度(rad/s)=" << g_csv_vel_rad_s[i] << std::endl;
    }
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    const uint8_t id = joint_ids[i];
    const CanEvoMode mode = IsPpId(id) ? CanEvoMode::kPp : CanEvoMode::kCsv;

    if (joints[i].NrtEnable(mode) != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 使能失败, ID=" << static_cast<int>(id) << std::endl;
      PrintJointDiag(joints[i], id);
      DisableAndDestroyAll(joints, bus);
      pthread_attr_destroy(&attr);
      return -1;
    }

    if (!WaitServoState(joints[i], CanEvoServoState::kRunning,
                        "等待 Running")) {
      PrintJointDiag(joints[i], id);
      DisableAndDestroyAll(joints, bus);
      pthread_attr_destroy(&attr);
      return -1;
    }

    if (!WaitControlMode(joints[i], mode, "等待模式切换")) {
      PrintJointDiag(joints[i], id);
      DisableAndDestroyAll(joints, bus);
      pthread_attr_destroy(&attr);
      return -1;
    }
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    int ret = static_cast<int>(CanEvoError::kOk);
    if (IsPpId(joint_ids[i])) {
      ret = joints[i].RtSetPpTargetPosition(g_pp_target_rad[i], kPpVelRadS,
                                            kPpAccRadSS, kPpAccRadSS);
    } else {
      ret = joints[i].RtSetCsvTargetVelocity(g_csv_vel_rad_s[i]);
    }
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 初始 PDO 目标发送失败, ID="
                << static_cast<int>(joint_ids[i]) << ", ret=" << ret
                << std::endl;
      DisableAndDestroyAll(joints, bus);
      pthread_attr_destroy(&attr);
      return -1;
    }
  }

  pthread_t rt_thread{};
  const int create_ret =
      __RT(pthread_create(&rt_thread, &attr, RtLoop, nullptr));
  if (create_ret == 0) {
    __RT(pthread_setname_np(rt_thread, "pp_csv_mix"));
  }
  pthread_attr_destroy(&attr);

  if (create_ret != 0) {
    std::cerr << "✗ 创建实时线程失败, errno=" << create_ret << std::endl;
    DisableAndDestroyAll(joints, bus);
    return -1;
  }

  std::cout << "✓ 混合模式开始运行，按 Ctrl+C 停止" << std::endl;

  const int ok = static_cast<int>(CanEvoError::kOk);
  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (g_rt_ret.load(std::memory_order_acquire) != ok) {
    std::cerr << "✗ 实时线程异常退出, ret="
              << g_rt_ret.load(std::memory_order_acquire) << std::endl;
  } else {
    std::cout << "收到停止信号，CSV 速度清零..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  g_run_state.store(RunState::kRtExit, std::memory_order_release);
  __RT(pthread_join(rt_thread, nullptr));

  DisableAndDestroyAll(joints, bus);

  std::cout << "测试结束" << std::endl;
  return 0;
}
