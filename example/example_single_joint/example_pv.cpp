/**
 * @file example_pv.cpp
 * @brief PV 轮廓速度模式测试 - 外部实时线程调用 RtStepOnce()
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 txqueuelen 1
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 *
 * 测试内容：
 * 1. 使能 PV 模式
 * 2. 通过 RxPDO4 下发目标速度、轮廓加速度、轮廓减速度
 * 3. 在 -30°～+30° 范围内以 10 rpm 往复运动
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

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

constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kRpmToRadS = 2.0f * static_cast<float>(M_PI) / 60.0f;
constexpr float kPvSpeedRadS = 10.0f * kRpmToRadS;
constexpr float kPvAccRadSS = 10.0f * kRpmToRadS;
constexpr float kPvDecRadSS = 10.0f * kRpmToRadS;
constexpr float kPvMinPosRad = -30.0f * kDegToRad;
constexpr float kPvMaxPosRad = 30.0f * kDegToRad;

static modi_bus_canevo* g_bus = nullptr;
static modi_joint_canevo* g_joint = nullptr;
static int g_period_us = 5000;
static std::atomic<int> g_rt_ret{0};

enum class RunState {
  kRunning,
  kStopRequested,
  kRtExit,
};

static std::atomic<RunState> g_run_state{RunState::kRunning};

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

int SendPvTarget(float target_vel_rad_s) {
  return g_joint->RtSetPvTargetVelocity(target_vel_rad_s, kPvAccRadSS,
                                        kPvDecRadSS);
}

void* RtLoop(void*) {
  timespec next_wakeup{};
  __RT(clock_gettime(CLOCK_MONOTONIC, &next_wakeup));

  const long period_ns = g_period_us * 1000L;
  float target_vel_rad_s = kPvSpeedRadS;
  const float init_pos = g_joint->NrtGetActualPosition();
  if (init_pos >= kPvMaxPosRad) {
    target_vel_rad_s = -kPvSpeedRadS;
  } else if (init_pos <= kPvMinPosRad) {
    target_vel_rad_s = kPvSpeedRadS;
  }

  while (g_run_state.load(std::memory_order_acquire) != RunState::kRtExit) {
    next_wakeup = AddNs(next_wakeup, period_ns);
    const int sleep_ret = __RT(
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr));
    if (sleep_ret != 0) {
      g_rt_ret.store(sleep_ret, std::memory_order_release);
      break;
    }

    const int ret = g_bus->RtStepOnce();
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      g_rt_ret.store(ret, std::memory_order_release);
      break;
    }

    const auto current_mode = g_joint->RtGetCurrentMode();
    const auto servo_state = g_joint->RtGetServoState();
    if (servo_state == CanEvoServoState::kFault) {
      g_rt_ret.store(-1000, std::memory_order_release);
      break;
    }

    const bool in_pv = g_joint && current_mode == CanEvoMode::kPv &&
                       servo_state == CanEvoServoState::kRunning;
    if (!in_pv) continue;

    if (g_run_state.load(std::memory_order_acquire) ==
        RunState::kStopRequested) {
      const int set_ret = SendPvTarget(0.0f);
      if (set_ret != static_cast<int>(CanEvoError::kOk)) {
        g_rt_ret.store(set_ret, std::memory_order_release);
        break;
      }
      continue;
    }

    JointStatus status{};
    if (g_joint->RtGetJointStatus(status) ==
        static_cast<int>(CanEvoError::kOk)) {
      if (status.actual_pos >= kPvMaxPosRad) {
        target_vel_rad_s = -kPvSpeedRadS;
      } else if (status.actual_pos <= kPvMinPosRad) {
        target_vel_rad_s = kPvSpeedRadS;
      }
    }

    const int set_ret = SendPvTarget(target_vel_rad_s);
    if (set_ret != static_cast<int>(CanEvoError::kOk)) {
      g_rt_ret.store(set_ret, std::memory_order_release);
      break;
    }
  }

  return nullptr;
}

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
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(3);
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
  PrintJointDiag(joint);
  return false;
}

int main(int argc, char* argv[]) {
  std::cout << "========================================" << std::endl;
  std::cout << "PV 测试程序 - 在 ±30° 内以 10 rpm 轮廓速度往复运动"
            << std::endl;
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

  int sched_policy = SCHED_FIFO;
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
  int cpu = 1;
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
  modi_joint_canevo joint;
  g_bus = &bus;
  g_joint = &joint;

  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 无法打开 CAN 总线" << std::endl;
    pthread_attr_destroy(&attr);
    return -1;
  }

  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "✗ 未扫描到在线关节" << std::endl;
    pthread_attr_destroy(&attr);
    bus.Close();
    return -1;
  }

  std::cout << "✓ 扫描到 " << joint_ids.size() << " 个关节, ID: ";
  for (const auto id : joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  uint8_t node_id = joint_ids.front();
  if (argc >= 2) {
    const int requested_id = std::atoi(argv[1]);
    if (requested_id < 1 || requested_id > 127) {
      std::cerr << "✗ 无效关节 ID: " << argv[1] << std::endl;
      pthread_attr_destroy(&attr);
      bus.Close();
      return -1;
    }

    const auto it = std::find(joint_ids.begin(), joint_ids.end(),
                              static_cast<uint8_t>(requested_id));
    if (it == joint_ids.end()) {
      std::cerr << "✗ 指定关节不在线, ID=" << requested_id << std::endl;
      pthread_attr_destroy(&attr);
      bus.Close();
      return -1;
    }

    node_id = static_cast<uint8_t>(requested_id);
    std::cout << "✓ 指定选择关节 ID: " << requested_id << std::endl;
  } else {
    std::cout << "✓ 默认选择第一个关节 ID: " << static_cast<int>(node_id)
              << std::endl;
  }

  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节初始化失败" << std::endl;
    pthread_attr_destroy(&attr);
    bus.Close();
    return -1;
  }
  std::cout << "✓ Joint 初始化完成 (Node ID: " << static_cast<int>(node_id)
            << ")" << std::endl;

  auto fault_code = joint.NrtGetFaultCode();
  std::cerr << "fault_code: 0x" << std::hex << static_cast<uint16_t>(fault_code)
            << std::dec << std::endl;
  if (fault_code != CanEvoFault::kNone) {
    std::cerr << "检测到故障，先清除故障..." << std::endl;
    PrintJointDiag(joint);
    const int clear_ret = joint.NrtClearFault();
    if (clear_ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 清除故障失败, ret=" << clear_ret << std::endl;
      pthread_attr_destroy(&attr);
      joint.NrtDestroy();
      bus.Close();
      return -1;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  if (joint.NrtEnable(CanEvoMode::kPv) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 使能 PV 模式失败" << std::endl;
    PrintJointDiag(joint);
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitServoState(joint, CanEvoServoState::kRunning,
                      "等待 PV Running 状态")) {
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitControlMode(joint, CanEvoMode::kPv, "等待 PV 模式切换")) {
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  const int init_ret =
      joint.RtSetPvTargetVelocity(0.0f, kPvAccRadSS, kPvDecRadSS);
  if (init_ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 初始 PV PDO 目标发送失败, ret=" << init_ret << std::endl;
    pthread_attr_destroy(&attr);
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  pthread_t rt_thread{};
  const int create_ret =
      __RT(pthread_create(&rt_thread, &attr, RtLoop, nullptr));
  if (create_ret == 0) {
    __RT(pthread_setname_np(rt_thread, "canevo_pv_rt"));
  }
  pthread_attr_destroy(&attr);
  if (create_ret != 0) {
    std::cerr << "✗ 创建实时线程失败, errno=" << create_ret << std::endl;
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  std::cout << "✓ 关节已切换到 PV 模式" << std::endl;
  std::cout << "PV 速度轨迹运行中：±30° 内 10 rpm 往复，按 Ctrl+C 终止..."
            << std::endl;

  const int ok = static_cast<int>(CanEvoError::kOk);
  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (g_run_state.load(std::memory_order_acquire) == RunState::kStopRequested &&
      g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::cout << "收到退出信号，先下发 PV 目标速度 0..." << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  g_run_state.store(RunState::kRtExit, std::memory_order_release);
  __RT(pthread_join(rt_thread, nullptr));
  PrintEndTime();

  const int rt_ret = g_rt_ret.load(std::memory_order_acquire);
  if (rt_ret != static_cast<int>(CanEvoError::kOk)) {
    if (rt_ret == -1000) {
      std::cerr << "实时线程出错: CanEvoServoState::kFault" << std::endl;
    } else {
      std::cerr << "实时线程出错, ret=" << rt_ret << std::endl;
    }
  }

  joint.NrtDisable();
  joint.NrtDestroy();
  bus.Close();
  return 0;
}
