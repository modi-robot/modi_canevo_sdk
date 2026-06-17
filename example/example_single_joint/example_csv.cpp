/**
 * @file example_csv.cpp
 * @brief CSV 速度模式测试 - 外部实时线程调用 RtStepOnce()
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 txqueuelen 1
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 *
 * 测试内容：
 * 1. 使能 CSV 模式
 * 2. 在 -90°～+90° 范围内以 10°/s 往复运动
 * 3. 到达 -90° 时切换为 +10°/s，到达 +90° 时切换为 -10°/s
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
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
constexpr float kCsvSpeedRadS = 10.0f * kDegToRad;
constexpr float kCsvMinPosRad = -90.0f * kDegToRad;
constexpr float kCsvMaxPosRad = 90.0f * kDegToRad;

// 全局变量用于信号处理
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

void* RtLoop(void*) {
  timespec next_wakeup{};
  __RT(clock_gettime(CLOCK_MONOTONIC, &next_wakeup));

  const long period_ns = g_period_us * 1000L;
  float vel_rad_s = kCsvSpeedRadS;
  const float init_pos = g_joint->NrtGetActualPosition();
  if (init_pos >= kCsvMaxPosRad) {
    vel_rad_s = -kCsvSpeedRadS;
  } else if (init_pos <= kCsvMinPosRad) {
    vel_rad_s = kCsvSpeedRadS;
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
    const bool in_csv = g_joint && current_mode == CanEvoMode::kCsv &&
                        servo_state == CanEvoServoState::kRunning;
    if (in_csv) {
      JointStatus status{};
      g_joint->RtGetJointStatus(status);
      if (status.actual_pos >= kCsvMaxPosRad) {
        vel_rad_s = -kCsvSpeedRadS;
      } else if (status.actual_pos <= kCsvMinPosRad) {
        vel_rad_s = kCsvSpeedRadS;
      }
      const int set_ret = g_joint->RtSetCsvTargetVelocity(vel_rad_s);
      if (set_ret != static_cast<int>(CanEvoError::kOk)) {
        g_rt_ret.store(set_ret, std::memory_order_release);
        break;
      }
    }
  }

  return nullptr;
}

// 信号处理函数
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
  // 等待 3s
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
  // 等待 3 秒
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

int main() {
  std::cout << "========================================" << std::endl;
  std::cout << "CSV 测试程序 - 在 ±90° 内以 10°/s 往复运动" << std::endl;
  std::cout << "========================================" << std::endl;
  qiuniu_init();
  std::cout << "NIIC hard realtime: "
            << (CANEVO_HAVE_NECRO ? "ON (__RT -> qiuniu)" : "OFF (POSIX)")
            << std::endl;

  // 1. 禁止内存交换
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "警告: mlockall 失败，实时抖动可能增大" << std::endl;
    return -1;
  }

  pthread_attr_t attr;
  pthread_attr_init(&attr);

  // 2. 不继承主线程属性
  int attr_ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setinheritsched 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 3. 线程调度策略使用 SCHED_FIFO
  int sched_policy = SCHED_FIFO;
  attr_ret = pthread_attr_setschedpolicy(&attr, sched_policy);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedpolicy 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 4. 线程优先级设为 99
  sched_param param{};
  param.sched_priority = 99;
  attr_ret = pthread_attr_setschedparam(&attr, &param);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedparam 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 5. 设置 CPU 亲和性，把实时线程绑定在隔离内核；当前启动参数隔离的是 CPU1
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  int cpu = 1;
  CPU_SET(cpu, &cpuset);
  attr_ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setaffinity_np 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }
  std::cout << "✓ 实时线程绑定 CPU" << cpu << std::endl;

  // 6. 配置 SDK 内部状态更新线程
  TaskConfig task_config;
  task_config.sync_period_us = 5000;  // 5000 us = 5 ms, 下发给关节
  // 不要和实时线程绑到同一个 cpu 上
  task_config.cpu_affinity = 3;
  task_config.sched_policy = sched_policy;
  // 如果与实时线程绑在同一个 CPU 上，优先级建议比实时线程低一些
  task_config.sched_priority = 90;
  g_period_us = task_config.sync_period_us;
  std::cout << "✓ SDK Rx 线程绑定 CPU" << task_config.cpu_affinity
            << ", 优先级 " << task_config.sched_priority << std::endl;

  // 7. 创建总线和关节对象
  modi_bus_canevo bus;
  modi_joint_canevo joint;
  // 设置全局指针用于信号处理
  g_bus = &bus;
  g_joint = &joint;
  // 注册信号处理函数
  std::signal(SIGINT, SignalHandler);   // Ctrl+C
  std::signal(SIGTERM, SignalHandler);  // kill 命令

  // 8. 打开已手动配置好的 CAN 总线（内部只启动 Rx 线程）
  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 无法打开 CAN 总线" << std::endl;
    return -1;
  }
  // 9. 配置超时
  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  // 10. 扫描 can 总线上的节点，若已知节点号可以跳过此步
  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "✗ 未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "✓ 扫描到 " << joint_ids.size() << " 个关节, ID: ";
  for (const auto id : joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;
  const uint8_t node_id = joint_ids.front();
  std::cout << "✓ 默认选择第一个关节 ID: " << static_cast<int>(node_id)
            << std::endl;

  // 11. 初始化关节
  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节初始化失败" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "✓ Joint 初始化完成 (Node ID: " << static_cast<int>(node_id)
            << ")" << std::endl;

  // 12. 如果有故障就先清除故障
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

  // 13. 使能关节并切换到 CSV
  if (joint.NrtEnable(CanEvoMode::kCsv) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 使能 CSV 模式失败" << std::endl;
    PrintJointDiag(joint);
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitServoState(joint, CanEvoServoState::kRunning,
                      "等待 CSV Running 状态")) {
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitControlMode(joint, CanEvoMode::kCsv, "等待 CSV 模式切换")) {
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  // 14. CSV Running 后再创建实时线程
  pthread_t rt_thread{};
  const int create_ret =
      __RT(pthread_create(&rt_thread, &attr, RtLoop, nullptr));
  if (create_ret == 0) {
    __RT(pthread_setname_np(rt_thread, "canevo_csv_rt"));
  }
  pthread_attr_destroy(&attr);
  if (create_ret != 0) {
    std::cerr << "✗ 创建实时线程失败, errno=" << create_ret << std::endl;
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  std::cout << "✓ 关节已切换到 CSV 模式" << std::endl;
  std::cout << "CSV 速度轨迹运行中：±90° 内 10°/s 往复，按 Ctrl+C 终止..."
            << std::endl;

  const int ok = static_cast<int>(CanEvoError::kOk);
  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (g_run_state.load(std::memory_order_acquire) == RunState::kStopRequested &&
      g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::cout << "收到退出信号，先失能关节并保持 CSV 周期帧收尾..."
              << std::endl;
    const int disable_ret = joint.NrtDisable();
    if (disable_ret != static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 失能失败, ret=" << disable_ret << std::endl;
      PrintJointDiag(joint);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
