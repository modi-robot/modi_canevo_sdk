/**
 * @file example_cst.cpp
 * @brief 多关节 CST 电流模式测试 - 外部实时线程调用 RtStepOnce()
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 txqueuelen 1
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 *
 * 测试内容：
 * 1. 使能 CST 模式
 * 2. 全部关节持续下发 0 A 目标电流，按 Ctrl+C 退出
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <iomanip>
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

constexpr float kTargetCurrentA = 0.7f;

static modi_bus_canevo* g_bus = nullptr;
static std::vector<modi_joint_canevo>* g_joints = nullptr;
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

    for (size_t i = 0; i < g_joints->size(); ++i) {
      auto& joint = (*g_joints)[i];
      const auto current_mode = joint.RtGetCurrentMode();
      const auto servo_state = joint.RtGetServoState();
      if (servo_state == CanEvoServoState::kFault) {
        g_rt_ret.store(-1000, std::memory_order_release);
        break;
      }
      const bool in_cst = current_mode == CanEvoMode::kCst &&
                          servo_state == CanEvoServoState::kRunning;
      if (!in_cst) continue;

      const int set_ret = joint.RtSetCstTargetCurrent(kTargetCurrentA);
      if (set_ret != static_cast<int>(CanEvoError::kOk)) {
        g_rt_ret.store(set_ret, std::memory_order_release);
        break;
      }
    }

    if (g_rt_ret.load(std::memory_order_acquire) !=
        static_cast<int>(CanEvoError::kOk)) {
      break;
    }
  }

  return nullptr;
}

void SignalHandler(int signum) {
  (void)signum;
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
  return false;
}

int main() {
  std::cout << "========================================" << std::endl;
  std::cout << "CST 测试程序 - 多关节持续下发 0 A 目标电流" << std::endl;
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
  std::vector<modi_joint_canevo> joints;
  g_bus = &bus;
  g_joints = &joints;
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

  std::vector<modi_joint_canevo> tmp_joints(joint_ids.size());
  joints.swap(tmp_joints);

  // 11. 初始化全部关节
  for (size_t i = 0; i < joint_ids.size(); ++i) {
    if (joints[i].NrtInit(bus, joint_ids[i]) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 关节初始化失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      for (size_t j = 0; j < i; ++j) {
        joints[j].NrtDestroy();
      }
      bus.Close();
      return -1;
    }
    std::cout << "✓ Joint 初始化完成 (Node ID: "
              << static_cast<int>(joint_ids[i]) << ")" << std::endl;
  }

  // 12. 如果有故障就先清除故障
  for (size_t i = 0; i < joints.size(); ++i) {
    const auto fault_code = joints[i].NrtGetFaultCode();
    std::cerr << "joint " << static_cast<int>(joint_ids[i]) << " fault_code: 0x"
              << std::hex << static_cast<uint16_t>(fault_code) << std::dec
              << std::endl;
    if (fault_code != CanEvoFault::kNone) {
      std::cerr << "检测到故障，先清除故障..." << std::endl;
      PrintJointDiag(joints[i], joint_ids[i]);
      const int clear_ret = joints[i].NrtClearFault();
      if (clear_ret != static_cast<int>(CanEvoError::kOk)) {
        std::cerr << "✗ 清除故障失败, ID=" << static_cast<int>(joint_ids[i])
                  << ", ret=" << clear_ret << std::endl;
        pthread_attr_destroy(&attr);
        for (auto& joint : joints) {
          joint.NrtDestroy();
        }
        bus.Close();
        return -1;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }

  // 13. 使能全部关节并切换到 CST
  for (size_t i = 0; i < joints.size(); ++i) {
    if (joints[i].NrtEnable(CanEvoMode::kCst) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "✗ 使能 CST 模式失败, ID=" << static_cast<int>(joint_ids[i])
                << std::endl;
      PrintJointDiag(joints[i], joint_ids[i]);
      pthread_attr_destroy(&attr);
      for (auto& joint : joints) {
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  for (size_t i = 0; i < joints.size(); ++i) {
    if (!WaitServoState(joints[i], CanEvoServoState::kRunning,
                        "等待 CST Running 状态")) {
      PrintJointDiag(joints[i], joint_ids[i]);
      pthread_attr_destroy(&attr);
      for (auto& joint : joints) {
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
    if (!WaitControlMode(joints[i], CanEvoMode::kCst, "等待 CST 模式切换")) {
      PrintJointDiag(joints[i], joint_ids[i]);
      pthread_attr_destroy(&attr);
      for (auto& joint : joints) {
        joint.NrtDestroy();
      }
      bus.Close();
      return -1;
    }
  }

  // 14. CST Running 后再创建实时线程
  pthread_t rt_thread{};
  const int create_ret =
      __RT(pthread_create(&rt_thread, &attr, RtLoop, nullptr));
  if (create_ret == 0) {
    __RT(pthread_setname_np(rt_thread, "canevo_multi_cst"));
  }
  pthread_attr_destroy(&attr);
  if (create_ret != 0) {
    std::cerr << "✗ 创建实时线程失败, errno=" << create_ret << std::endl;
    for (auto& joint : joints) {
      joint.NrtDisable();
      joint.NrtDestroy();
    }
    bus.Close();
    return -1;
  }

  std::cout << "✓ 全部关节已切换到 CST 模式" << std::endl;
  std::cout << "CST 电流控制运行中：目标 " << kTargetCurrentA
            << " A，按 Ctrl+C 终止..." << std::endl;

  const int ok = static_cast<int>(CanEvoError::kOk);
  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (g_run_state.load(std::memory_order_acquire) == RunState::kStopRequested &&
      g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::cout << "收到退出信号，先失能关节并保持 CST 周期帧收尾..."
              << std::endl;
    for (size_t i = 0; i < joints.size(); ++i) {
      const int disable_ret = joints[i].NrtDisable();
      if (disable_ret != static_cast<int>(CanEvoError::kOk)) {
        std::cerr << "✗ 失能失败, ID=" << static_cast<int>(joint_ids[i])
                  << ", ret=" << disable_ret << std::endl;
        PrintJointDiag(joints[i], joint_ids[i]);
      }
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

  for (auto& joint : joints) {
    joint.NrtDisable();
    joint.NrtDestroy();
  }
  bus.Close();
  return 0;
}
