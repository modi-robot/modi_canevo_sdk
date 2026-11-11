/**
 * @file example_csp.cpp
 * @brief CSP测试demo - 使用外部实时循环调用 RtStepOnce()
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
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
#include <ctime>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kCspStepRad = 0.005f * kDegToRad;
constexpr float kCspMoveRangeRad = 1.0f * kDegToRad;

// 全局变量用于信号处理
static modi_bus_canevo* g_bus = nullptr;
static modi_joint_canevo* g_joint = nullptr;
static int g_period_us = 5000;
static std::atomic<int> g_rt_ret{0};
static float g_start_pos_rad = 0.0f;
static bool g_motion_enabled = false;

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
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

  const long period_ns = g_period_us * 1000L;
  float target_pos_rad = g_start_pos_rad;
  float step_rad = kCspStepRad;
  const float min_pos_rad = g_start_pos_rad - kCspMoveRangeRad;
  const float max_pos_rad = g_start_pos_rad + kCspMoveRangeRad;

  while (g_run_state.load(std::memory_order_acquire) != RunState::kRtExit) {
    next_wakeup = AddNs(next_wakeup, period_ns);
    const int sleep_ret =
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
    if (sleep_ret != 0) {
      g_rt_ret.store(sleep_ret, std::memory_order_release);
      break;
    }

    const int ret = g_bus->RtStepOnce();
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      g_rt_ret.store(ret, std::memory_order_release);
      break;
    }

    auto current_mode = g_joint->RtGetCurrentMode();
    auto servo_state = g_joint->RtGetServoState();
    if (servo_state == CanEvoServoState::kFault) {
      g_rt_ret.store(-1000, std::memory_order_release);
      break;
    }
    const bool in_csp = g_joint && current_mode == CanEvoMode::kCsp &&
                        servo_state == CanEvoServoState::kRunning;

    if (in_csp) {
      // 默认持续发送当前位置，只有显式传入 --move 才做小幅往复。
      if (g_motion_enabled) {
        target_pos_rad += step_rad;
        if (target_pos_rad >= max_pos_rad) {
          target_pos_rad = max_pos_rad;
          step_rad = -kCspStepRad;
        } else if (target_pos_rad <= min_pos_rad) {
          target_pos_rad = min_pos_rad;
          step_rad = kCspStepRad;
        }
      }

      const int set_ret = g_joint->RtSetCspTargetPosition(target_pos_rad);
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
  // 等待3s
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
  // 等待3秒
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
  if (argc < 3 || argc > 4 ||
      (argc == 4 && std::string(argv[3]) != "--move")) {
    std::cerr << "用法: " << argv[0]
              << " <CAN接口> <节点ID> [--move]\n"
              << "示例（9号节点保持当前位置）: " << argv[0]
              << " can1 9\n"
              << "示例（9号节点在当前位置±1度往复）: " << argv[0]
              << " can1 9 --move" << std::endl;
    return -1;
  }

  const std::string can_interface = argv[1];
  int node_id_value = 0;
  try {
    std::size_t parsed = 0;
    node_id_value = std::stoi(argv[2], &parsed);
    if (parsed != std::string(argv[2]).size() || node_id_value < 1 ||
        node_id_value > 62) {
      throw std::invalid_argument("节点ID超出范围");
    }
  } catch (const std::exception&) {
    std::cerr << "节点ID无效: " << argv[2] << "（有效范围 1~62）"
              << std::endl;
    return -1;
  }
  const uint8_t node_id = static_cast<uint8_t>(node_id_value);
  g_motion_enabled = argc == 4;

  std::cout << "========================================" << std::endl;
  std::cout << "CSP 单关节测试程序" << std::endl;
  std::cout << "CAN接口: " << can_interface
            << ", 节点ID: " << node_id_value
            << ", 动作: " << (g_motion_enabled ? "当前位置±1度往复" : "保持当前位置")
            << std::endl;
  std::cout << "========================================" << std::endl;

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

  // 4. 线程优先级设为99
  sched_param param{};
  param.sched_priority = 99;
  attr_ret = pthread_attr_setschedparam(&attr, &param);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedparam 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 5. 设置CPU亲和性，把实时线程绑定在隔离内核；当前启动参数隔离的是 CPU1
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

  // 6. 配置SDK内部状态更新线程
  TaskConfig task_config;
  task_config.sync_period_us = 5000;  // 5000 us = 5 ms, 下发给关节
  // 不要和实时线程邦到同一个cpu上
  task_config.cpu_affinity = 3;
  task_config.sched_policy = sched_policy;
  // 如果与实时线程绑在同一个CPU上，优先级建议比实时线程底一些，防止与实时线程发生竞争
  task_config.sched_priority = 90;
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
  if (bus.Open(can_interface, task_config) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 无法打开 CAN 总线: " << can_interface << std::endl;
    return -1;
  }
  // 9. 配置超时
  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  // 10. 直接初始化命令行指定的关节，不扫描或选择其他节点。
  if (joint.NrtInit(bus, node_id) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 关节初始化失败" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "✓ Joint 初始化完成 (Node ID: " << static_cast<int>(node_id)
            << ")" << std::endl;

  // CSP 的首帧目标必须从反馈位置开始，避免使能瞬间产生位置跳变。
  g_start_pos_rad = joint.NrtGetActualPosition();
  std::cout << "✓ 初始位置: " << g_start_pos_rad / kDegToRad << " deg"
            << std::endl;

  // 12. 如果有故障就先清除故障。
  auto fault_code = joint.NrtGetFaultCode();
  std::cerr << "fault_code: 0x" << std::hex
            << static_cast<uint16_t>(fault_code) << std::dec << std::endl;
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

  // 13. 使能关节并切换到 CSP
  if (joint.NrtEnable(CanEvoMode::kCsp) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 使能 CSP 模式失败" << std::endl;
    PrintJointDiag(joint);
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  // 先发送当前位置目标，再发第一帧 SYNC。这样 CSP 切换生效时不会使用
  // 未初始化或遗留的位置目标。
  const int prefill_ret = joint.RtSetCspTargetPosition(g_start_pos_rad);
  if (prefill_ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 预填 CSP 初始目标失败, ret=" << prefill_ret << std::endl;
    joint.NrtDisable();
    pthread_attr_destroy(&attr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  // NrtEnable 返回后立即启动 5 ms 周期。模式切换阶段也必须持续发送
  // SYNC，不能等到 CSP/Running 已经确认后才启动周期线程。
  pthread_t rt_thread{};
  const int create_ret = pthread_create(&rt_thread, &attr, RtLoop, nullptr);
  if (create_ret == 0) {
    pthread_setname_np(rt_thread, "canevo_csp_rt");
  }
  pthread_attr_destroy(&attr);
  if (create_ret != 0) {
    std::cerr << "✗ 创建实时线程失败, errno=" << create_ret << std::endl;
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  if (!WaitServoState(joint, CanEvoServoState::kRunning,
                      "等待 CSP Running 状态")) {
    joint.NrtDisable();
    g_run_state.store(RunState::kRtExit, std::memory_order_release);
    pthread_join(rt_thread, nullptr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  if (!WaitControlMode(joint, CanEvoMode::kCsp, "等待 CSP 模式切换")) {
    joint.NrtDisable();
    g_run_state.store(RunState::kRtExit, std::memory_order_release);
    pthread_join(rt_thread, nullptr);
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  std::cout << "✓ 关节已切换到 CSP 模式" << std::endl;
  if (g_motion_enabled) {
    std::cout << "CSP 轨迹运行中：以初始位置为中心±1 deg，速度约1 deg/s；"
                 "按 Ctrl+C 终止..."
              << std::endl;
  } else {
    std::cout << "CSP 保持当前位置中；按 Ctrl+C 终止..." << std::endl;
  }

  const int ok = static_cast<int>(CanEvoError::kOk);
  while (g_run_state.load(std::memory_order_acquire) == RunState::kRunning &&
         g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }

  if (g_run_state.load(std::memory_order_acquire) == RunState::kStopRequested &&
      g_rt_ret.load(std::memory_order_acquire) == ok) {
    std::cout << "收到退出信号，先失能关节并保持 CSP 周期帧收尾..."
              << std::endl;
  } else {
    std::cerr << "周期线程异常，先失能关节再退出..." << std::endl;
  }

  // 无论是 Ctrl+C 还是实时线程异常，都在周期线程仍运行时先失能。
  const int disable_ret = joint.NrtDisable();
  if (disable_ret != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "✗ 失能失败, ret=" << disable_ret << std::endl;
    PrintJointDiag(joint);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));

  g_run_state.store(RunState::kRtExit, std::memory_order_release);
  pthread_join(rt_thread, nullptr);
  PrintEndTime();

  const int rt_ret = g_rt_ret.load(std::memory_order_acquire);
  if (rt_ret != static_cast<int>(CanEvoError::kOk)) {
    if (rt_ret == -1000) {
      std::cerr << "实时线程出错: CanEvoServoState::kFault" << std::endl;
    } else {
      std::cerr << "实时线程出错, ret=" << rt_ret << std::endl;
    }
  }

  joint.NrtDestroy();
  bus.Close();
  return 0;
}
