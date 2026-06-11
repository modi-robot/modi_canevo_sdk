/**
 * @file example_cst.cpp
 * @brief CST电流模式测试 - 外部实时线程调用 RtStepOnce()
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <atomic>
#include <iostream>

#include "modi_joint_canevo.h"

namespace {

constexpr int kPeriodUs = 5000;
constexpr int kCpuAffinity = 2;
constexpr int kRealtimePriority = 99;
constexpr int kTotalCycles = 3 * 1000000 / kPeriodUs;
constexpr float kTargetCurrentA = 0.0f;

modi_bus_canevo* g_bus = nullptr;
modi_joint_canevo* g_joint = nullptr;
std::atomic<int> g_rt_ret{0};

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

void* CstRtLoop(void*) {
  timespec next_wakeup{};
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
  const long period_ns = kPeriodUs * 1000L;

  int cycle = 0;
  while (true) {
    if (cycle >= kTotalCycles) break;

    const int ret = g_bus->RtStepOnce();
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      g_rt_ret.store(ret, std::memory_order_release);
      break;
    }
    const int set_ret = g_joint->RtSetCstTargetCurrent(kTargetCurrentA);
    if (set_ret != static_cast<int>(CanEvoError::kOk)) {
      g_rt_ret.store(set_ret, std::memory_order_release);
      break;
    }
    ++cycle;

    next_wakeup = AddNs(next_wakeup, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
  }

  return nullptr;
}

int CreateRealtimeThread(pthread_t* thread, void* (*entry)(void*)) {
  pthread_attr_t attr;
  pthread_attr_init(&attr);

  int ret = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (ret != 0) return ret;
  ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (ret != 0) return ret;

  sched_param param{};
  param.sched_priority = kRealtimePriority;
  ret = pthread_attr_setschedparam(&attr, &param);
  if (ret != 0) return ret;

  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(kCpuAffinity, &cpuset);
  ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
  if (ret != 0) return ret;

  ret = pthread_create(thread, &attr, entry, nullptr);
  pthread_attr_destroy(&attr);
  return ret;
}

}  // namespace

int main() {
  TaskConfig task_config;
  task_config.sync_period_us = kPeriodUs;
  task_config.cpu_affinity = kCpuAffinity;

  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "mlockall 失败" << std::endl;
    return -1;
  }

  modi_bus_canevo bus;
  modi_joint_canevo joint;
  g_bus = &bus;
  g_joint = &joint;

  if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 总线失败" << std::endl;
    return -1;
  }

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
  const uint8_t node_id = joint_ids.front();
  std::cout << "默认选择第一个关节 ID: " << static_cast<int>(node_id)
            << std::endl;

  if (joint.NrtInit(bus, node_id) !=
      static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "初始化关节失败" << std::endl;
    bus.Close();
    return -1;
  }

  if (joint.NrtEnable(CanEvoMode::kCst) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "使能 CST 模式失败" << std::endl;
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }

  pthread_t rt_thread{};
  const int create_ret = CreateRealtimeThread(&rt_thread, CstRtLoop);
  if (create_ret != 0) {
    std::cerr << "创建实时线程失败, errno=" << create_ret << std::endl;
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    return -1;
  }
  pthread_join(rt_thread, nullptr);

  if (g_rt_ret.load(std::memory_order_acquire) != 0) {
    std::cerr << "RtStepOnce 失败, ret=" << g_rt_ret.load() << std::endl;
  }

  joint.NrtDisable();
  joint.NrtDestroy();
  bus.Close();
  return 0;
}
