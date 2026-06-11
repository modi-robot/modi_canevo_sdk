/**
 * @file example_rt_thread.cpp
 * @brief 外部实时线程时间戳采样示例
 *
 * 该示例不实例化 SDK、不访问 CAN。实时线程只按固定周期缓存
 * CLOCK_MONOTONIC 纳秒时间戳，运行 30 秒后退出，主线程写 CSV 文件。
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>

#include <algorithm>
#include <atomic>
#include <cstdlib>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>

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

constexpr int kPeriodUs = 5000;
constexpr int kRunSeconds = 30;
constexpr int kRealtimeCpu = 1;
constexpr int kRealtimePriority = 99;
constexpr int kIgnoreInitialSamples = 10;
constexpr int kSampleCapacity =
    (kRunSeconds * 1000000 + kPeriodUs - 1) / kPeriodUs + 16;

struct Sample {
  int64_t planned_ns = 0;
  int64_t wakeup_ns = 0;
  int64_t wakeup_latency_ns = 0;
  int64_t period_jitter_ns = 0;
  int cpu = -1;
};

Sample g_samples[kSampleCapacity];
std::atomic<int> g_sample_count{0};

int64_t ToNs(const timespec& ts) {
  return static_cast<int64_t>(ts.tv_sec) * 1000000000LL + ts.tv_nsec;
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

void* RtLoop(void*) {
  timespec next_wakeup{};
  __RT(clock_gettime(CLOCK_MONOTONIC, &next_wakeup));

  const long period_ns = kPeriodUs * 1000L;
  int64_t last_wakeup_ns = 0;
  int i = 0;
  while (true) {
    // 使用系统绝对时钟sleep，避免时钟飘移
    next_wakeup = AddNs(next_wakeup, period_ns);
    const int sleep_ret =
        __RT(clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup,
                             nullptr));
    if (sleep_ret != 0 || i >= kSampleCapacity) break;

    timespec wakeup{};
    __RT(clock_gettime(CLOCK_MONOTONIC, &wakeup));

    const int64_t planned_ns = ToNs(next_wakeup);
    const int64_t wakeup_ns = ToNs(wakeup);
    g_samples[i].planned_ns = planned_ns;
    g_samples[i].wakeup_ns = wakeup_ns;
    g_samples[i].wakeup_latency_ns = wakeup_ns - planned_ns;
    g_samples[i].period_jitter_ns =
        last_wakeup_ns == 0 ? 0 : wakeup_ns - last_wakeup_ns - period_ns;
    g_samples[i].cpu = sched_getcpu();

    last_wakeup_ns = wakeup_ns;
    i++;
    g_sample_count.store(i, std::memory_order_release);
  }

  return nullptr;
}

bool WriteCsv(const std::string& path, const Sample* samples, int count) {
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) return false;

  out << "time_ns,planned_ns,wakeup_latency_us,period_jitter_us,cpu\n";
  for (int i = 0; i < count; ++i) {
    out << samples[i].wakeup_ns << ',' << samples[i].planned_ns << ','
        << samples[i].wakeup_latency_ns / 1000.0 << ','
        << samples[i].period_jitter_ns / 1000.0 << ',' << samples[i].cpu
        << '\n';
  }

  return true;
}

void PrintStats(const Sample* samples, int count) {
  if (count <= kIgnoreInitialSamples + 1) {
    std::cerr << "样本太少，无法统计" << std::endl;
    return;
  }

  int64_t max_abs_period_jitter_ns = 0;
  int64_t max_wakeup_latency_ns = std::numeric_limits<int64_t>::min();
  int64_t min_wakeup_latency_ns = std::numeric_limits<int64_t>::max();
  long double sum_abs_period_jitter_ns = 0.0;
  long double sum_wakeup_latency_ns = 0.0;
  int over_20us = 0;
  int over_50us = 0;
  int stat_count = 0;

  for (int i = kIgnoreInitialSamples + 1; i < count; ++i) {
    const int64_t abs_period_jitter_ns =
        std::llabs(samples[i].period_jitter_ns);
    max_abs_period_jitter_ns =
        std::max(max_abs_period_jitter_ns, abs_period_jitter_ns);
    min_wakeup_latency_ns =
        std::min(min_wakeup_latency_ns, samples[i].wakeup_latency_ns);
    max_wakeup_latency_ns =
        std::max(max_wakeup_latency_ns, samples[i].wakeup_latency_ns);
    sum_abs_period_jitter_ns += abs_period_jitter_ns;
    sum_wakeup_latency_ns += samples[i].wakeup_latency_ns;
    if (abs_period_jitter_ns > 20000) over_20us++;
    if (abs_period_jitter_ns > 50000) over_50us++;
    stat_count++;
  }

  std::cout << "\n========== 实时线程抖动统计 ==========\n";
  std::cout << "NIIC hard realtime: "
            << (CANEVO_HAVE_NECRO ? "ON (__RT -> qiuniu)" : "OFF (POSIX)")
            << '\n';
  std::cout << "周期: " << kPeriodUs << " us, 运行: " << kRunSeconds
            << " s, 样本: " << count << ", 忽略前 " << kIgnoreInitialSamples
            << " 个样本\n";
  std::cout << "最大绝对周期抖动: "
            << max_abs_period_jitter_ns / 1000.0 << " us\n";
  std::cout << "平均绝对周期抖动: "
            << (sum_abs_period_jitter_ns / stat_count) / 1000.0 << " us\n";
  std::cout << "唤醒延迟 min/max/avg: " << min_wakeup_latency_ns / 1000.0
            << " / " << max_wakeup_latency_ns / 1000.0 << " / "
            << (sum_wakeup_latency_ns / stat_count) / 1000.0 << " us\n";
  std::cout << "|周期抖动| > 20 us: " << over_20us << " 次\n";
  std::cout << "|周期抖动| > 50 us: " << over_50us << " 次\n";
}

}  // namespace

int main() {
  std::cout << "\n========== 示例: 实时线程配置 ==========\n" << std::endl;
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
  attr_ret = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedpolicy 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 4. 线程优先级设为99
  sched_param param{};
  param.sched_priority = kRealtimePriority;
  attr_ret = pthread_attr_setschedparam(&attr, &param);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedparam 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 5. 设置CPU亲和性，把实时线程绑定在隔离内核；当前启动参数隔离的是 CPU1
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(kRealtimeCpu, &cpuset);
  attr_ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setaffinity_np 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }
  std::cout << "实时线程 CPU: " << kRealtimeCpu
            << ", SCHED_FIFO priority: " << kRealtimePriority << std::endl;

  // 6. 创建实时线程，RtLoop为子线程入口
  pthread_t thread{};
  const int create_ret = __RT(pthread_create(&thread, &attr, RtLoop, nullptr));
  if (create_ret == 0) {
    __RT(pthread_setname_np(thread, "canevo_rt_jitter"));
  }
  pthread_attr_destroy(&attr);

  if (create_ret != 0) {
    std::cerr << "创建实时线程失败, errno=" << create_ret << std::endl;
    return -1;
  }

  // 7. 等待实时线程执行完毕
  __RT(pthread_join(thread, nullptr));

  const int sample_count = g_sample_count.load(std::memory_order_acquire);
  PrintStats(g_samples, sample_count);

  // 8. 实时性测试，把子线程循环时间戳打印到CSV中
  const char* csv_path = "rt_thread_timestamps.csv";
  if (!WriteCsv(csv_path, g_samples, sample_count)) {
    std::cerr << "写 CSV 失败: " << csv_path << std::endl;
    return -1;
  }
  std::cout << "CSV 已写入: " << csv_path << std::endl;
  return 0;
}
