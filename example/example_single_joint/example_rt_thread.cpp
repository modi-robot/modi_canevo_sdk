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

#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>

namespace {

constexpr int kPeriodUs = 5000;
constexpr int kRunSeconds = 30;
constexpr int kSampleCapacity =
    (kRunSeconds * 1000000 + kPeriodUs - 1) / kPeriodUs + 16;

int64_t g_timestamps_ns[kSampleCapacity];

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
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);

  const long period_ns = kPeriodUs * 1000L;
  int i = 0;
  while (true) {
    // 使用系统绝对时钟sleep，避免时钟飘移
    next_wakeup = AddNs(next_wakeup, period_ns);
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
    // 用户处理实时任务，当前任务是统计时钟抖动，运行30s自动结束
    {
      if (i >= kSampleCapacity) break;
      timespec wakeup{};
      clock_gettime(CLOCK_MONOTONIC, &wakeup);
      g_timestamps_ns[i] = ToNs(wakeup);
      i++;
    }
  }

  return nullptr;
}

bool WriteCsv(const std::string& path, const int64_t* timestamps, int count) {
  std::ofstream out(path, std::ios::out | std::ios::trunc);
  if (!out) return false;

  out << "time_ns\n";
  for (int i = 0; i < count; ++i) {
    out << timestamps[i] << '\n';
  }

  return true;
}

}  // namespace

int main() {
  std::cout << "\n========== 示例: 实时线程配置 ==========\n" << std::endl;
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
  param.sched_priority = 99;
  attr_ret = pthread_attr_setschedparam(&attr, &param);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setschedparam 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 5. 设置CPU亲和性，把实时线程绑定在隔离内核，我这里隔离的是2号CPU
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(2, &cpuset);
  attr_ret = pthread_attr_setaffinity_np(&attr, sizeof(cpuset), &cpuset);
  if (attr_ret != 0) {
    std::cerr << "警告: pthread_attr_setaffinity_np 失败, errno=" << attr_ret
              << std::endl;
    return -1;
  }

  // 6. 创建实时线程，RtLoop为子线程入口
  pthread_t thread{};
  const int create_ret = pthread_create(&thread, &attr, RtLoop, nullptr);
  pthread_attr_destroy(&attr);

  if (create_ret != 0) {
    std::cerr << "创建实时线程失败, errno=" << create_ret << std::endl;
    return -1;
  }

  // 7. 等待实时线程执行完毕
  pthread_join(thread, nullptr);

  // 8. 实时性测试，把子线程循环时间戳打印到CSV中
  const char* csv_path = "rt_thread_timestamps.csv";
  if (!WriteCsv(csv_path, g_timestamps_ns, kSampleCapacity)) {
    std::cerr << "写 CSV 失败: " << csv_path << std::endl;
    return -1;
  }
  std::cout << "CSV 已写入: " << csv_path << std::endl;
  return 0;
}
