/**
 * @file    canevo_impl.cpp
 * @brief   modi_bus_canevo::Impl / modi_joint_canevo::Impl 核心实现
 *
 * @details
 * SocketCAN CAN-FD 收发、Rx/Tx 线程、协议编解码、SDO 串行化。
 *
 * @protocol CanEvo V1.2.3
 * @version  1.0
 * @date     2026-02-26
 */

#include "canevo_impl.h"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <pthread.h>
#include <sched.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>

using namespace canevo;

/* ---- 单位转换常量 ---- */
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kRpmToRads = (2.0f * static_cast<float>(M_PI)) / 60.0f;

/* ============================================================
 *              modi_bus_canevo::Impl 协议 / SocketCAN 辅助
 * ============================================================ */

/** @brief DLC -> 实际数据字节长度 */
uint8_t modi_bus_canevo::Impl::DlcToLen(uint8_t dlc) const {
  constexpr uint8_t kMap[16] = {0, 1,  2,  3,  4,  5,  6,  7,
                                8, 12, 16, 20, 24, 32, 48, 64};
  return (dlc < 16) ? kMap[dlc] : 64;
}

/** @brief 实际数据字节长度 -> 最小合适 DLC */
uint8_t modi_bus_canevo::Impl::LenToDlc(uint8_t len) const {
  if (len <= 8) return len;
  if (len <= 12) return 9;
  if (len <= 16) return 10;
  if (len <= 20) return 11;
  if (len <= 24) return 12;
  if (len <= 32) return 13;
  if (len <= 48) return 14;
  return 15;
}

/** @brief 创建 SocketCAN FD socket 并绑定到接口 */
int modi_bus_canevo::Impl::CreateCanFdSocket(const std::string& ifname) {
  int fd = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (fd < 0) return -1;

  /* 启用 CAN-FD */
  int enable_fd = 1;
  if (::setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_fd,
                   sizeof(enable_fd)) < 0) {
    ::close(fd);
    return -1;
  }

  struct ifreq ifr {};
  std::strncpy(ifr.ifr_name, ifname.c_str(), IFNAMSIZ - 1);
  if (::ioctl(fd, SIOCGIFINDEX, &ifr) < 0) {
    ::close(fd);
    return -1;
  }

  struct sockaddr_can addr {};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(fd, reinterpret_cast<struct sockaddr*>(&addr), sizeof(addr)) < 0) {
    ::close(fd);
    return -1;
  }

  return fd;
}

/** @brief 阻塞发送一帧 CANFD，返回 0 成功 / -1 失败 */
int modi_bus_canevo::Impl::SendFrame(int fd, const CanFrame& f) {
  struct canfd_frame cf {};
  cf.can_id = f.id;
  cf.len = f.len;
  cf.flags = CANFD_BRS; /* BRS always on for CAN-FD */
  std::memcpy(cf.data, f.data, f.len);

  ssize_t nbytes = ::write(fd, &cf, sizeof(cf));
  return (nbytes == static_cast<ssize_t>(sizeof(cf))) ? 0 : -1;
}

/* ============================================================
 *              modi_bus_canevo::Impl 实现
 * ============================================================ */

modi_bus_canevo::Impl::Impl() = default;

modi_bus_canevo::Impl::~Impl() { Close(); }

int modi_bus_canevo::Impl::Open(const std::string& ifname,
                                const TaskConfig& task_config) {
  if (sock_fd_ >= 0) return static_cast<int>(CanEvoError::kOk);  // 已打开

  ifname_ = ifname;
  task_config_ = task_config;  // 保存配置

  /* 主 socket（Rx + SDO Tx 等） */
  sock_fd_ = CreateCanFdSocket(ifname);
  if (sock_fd_ < 0) return static_cast<int>(CanEvoError::kSendFailed);

  /* SYNC 专用 socket */
  sync_fd_ = CreateCanFdSocket(ifname);
  if (sync_fd_ < 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
    return static_cast<int>(CanEvoError::kSendFailed);
  }

  /* 启动 Rx 线程 */
  rx_running_.store(true, std::memory_order_release);
  rx_thread_ = std::thread(&modi_bus_canevo::Impl::RxLoop, this);

  // 配置 Rx 线程为实时优先级（使用 TaskConfig）
  struct sched_param rx_param;
  rx_param.sched_priority = task_config_.priority;
  if (pthread_setschedparam(rx_thread_.native_handle(), SCHED_FIFO,
                            &rx_param) != 0) {
    std::cerr << "警告: 无法设置 RX 线程实时优先级 (errno=" << errno << ")"
              << std::endl;
  }

  // 配置 Rx 线程 CPU 亲和性（使用 TaskConfig）
  cpu_set_t rx_cpuset;
  CPU_ZERO(&rx_cpuset);
  CPU_SET(task_config_.cpu_affinity, &rx_cpuset);
  if (pthread_setaffinity_np(rx_thread_.native_handle(), sizeof(rx_cpuset),
                             &rx_cpuset) != 0) {
    std::cerr << "警告: 无法设置 RX 线程 CPU 亲和性 (errno=" << errno << ")"
              << std::endl;
  }

  /* 锁定内存（防止页面交换） */
  if (mlockall(MCL_CURRENT | MCL_FUTURE) != 0) {
    std::cerr << "警告: 无法锁定内存 (errno=" << errno << ")" << std::endl;
  }

  return static_cast<int>(CanEvoError::kOk);
}

void modi_bus_canevo::Impl::Close() {
  /* 停止控制循环 */
  if (ctrl_running_.load(std::memory_order_acquire)) {
    ctrl_running_.store(false, std::memory_order_release);
    if (ctrl_thread_.joinable()) ctrl_thread_.join();
  }

  /* 停止 Rx */
  if (rx_running_.load(std::memory_order_acquire)) {
    rx_running_.store(false, std::memory_order_release);
    if (rx_thread_.joinable()) rx_thread_.join();
  }

  if (sync_fd_ >= 0) {
    ::close(sync_fd_);
    sync_fd_ = -1;
  }
  if (sock_fd_ >= 0) {
    ::close(sock_fd_);
    sock_fd_ = -1;
  }

  /* 清空注册表 */
  {
    std::lock_guard<std::mutex> lk(map_mu_);
    joints_.clear();
  }
}

int modi_bus_canevo::Impl::Send(const CanFrame& f) {
  if (sock_fd_ < 0) return static_cast<int>(CanEvoError::kBusNotOpen);
  std::lock_guard<std::mutex> lk(tx_mu_);
  return (SendFrame(sock_fd_, f) == 0)
             ? static_cast<int>(CanEvoError::kOk)
             : static_cast<int>(CanEvoError::kSendFailed);
}

int modi_bus_canevo::Impl::SendSync(uint8_t counter) {
  if (sync_fd_ < 0) return static_cast<int>(CanEvoError::kBusNotOpen);

  CanFrame f;
  f.id = kCobSync;
  f.len = 2;  // 协议规定：DATA0(0x3C) + DATA1(counter)
  f.dlc = LenToDlc(f.len);
  f.data[0] = 0x3C;  // 协议固定值
  f.data[1] = counter;  // 循环同步计数器（0~255递增）

  return (SendFrame(sync_fd_, f) == 0)
             ? static_cast<int>(CanEvoError::kOk)
             : static_cast<int>(CanEvoError::kSendFailed);
}

/* ---- 控制循环线程接口 ---- */
int modi_bus_canevo::Impl::StartControlLoop(ControlLoopCallback callback) {
  if (sock_fd_ < 0) return static_cast<int>(CanEvoError::kBusNotOpen);

  // 检查是否已有控制循环在运行
  if (ctrl_running_.load(std::memory_order_acquire)) {
    return static_cast<int>(CanEvoError::kInvalidParam);  // 已在运行
  }

  // 保存回调函数（周期等配置已在 Open() 时保存到 task_config_）
  ctrl_callback_ = callback;

  // 启动控制线程（优先级和CPU亲和性在线程内部设置）
  ctrl_running_.store(true, std::memory_order_release);
  ctrl_thread_ = std::thread(&modi_bus_canevo::Impl::ControlLoop, this);

  return static_cast<int>(CanEvoError::kOk);
}

int modi_bus_canevo::Impl::StopControlLoop() {
  if (!ctrl_running_.load(std::memory_order_acquire)) {
    return static_cast<int>(CanEvoError::kOk);  // 未在运行
  }

  ctrl_running_.store(false, std::memory_order_release);
  if (ctrl_thread_.joinable()) {
    ctrl_thread_.join();
  }

  return static_cast<int>(CanEvoError::kOk);
}

void modi_bus_canevo::Impl::Join() {
  if (ctrl_thread_.joinable()) {
    ctrl_thread_.join();
  }
}

/* ---- 控制循环线程函数 ---- */

void modi_bus_canevo::Impl::ControlLoop() {
  std::cerr << "[控制循环] 线程已启动，周期: " << task_config_.period_us
            << " us" << std::endl;
  // 1. 设置线程优先级和 CPU 亲和性（使用 TaskConfig）
  struct sched_param param;
  param.sched_priority = task_config_.priority;
  if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &param) != 0) {
    std::cerr << "警告: 无法设置控制循环线程实时优先级 (errno=" << errno << ")"
              << std::endl;
  }

  // 设置 CPU 亲和性
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  CPU_SET(task_config_.cpu_affinity, &cpuset);
  if (pthread_setaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) != 0) {
    std::cerr << "警告: 无法设置控制循环线程 CPU 亲和性 (errno=" << errno << ")"
              << std::endl;
  }

  // 2. 主控制循环
  uint8_t sync_counter = 0;
  struct timespec next_wakeup;
  clock_gettime(CLOCK_MONOTONIC, &next_wakeup);
  const long period_ns = task_config_.period_us * 1000L;  // us -> ns

  while (ctrl_running_.load(std::memory_order_acquire)) {
    // 先计算下一次目标唤醒时间
    next_wakeup = CalWaitClock(next_wakeup, period_ns);

    // 发送 SYNC
    SendSync(sync_counter++);  // 0-255循环,自动溢出：255→0
    if (ctrl_callback_) {
      try {
        ctrl_callback_();
      } catch (const std::exception& e) {
        std::cerr << "[控制循环] 回调函数异常: " << e.what() << std::endl;
      } catch (...) {
        std::cerr << "[控制循环] 回调函数发生未知异常" << std::endl;
      }
    }

    // 获取当前时间，判断是否超时
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);

    // 比较当前时间和目标唤醒时间
    if (now.tv_sec < next_wakeup.tv_sec ||
        (now.tv_sec == next_wakeup.tv_sec &&
         now.tv_nsec < next_wakeup.tv_nsec)) {
      // 正常情况：还没到目标时间，睡眠到目标时间点
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wakeup, nullptr);
    } else {
      // 超时情况：当前时间已超过目标时间
      long overrun_ns = (now.tv_sec - next_wakeup.tv_sec) * 1000000000L +
                        (now.tv_nsec - next_wakeup.tv_nsec);
      std::cerr << "[控制循环] 警告: 周期超时 " << (overrun_ns / 1000.0)
                << " us (周期=" << task_config_.period_us << " us)"
                << std::endl;

      // 重新同步到当前时间（下次循环开始会自动加 period_ns）
      next_wakeup = now;
    }
  }

  std::cerr << "[控制循环] 线程已停止" << std::endl;
}

struct timespec modi_bus_canevo::Impl::CalWaitClock(
    const struct timespec& current, long period_ns) const {
  struct timespec target = current;
  target.tv_nsec += period_ns;

  // 处理纳秒溢出
  if (target.tv_nsec >= 1000000000L) {
    target.tv_sec += 1;
    target.tv_nsec -= 1000000000L;
  }

  return target;
}

void modi_bus_canevo::Impl::RegisterJoint(uint8_t node_id,
                                          modi_joint_canevo::Impl* joint) {
  std::lock_guard<std::mutex> lk(map_mu_);
  joints_[node_id] = joint;
}

void modi_bus_canevo::Impl::UnregisterJoint(uint8_t node_id) {
  std::lock_guard<std::mutex> lk(map_mu_);
  joints_.erase(node_id);
}

/* ---- Rx 线程 ---- */
void modi_bus_canevo::Impl::RxLoop() {
  struct canfd_frame cf {};
  struct pollfd pfd {};
  pfd.fd = sock_fd_;
  pfd.events = POLLIN;

  while (rx_running_.load(std::memory_order_acquire)) {
    int ret = ::poll(&pfd, 1, 5 /* ms */);
    if (ret <= 0) continue;

    ssize_t nbytes = ::read(sock_fd_, &cf, sizeof(cf));
    if (nbytes <= 0) continue;

    CanFrame f;
    f.id = static_cast<uint16_t>(cf.can_id & 0x7FFu);
    f.dlc = LenToDlc(cf.len);
    f.len = cf.len;
    f.is_fd = (nbytes == sizeof(struct canfd_frame)) ? 1 : 0;
    f.brs = (cf.flags & CANFD_BRS) ? 1 : 0;
    std::memcpy(f.data, cf.data, cf.len);

    DispatchFrame(f);
  }
}

/* ---- 帧分发 ---- */
void modi_bus_canevo::Impl::DispatchFrame(const CanFrame& f) {
  /* 根据 cob_id 确定 node_id */
  uint8_t node_id = 0;

  if (f.id >= kCobTxPdo0Base && f.id < kCobTxPdo0Base + 63) {
    node_id = static_cast<uint8_t>(f.id - kCobTxPdo0Base);
  } else if (f.id >= kCobSdoRspBase && f.id < kCobSdoRspBase + 63) {
    node_id = static_cast<uint8_t>(f.id - kCobSdoRspBase);
  } else if (f.id >= kCobEmcyBase && f.id < kCobEmcyBase + 63) {
    node_id = static_cast<uint8_t>(f.id - kCobEmcyBase);
  } else {
    return;  // 不关心的 cob_id
  }

  if (node_id == 0) return;

  std::lock_guard<std::mutex> lk(map_mu_);
  auto it = joints_.find(node_id);
  if (it != joints_.end()) {
    it->second->HandleFrame(f);
  }
}

/* ============================================================
 *              modi_joint_canevo::Impl 实现
 * ============================================================ */

int modi_joint_canevo::Impl::Init(modi_bus_canevo::Impl* bus, uint8_t node_id) {
  bus_ = bus;
  node_id_ = node_id;
  sdo_timeout_ms_ = bus->DefaultSdoTimeoutMs();
  controlword_.store(0, std::memory_order_relaxed);
  fault_clr_pending_.store(false, std::memory_order_relaxed);
  status_valid_ = false;
  return static_cast<int>(CanEvoError::kOk);
}

void modi_joint_canevo::Impl::Shutdown() {
  if (bus_) {
    bus_->UnregisterJoint(node_id_);
    bus_ = nullptr;
  }
  node_id_ = 0;
  controlword_.store(0, std::memory_order_relaxed);
  fault_clr_pending_.store(false, std::memory_order_relaxed);
  {
    std::lock_guard<std::mutex> lk(status_mu_);
    status_cache_ = JointStatus{};
    status_valid_ = false;
  }
}

/* ============================================================
 * controlword 缓存操作（非阻塞）
 * ============================================================ */

void modi_joint_canevo::Impl::SetEnable(bool on) {
  uint16_t cw = controlword_.load(std::memory_order_relaxed);
  if (on)
    cw |= kCwEnable;
  else
    cw &= ~kCwEnable;
  controlword_.store(cw, std::memory_order_relaxed);
}

void modi_joint_canevo::Impl::SetMode(CanEvoMode mode) {
  uint16_t cw = controlword_.load(std::memory_order_relaxed);
  cw &= ~kCwModeMask;
  cw |= (static_cast<uint16_t>(mode) << kCwModeShift);
  controlword_.store(cw, std::memory_order_relaxed);
}

void modi_joint_canevo::Impl::TriggerFaultClr() {
  fault_clr_pending_.store(true, std::memory_order_relaxed);
}

void modi_joint_canevo::Impl::SetEstop(bool on) {
  uint16_t cw = controlword_.load(std::memory_order_relaxed);
  if (on)
    cw |= kCwEstop;
  else
    cw &= ~kCwEstop;
  controlword_.store(cw, std::memory_order_relaxed);
}

uint16_t modi_joint_canevo::Impl::GetControlword() const {
  return controlword_.load(std::memory_order_relaxed);
}

/**
 * @brief 构建当前帧的 controlword 并消费单次脉冲标志
 *
 * 若 fault_clr_pending_ 为 true，则本帧 controlword 的 bit0=1，
 * 同时自动清除标志，下一帧 bit0 恢复为 0 — 形成单周期脉冲。
 */
uint16_t modi_joint_canevo::Impl::ConsumeControlword() {
  uint16_t cw = controlword_.load(std::memory_order_relaxed);
  if (fault_clr_pending_.exchange(false, std::memory_order_relaxed)) {
    cw |= kCwFaultClr;
  }
  return cw;
}

/* ============================================================
 * PDO 发送（非阻塞，自动携带内部 controlword）
 * ============================================================ */

int modi_joint_canevo::Impl::SendRxPdo0(float target_pos_deg) {
  CanFrame f;
  f.id = kCobRxPdo0Base + node_id_;
  f.len = 8;
  f.dlc = bus_->LenToDlc(f.len);

  WriteU16LE(f.data + 0, ConsumeControlword());
  WriteF32LE(f.data + 2, target_pos_deg);
  /* data[2..3] 保留 */

  return bus_->Send(f);
}

int modi_joint_canevo::Impl::SendRxPdo1(float target_vel_rpm) {
  CanFrame f;
  f.id = kCobRxPdo1Base + node_id_;
  f.len = 6;
  f.dlc = bus_->LenToDlc(f.len);

  WriteU16LE(f.data + 0, ConsumeControlword());
  WriteF32LE(f.data + 2, target_vel_rpm);

  return bus_->Send(f);
}

int modi_joint_canevo::Impl::SendRxPdo2(float target_cur_a) {
  CanFrame f;
  f.id = kCobRxPdo2Base + node_id_;
  f.len = 8;
  f.dlc = bus_->LenToDlc(f.len);

  WriteU16LE(f.data + 0, ConsumeControlword());
  WriteF32LE(f.data + 4, target_cur_a);

  return bus_->Send(f);
}

int modi_joint_canevo::Impl::SendRxPdo3(float target_pos_deg,
                                        float profile_vel_rpm,
                                        float profile_acc_rpms,
                                        float profile_dec_rpms) {
  CanFrame f;
  f.id = kCobRxPdo3Base + node_id_;
  f.len = 18;
  f.dlc = bus_->LenToDlc(f.len);

  WriteU16LE(f.data + 0, ConsumeControlword());
  /* data[2..3] 保留 */
  WriteF32LE(f.data + 2, target_pos_deg);
  WriteF32LE(f.data + 6, profile_vel_rpm);
  WriteF32LE(f.data + 10, profile_acc_rpms);
  WriteF32LE(f.data + 14, profile_dec_rpms);

  return bus_->Send(f);
}

/* ============================================================
 * 状态读取（非阻塞）
 * ============================================================ */

int modi_joint_canevo::Impl::GetStatus(JointStatus& out) {
  std::lock_guard<std::mutex> lk(status_mu_);
  if (!status_valid_) return static_cast<int>(CanEvoError::kNotInitialized);
  out = status_cache_;
  return static_cast<int>(CanEvoError::kOk);
}

uint16_t modi_joint_canevo::Impl::GetStatusword() {
  std::lock_guard<std::mutex> lk(status_mu_);
  return status_cache_.statusword;
}

bool modi_joint_canevo::Impl::GetEmcy(uint16_t& out_fault) {
  return emcy_q_.pop(out_fault);
}

/* ============================================================
 * SDO 阻塞读
 *
 * 帧格式（8 字节）：
 * Byte 0     : cmd (0x01 读请求)
 * Byte 1     : index (主索引)
 * Byte 2     : sub   (子索引)
 * Byte 3~7   : 保留
 *
 * cob_id = 0x780 + node_id
 * ============================================================ */

int modi_joint_canevo::Impl::SdoRead(uint8_t index, uint8_t sub, uint8_t* out,
                                     uint8_t out_len, uint8_t& actual_len) {
  if (!bus_ || !bus_->IsOpen())
    return static_cast<int>(CanEvoError::kBusNotOpen);

  std::unique_lock<std::mutex> lk(sdo_mu_);

  /* 发送 SDO 读请求 */
  CanFrame f;
  f.id = kCobSdoReqBase + node_id_;
  f.len = 8;
  f.dlc = bus_->LenToDlc(f.len);
  f.data[0] = kSdoCmdRd;
  f.data[1] = index;
  f.data[2] = sub;

  sdo_pending_.cmd = kSdoCmdRd;
  sdo_pending_.index = index;
  sdo_pending_.sub = sub;
  sdo_pending_.done = false;
  sdo_pending_.result = 0;
  sdo_pending_.payload_len = 0;

  int ret = bus_->Send(f);
  if (ret != static_cast<int>(CanEvoError::kOk)) return ret;

  /* 等待响应 */
  bool ok = sdo_cv_.wait_for(lk, std::chrono::milliseconds(sdo_timeout_ms_),
                             [this] { return sdo_pending_.done; });
  if (!ok) return static_cast<int>(CanEvoError::kSdoTimeout);
  if (sdo_pending_.result != 0) return sdo_pending_.result;

  actual_len =
      (sdo_pending_.payload_len < out_len) ? sdo_pending_.payload_len : out_len;
  std::memcpy(out, sdo_pending_.payload, actual_len);
  return static_cast<int>(CanEvoError::kOk);
}

/* ============================================================
 * SDO 阻塞写
 *
 * 帧格式（根据 CanEvo 协议 V1.2.3 表 7）：
 * Byte 0     : cmd (0x02 写请求)
 * Byte 1     : index
 * Byte 2     : sub
 * Byte 3~6   : data (little-endian, 1~4 bytes)
 *
 * cob_id = 0x780 + node_id
 * ============================================================ */

int modi_joint_canevo::Impl::SdoWrite(uint8_t index, uint8_t sub,
                                      const uint8_t* data, uint8_t len) {
  if (!bus_ || !bus_->IsOpen())
    return static_cast<int>(CanEvoError::kBusNotOpen);

  if (len > 4) return static_cast<int>(CanEvoError::kInvalidParam);

  std::unique_lock<std::mutex> lk(sdo_mu_);

  CanFrame f;
  f.id = kCobSdoReqBase + node_id_;
  f.len = 3 + len;
  f.dlc = bus_->LenToDlc(f.len);
  f.data[0] = kSdoCmdWr;
  f.data[1] = index;
  f.data[2] = sub;
  // f.data[3] = len;
  std::memcpy(f.data + 3, data, len);

  sdo_pending_.cmd = kSdoCmdWr;
  sdo_pending_.index = index;
  sdo_pending_.sub = sub;
  sdo_pending_.done = false;
  sdo_pending_.result = 0;

  int ret = bus_->Send(f);
  if (ret != static_cast<int>(CanEvoError::kOk)) return ret;

  bool ok = sdo_cv_.wait_for(lk, std::chrono::milliseconds(sdo_timeout_ms_),
                             [this] { return sdo_pending_.done; });
  if (!ok) return static_cast<int>(CanEvoError::kSdoTimeout);
  return sdo_pending_.result;
}

/* ============================================================
 * SDO 类型化辅助接口
 * ============================================================ */

int modi_joint_canevo::Impl::sdoReadU16(uint8_t index, uint8_t sub,
                                        uint16_t& val) {
  uint8_t buf[4] = {};
  uint8_t alen = 0;
  int ret = SdoRead(index, sub, buf, 2, alen);
  if (ret == static_cast<int>(CanEvoError::kOk)) val = ReadU16LE(buf);
  return ret;
}

int modi_joint_canevo::Impl::sdoWriteU16(uint8_t index, uint8_t sub,
                                         uint16_t val) {
  uint8_t buf[2];
  WriteU16LE(buf, val);
  return SdoWrite(index, sub, buf, 2);
}

int modi_joint_canevo::Impl::sdoReadI16(uint8_t index, uint8_t sub,
                                        int16_t& val) {
  uint8_t buf[4] = {};
  uint8_t alen = 0;
  int ret = SdoRead(index, sub, buf, 2, alen);
  if (ret == static_cast<int>(CanEvoError::kOk)) val = ReadI16LE(buf);
  return ret;
}

int modi_joint_canevo::Impl::sdoWriteI16(uint8_t index, uint8_t sub,
                                         int16_t val) {
  uint8_t buf[2];
  WriteI16LE(buf, val);
  return SdoWrite(index, sub, buf, 2);
}

  int modi_joint_canevo::Impl::sdoReadU32(uint8_t index, uint8_t sub,
                                          uint32_t& val) {
    uint8_t buf[4] = {};
    uint8_t alen = 0;
    int ret = SdoRead(index, sub, buf, 4, alen);
    if (ret == static_cast<int>(CanEvoError::kOk)) val = ReadU32LE(buf);
    return ret;
  }

  int modi_joint_canevo::Impl::sdoWriteU32(uint8_t index, uint8_t sub,
                                           uint32_t val) {
    uint8_t buf[4];
    WriteU32LE(buf, val);
    return SdoWrite(index, sub, buf, 4);
  }

  int modi_joint_canevo::Impl::sdoReadF32(uint8_t index, uint8_t sub,
                                          float& val) {
    uint8_t buf[4] = {};
    uint8_t alen = 0;
    int ret = SdoRead(index, sub, buf, 4, alen);
    if (ret == static_cast<int>(CanEvoError::kOk)) val = ReadF32LE(buf);
    return ret;
  }

  int modi_joint_canevo::Impl::sdoWriteF32(uint8_t index, uint8_t sub,
                                           float val) {
    uint8_t buf[4];
    WriteF32LE(buf, val);
    return SdoWrite(index, sub, buf, 4);
  }

  int modi_joint_canevo::Impl::SetSyncPeriod(uint16_t period_us) {
    return sdoWriteU16(0x00, 0x0F, period_us);
  }

  /* ============================================================
   * Bus Rx 线程回调入口
   * ============================================================ */

  void modi_joint_canevo::Impl::HandleFrame(const CanFrame& f) {
    /* 判断帧类型 */
    if (f.id == kCobSdoRspBase + node_id_) {
      OnSdoResp(f);
    } else if (f.id == kCobTxPdo0Base + node_id_) {
      OnTxPdo0(f);
    } else if (f.id == kCobEmcyBase + node_id_) {
      OnEmcy(f);
    }
  }

  void modi_joint_canevo::Impl::OnSdoResp(const CanFrame& f) {
    std::lock_guard<std::mutex> lk(sdo_mu_);

    if (sdo_pending_.done) return;

    uint8_t cmd = f.data[0];

    /* 应答故障 */
    if (cmd & kSdoCmdAbortBit) {
      sdo_pending_.abort_code = ReadU16LE(f.data + 4);
      sdo_pending_.result = static_cast<int>(CanEvoError::kSdoAbort);
      sdo_pending_.done = true;
      sdo_cv_.notify_one();
      return;
    }

    /* 校验 index/sub */
    if (f.data[1] != sdo_pending_.index || f.data[2] != sdo_pending_.sub)
      return;

    if (sdo_pending_.cmd == kSdoCmdRd) {
      /* CanEvo 协议：数据从 Byte3 开始，长度 = f.len - 3 */
      uint8_t dlen = f.len - 3;  // 总长度减去3字节头
      if (dlen > 4) dlen = 4;    // 最多4字节

      std::memcpy(sdo_pending_.payload, f.data + 3, dlen);
      sdo_pending_.payload_len = dlen;
      sdo_pending_.result = static_cast<int>(CanEvoError::kOk);

    } else if (sdo_pending_.cmd == kSdoCmdWr) {
      sdo_pending_.result = static_cast<int>(CanEvoError::kOk);
    }

    sdo_pending_.done = true;
    sdo_cv_.notify_one();
  }
  /* ============================================================
   * TxPDO0 解析
   *
   * 默认映射 (CanEvo V1.2.3)：
   * 默认映射 (CanEvo V1.2.3)：
   *   Byte  0~1  : statusword (uint16)
   *   Byte  2~5  : actual_pos (float, °)
   *   Byte  6~9 : actual_vel (float, rpm)
   *   Byte 10~13 : actual_cur (float, A)
   *   Byte 14~17 : actual_acc (float, rpm/s)
   *   Byte 18~19 : bus_voltage (uint16, *0.01 => V)
   *   Byte 20~21 : pcb_temp   (int16, *0.1 => ℃)
   *   Byte 22~23 : motor_temp (int16, *0.1 => ℃)
   * ============================================================ */

  void modi_joint_canevo::Impl::OnTxPdo0(const CanFrame& f) {
    if (f.len < 24) return;  // 至少需要 24 字节

    JointStatus st;
    st.statusword = ReadU16LE(f.data + 0);
    st.actual_pos_rad = ReadF32LE(f.data + 2) * kDegToRad;
    st.actual_vel_rads = ReadF32LE(f.data + 6) * kRpmToRads;
    st.actual_cur_a = ReadF32LE(f.data + 10);
    st.actual_acc_radss = ReadF32LE(f.data + 14) * kRpmToRads;
    st.bus_voltage_v = ReadU16LE(f.data + 18) * 0.01f;
    st.pcb_temp_c = ReadI16LE(f.data + 20) * 0.1f;
    st.motor_temp_c = ReadI16LE(f.data + 22) * 0.1f;

    {
      std::lock_guard<std::mutex> lk(status_mu_);
      status_cache_ = st;
      status_valid_ = true;
    }
  }

  void modi_joint_canevo::Impl::OnEmcy(const CanFrame& f) {
    if (f.len < 2) return;
    uint16_t fault = ReadU16LE(f.data);
    emcy_q_.push(fault);
  }
