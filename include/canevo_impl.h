/**
 * @file    canevo_impl.h
 * @brief   CanEvo SDK 内部实现头文件（不对外发布）
 *
 * @details
 * 直接定义 modi_bus_canevo::Impl 和 modi_joint_canevo::Impl（PIMPL 实现类），
 * 以及内部用到的 CAN 帧结构、实时发送路径、协议编解码辅助等。
 *
 * @note 本文件仅供 SDK 内部 .cpp 使用，用户不应直接包含。
 *
 * @protocol CanEvo V1.2.4
 * @version  1.0
 * @date     2026-02-26
 */

#pragma once

#include <pthread.h>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "modi_joint_canevo.h"

namespace canevo {

/* ============================================================
 * 内部 CAN 帧结构
 * ============================================================ */

struct CanFrame {
  uint16_t id = 0;
  uint8_t dlc = 0;   /**< DLC code (0~15) */
  uint8_t len = 0;   /**< 实际 payload 字节数 (0~64) */
  uint8_t is_fd = 1; /**< CAN-FD 标志 */
  uint8_t brs = 1;   /**< Bit Rate Switch */
  uint8_t data[64] = {};
};

/* ============================================================
 * 带互斥锁的环形队列（多生产者安全，用于 EMCY 等低频场景）
 * ============================================================ */

template <typename T>
class RingQueue {
 public:

  explicit RingQueue(uint32_t capacity)
      : buf_(capacity), mask_(capacity - 1), capacity_(capacity) {}

  bool push(const T& item) {
    std::lock_guard<std::mutex> lk(mu_);
    uint32_t next = (write_ + 1) & mask_;
    if (next == read_) {
      // 覆盖最旧
      read_ = (read_ + 1) & mask_;
    }
    buf_[write_] = item;
    write_ = next;
    return true;
  }

  bool pop(T& item) {
    std::lock_guard<std::mutex> lk(mu_);
    if (read_ == write_) return false;
    item = buf_[read_];
    read_ = (read_ + 1) & mask_;
    return true;
  }

 private:

  std::vector<T> buf_;
  uint32_t mask_;
  uint32_t capacity_;
  uint32_t write_ = 0;
  uint32_t read_ = 0;
  std::mutex mu_;
};

/* ============================================================
 * 协议常量
 * ============================================================ */

constexpr uint16_t kCobSync = 0x03Fu;
constexpr uint16_t kCobEmcyBase = 0x080u;
constexpr uint16_t kCobRxPdo0Base = 0x200u;
constexpr uint16_t kCobRxPdo1Base = 0x240u;
constexpr uint16_t kCobRxPdo2Base = 0x280u;
constexpr uint16_t kCobRxPdo3Base = 0x2C0u;
constexpr uint16_t kCobRxPdo4Base = 0x300u;
constexpr uint16_t kCobRxPdo6Base = 0x380u;
constexpr uint16_t kCobTxPdo0Base = 0x400u;
constexpr uint16_t kCobTxPdo1Base = 0x440u;
constexpr uint16_t kCobSdoReqBase = 0x780u;
constexpr uint16_t kCobSdoRspBase = 0x7C0u;

constexpr uint8_t kSdoCmdRd = 0x01u;
constexpr uint8_t kSdoCmdWr = 0x02u;
constexpr uint8_t kSdoCmdAbortBit = 0x80u;

/* 控制字位定义 */
constexpr uint16_t kCwFaultClr = (1u << 0);
constexpr uint16_t kCwEnable = (1u << 1);
constexpr uint16_t kCwEstop = (1u << 2);
constexpr uint16_t kCwModeShift = 12u;
constexpr uint16_t kCwModeMask = (0xFu << kCwModeShift);

/* 状态字位定义 */
constexpr uint16_t kSwStateMask = 0x0Fu;
constexpr uint16_t kSwWarnBit = (1u << 4);
constexpr uint16_t kSwModeShift = 12u;
constexpr uint16_t kSwModeMask = (0xFu << kSwModeShift);

/* ============================================================
 * 协议编解码辅助（inline）
 * ============================================================ */

/** @brief 从字节流读取 little-endian uint16 */
inline uint16_t ReadU16LE(const uint8_t* p) {
  return static_cast<uint16_t>(p[0]) | (static_cast<uint16_t>(p[1]) << 8);
}

/** @brief 从字节流读取 little-endian int16 */
inline int16_t ReadI16LE(const uint8_t* p) {
  return static_cast<int16_t>(ReadU16LE(p));
}

/** @brief 从字节流读取 little-endian uint32 */
inline uint32_t ReadU32LE(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) |
         (static_cast<uint32_t>(p[3]) << 24);
}

/** @brief 从字节流读取 little-endian float (IEEE754 binary32) */
inline float ReadF32LE(const uint8_t* p) {
  uint32_t u = ReadU32LE(p);
  float f;
  std::memcpy(&f, &u, sizeof(f));
  return f;
}

/** @brief 写入 little-endian uint16 到字节流 */
inline void WriteU16LE(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
}

/** @brief 写入 little-endian int16 到字节流 */
inline void WriteI16LE(uint8_t* p, int16_t v) {
  WriteU16LE(p, static_cast<uint16_t>(v));
}

/** @brief 写入 little-endian uint32 到字节流 */
inline void WriteU32LE(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFF);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFF);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFF);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFF);
}

/** @brief 写入 little-endian float 到字节流 */
inline void WriteF32LE(uint8_t* p, float f) {
  uint32_t u;
  std::memcpy(&u, &f, sizeof(u));
  WriteU32LE(p, u);
}

}  // namespace canevo

/* ============================================================
 * modi_bus_canevo::Impl — 总线内部实现
 * ============================================================ */

class modi_bus_canevo::Impl {
 public:

  Impl();
  ~Impl();

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  int Open(const std::string& ifname, const TaskConfig& task_config);
  void Close();
  bool IsOpen() const { return sock_fd_ >= 0; }

  /* 发送（线程安全） */
  int Send(const canevo::CanFrame& f);

  /* 实时 PDO 发送（不经过 SDO 发送互斥锁） */
  int SendRealtime(const canevo::CanFrame& f);

  /* SYNC 发送（独立 fd） */
  int SendSync(uint8_t counter);

  /* 用户外部实时线程调用的一次总线步进 */
  int RtStepOnce();

  /* Joint 注册/注销 */
  void RegisterJoint(uint8_t node_id, modi_joint_canevo::Impl* joint);
  void UnregisterJoint(uint8_t node_id);
  std::vector<uint8_t> ScanJoints();

  /* 默认超时 */
  void SetDefaultSdoTimeoutMs(int ms) { default_sdo_timeout_ms_ = ms; }
  void SetDefaultPdoTimeoutMs(int ms) { default_pdo_timeout_ms_ = ms; }
  int DefaultSdoTimeoutMs() const { return default_sdo_timeout_ms_; }
  int DefaultPdoTimeoutMs() const { return default_pdo_timeout_ms_; }

  /* 任务配置访问 */
  const TaskConfig& GetTaskConfig() const { return task_config_; }

  /* 协议辅助 */
  uint8_t DlcToLen(uint8_t dlc) const;
  uint8_t LenToDlc(uint8_t len) const;

 private:

  int sock_fd_ = -1; /**< SocketCAN fd（接收 + SDO 发送） */
  int sync_fd_ = -1; /**< SYNC 专用 fd（无锁降低抖动） */
  int pdo_fd_ = -1;  /**< 实时 PDO 专用 fd（无锁降低抖动） */
  std::string ifname_;

  /* Rx 线程 */
  std::atomic<bool> rx_running_{false};
  std::thread rx_thread_;

  /* 发送互斥（用于 SDO 等非实时发送） */
  std::mutex tx_mu_;

  /* 默认超时 */
  int default_sdo_timeout_ms_ = 100;
  int default_pdo_timeout_ms_ = 50;

  /* 任务配置（Open 时设置） */
  TaskConfig task_config_;

  /* Joint 注册表 */
  std::mutex map_mu_;
  std::unordered_map<uint8_t, modi_joint_canevo::Impl*> joints_;  // 统一注册表

  std::atomic<uint8_t> sync_counter_{0};
  std::atomic<uint64_t> stat_sync_send_failed_{0};
  std::atomic<uint64_t> stat_pdo_send_failed_{0};

  /* 线程入口 */
  void RxLoop();
  int ConfigureThread(pthread_t thread, int sched_policy, int sched_priority,
                      int cpu_affinity);

  /* 帧分发 */
  void DispatchFrame(const canevo::CanFrame& f);

  /* SocketCAN 辅助 */
  int CreateCanFdSocket(const std::string& ifname);
  int SendFrame(int fd, const canevo::CanFrame& f);
};

/* ============================================================
 * modi_joint_canevo::Impl — 关节内部实现
 * ============================================================ */

class modi_joint_canevo::Impl {
 public:

  Impl() = default;
  ~Impl() = default;

  Impl(const Impl&) = delete;
  Impl& operator=(const Impl&) = delete;

  /** @brief 初始化（绑定到总线和节点） */
  int Init(modi_bus_canevo::Impl* bus, uint8_t node_id);

  /** @brief 关闭（从总线注销，重置状态） */
  void Shutdown();

  /** @brief 是否已初始化 */
  bool isInitialized() const { return bus_ != nullptr; }

  /* ---- controlword 缓存操作（非阻塞） ---- */
  void SetEnable(bool on);
  void SetMode(CanEvoMode mode);
  void TriggerFaultClr();
  void SetEstop(bool on);
  uint16_t GetControlword() const;

  /* ---- PDO 发送（非阻塞，自动携带内部 controlword） ---- */
  int SendRxPdo0(float target_pos_deg);
  int SendRxPdo1(float target_vel_rpm);
  int SendRxPdo2(float target_cur_a);
  int SendRxPdo3(float target_pos_deg, float profile_vel_rpm,
                 float profile_acc_rpms, float profile_dec_rpms);
  int SendRxPdo4(float target_vel_rpm, float profile_acc_rpms,
                 float profile_dec_rpms);
  int SendRxPdo6(float target_pos_deg, float target_vel_rpm,
                 float target_torque_nm, float kp, float kd);

  /* ---- 状态读取（非阻塞） ---- */
  int GetStatus(JointStatus& out);
  uint16_t GetStatusword();
  uint32_t StatusSeq() const;
  bool GetEmcy(uint16_t& out_fault);

  /* ---- SDO 阻塞读写 ---- */
  int SdoRead(uint8_t index, uint8_t sub, uint8_t* out, uint8_t out_len,
              uint8_t& actual_len);
  int SdoWrite(uint8_t index, uint8_t sub, const uint8_t* data, uint8_t len);

  /* ---- SDO 类型化辅助接口 ---- */
  int sdoReadU16(uint8_t index, uint8_t sub, uint16_t& val);
  int sdoWriteU16(uint8_t index, uint8_t sub, uint16_t val);
  int sdoReadI16(uint8_t index, uint8_t sub, int16_t& val);
  int sdoWriteI16(uint8_t index, uint8_t sub, int16_t val);
  int sdoReadU32(uint8_t index, uint8_t sub, uint32_t& val);
  int sdoWriteU32(uint8_t index, uint8_t sub, uint32_t val);
  int sdoReadF32(uint8_t index, uint8_t sub, float& val);
  int sdoWriteF32(uint8_t index, uint8_t sub, float val);

  void SetSdoTimeoutMs(int ms) { sdo_timeout_ms_ = ms; }

  /* ---- 内部配置接口 ---- */
  /** @brief 设置同步周期 (0x00/0x0F) - 在 NrtInit 时自动调用 */
  int SetSyncPeriod(uint16_t period_us);

  /* ---- Bus Rx 线程回调入口 ---- */
  void HandleFrame(const canevo::CanFrame& f);

 private:

  modi_bus_canevo::Impl* bus_ = nullptr;
  uint8_t node_id_ = 0;

  /* SDO 串行等待 */
  struct SdoPending {
    uint8_t cmd = 0;
    uint8_t index = 0;
    uint8_t sub = 0;
    bool done = false;
    int result = 0;
    uint16_t abort_code = 0;
    uint8_t payload[8] = {};
    uint8_t payload_len = 0;
  };

  std::mutex sdo_mu_;
  std::condition_variable sdo_cv_;
  SdoPending sdo_pending_;
  int sdo_timeout_ms_ = 100;

  /* controlword 内部缓存（用户线程读写，PDO 发送时自动携带） */
  std::atomic<uint16_t> controlword_{0};
  std::atomic<bool> fault_clr_pending_{false}; /**< 单次脉冲标志 */

  /** @brief 构建当前帧的 controlword（消费 fault_clr 脉冲） */
  uint16_t ConsumeControlword();

  /* 状态缓存 (TxPDO0/1) — Rx 线程写，用户线程读；seq 每次更新递增 */
  std::atomic<uint32_t> status_seq_{0};
  std::atomic<bool> status_valid_{false};
  std::atomic<uint16_t> statusword_cache_{0};
  std::atomic<float> actual_pos_rad_cache_{0.0f};
  std::atomic<float> actual_vel_rads_cache_{0.0f};
  std::atomic<float> actual_cur_a_cache_{0.0f};
  std::atomic<float> actual_acc_radss_cache_{0.0f};
  std::atomic<float> bus_voltage_v_cache_{0.0f};
  std::atomic<float> pcb_temp_c_cache_{0.0f};
  std::atomic<float> motor_temp_c_cache_{0.0f};

  /* EMCY 队列 */
  canevo::RingQueue<uint16_t> emcy_q_{16};

  /* 内部帧处理 */
  void OnSdoResp(const canevo::CanFrame& f);
  void OnTxPdo0(const canevo::CanFrame& f);
  void OnTxPdo1(const canevo::CanFrame& f);
  void OnEmcy(const canevo::CanFrame& f);
};
