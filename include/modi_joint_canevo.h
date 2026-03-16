/**
 * @file    modi_joint_canevo.h
 * @brief   CanEvo CAN-FD 关节控制 C++ SDK — 唯一对外公开头文件
 *
 * @details
 * 本文件提供 x86 (Linux/SocketCAN) 平台上的 CanEvo 关节 C++ 控制接口。
 * 包含两个核心类：
 *   - modi_bus_canevo  : 总线管理（对应一条 SocketCAN 接口，如 can0）
 *   - modi_joint_canevo: 关节控制（对应总线上的一个节点 node_id）
 *
 * 设计要点：
 *   - Bus + Joint 两级架构，同一总线上可挂多个关节
 *   - PDO 接口保证非阻塞实时性（SPSC 无锁队列 + 专用 Tx 线程）
 *   - SDO 接口为同步阻塞调用（内部串行，自动等待应答）
 *   - PIMPL 隐藏实现细节，公开头文件无平台依赖
 *
 * 典型用法：
 * @code
 *   modi_bus_canevo bus;
 *   bus.Open("can0");
 *
 *   modi_joint_canevo j1, j2;
 *   j1.NrtInit(bus, 1);
 *   j2.NrtInit(bus, 2);
 *
 *   // 配置模式并使能（使用 NrtEnable）
 *   j1.NrtEnable(CanEvoMode::kCsp);
 *   j2.NrtEnable(CanEvoMode::kCsp);
 *
 *   // 实时控制循环
 *   uint8_t cnt = 0;
 *   while (running) {
 *       bus.RtSendSync(cnt++);
 *
 *       JointStatus st;
 *       j1.RtGetJointStatus(st);
 *       // ...
 *
 *       j1.RtSetCspTargetPosition(pos1_rad);
 *       j2.RtSetCspTargetPosition(pos2_rad);
 *   }
 *
 *   // 失能（使用 NrtDisable）
 *   j1.NrtDisable();
 *   j2.NrtDisable();
 *
 *   j1.NrtDestroy();
 *   j2.NrtDestroy();
 *   bus.Close();
 * @endcode
 *
 * @protocol CanEvo V1.2.3
 * @version  1.0
 * @date     2026-02-26
 */

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

/* ============================================================
 * 错误码
 * ============================================================ */

enum class CanEvoError : int {
  kOk = 0,
  kInvalidParam = -1, /**< 参数非法（空指针/越界/不合理组合） */
  kBusNotOpen = -2,   /**< 总线未打开 */
  kSendFailed = -3,   /**< CAN 发送失败 */
  kSdoTimeout = -4,   /**< SDO 应答超时 */
  kSdoAbort = -5,     /**< SDO 应答式故障（从站返回错误） */
  kNotInitialized = -6, /**< Joint 未初始化 */
  kNodeNotFound = -7,   /**< 节点未注册 */
  kQueueFull = -8,      /**< 发送队列满 */
};

/* ============================================================
 * 工作模式（控制字 bit12~bit15）
 * ============================================================ */

enum class CanEvoMode : uint8_t {
  kCsp = 1, /**< 周期同步位置 (Cyclic Synchronous Position) */
  kCsv = 2, /**< 周期同步速度 (Cyclic Synchronous Velocity) */
  kCst = 3, /**< 周期同步转矩 (Cyclic Synchronous Torque) */
  kPp = 4,  /**< 轮廓位置 (Profile Position) */
};

/* ============================================================
 * 伺服状态机（状态字 bit0~bit3）
 * ============================================================ */

enum class CanEvoServoState : uint8_t {
  kInit = 0x00,       /**< 初始化 */
  kDisabled = 0x01,   /**< 禁止使能 */
  kReady = 0x02,      /**< 准备使能 */
  kRunning = 0x03,    /**< 运行（伺服使能） */
  kQuickStop = 0x04,  /**< 快速停机 */
  kFaultReact = 0x06, /**< 故障响应 */
  kFault = 0x07,      /**< 故障 */
  kInitFault = 0x08,  /**< 初始化故障 */
};

/* ============================================================
 * 故障码枚举（对象字典 0x0008，16bit）
 * ============================================================ */

enum class CanEvoFault : uint16_t {
  kNone = 0x0000,
  kOverCurrent = 0x2130,   /**< 相电流过流 */
  kBusOverVolt = 0x3210,   /**< 总线过压 */
  kBusUnderVolt = 0x3220,  /**< 总线欠压 */
  kMotorOverload = 0x3230, /**< 电机过载 */
  kPhaseLoss = 0x3331,     /**< 电机动力线缺相 */
  kOverTemp = 0x4310,      /**< 关节过温保护 */
  kUnderTemp = 0x4320,     /**< 关节低温保护 */
  kParamStorage = 0x5520,  /**< 硬件参数存储错误 */
  kBrake = 0x7110,         /**< 抱闸异常 */
  kMotorStall = 0x7121,    /**< 电机堵转保护 */
  kEncoder1 = 0x7303,      /**< 编码器1错误 */
  kEncoder2 = 0x7304,      /**< 编码器2错误 */
  kEnc1Battery = 0x7385,   /**< 多圈编码器1电池低电压 */
  kEnc2Battery = 0x7386,   /**< 多圈编码器2电池低电压 */
  kComm = 0x8100,          /**< 通讯错误 */
  kOverSpeed = 0x8400,     /**< 超过最大速度 */
  kFollowError = 0x8611,   /**< 跟随误差过大 */
  kCurrentSensor = 0xFF00, /**< 电流传感器故障 */
  kEncZLoss = 0xFF01,      /**< 编码器Z信号丢失 */
  kPeakOverload = 0xFF02,  /**< 关节峰值电流过载 */
  kHwShort = 0xFF03,       /**< 硬件短路保护 */
  kHall = 0xFF04,          /**< 编码器Hall信号故障 */
  kSyncError = 0xFF05,     /**< 同步周期误差过大 */
  kAngleIdent = 0xFF06,    /**< 电角度辨识错误 */
  kTimeout = 0xFF07,       /**< 伺服程序运行超时 */
  kFwError = 0xFF08,       /**< 固件错误 */
};

/* ============================================================
 * 警告码枚举（对象字典 0x0009，16bit）
 * ============================================================ */

enum class CanEvoWarn : uint16_t {
  kNone = 0x0000,
  kPcbHighTemp = 0x2101,   /**< PCB高温 */
  kMotorHighTemp = 0x2102, /**< 电机高温 */
  kGearHighTemp = 0x2103,  /**< 减速器高温 */
  kPhaseOverload = 0x2201, /**< 相电流过载 */
  kSpeedHigh = 0x2301,     /**< 速度过高 */
  kPosLimit = 0x2401,      /**< 位置超过限制值 */
  kFlashErase = 0x2601,    /**< flash擦除次数过高 */
  kVoltHigh = 0x2701,      /**< 供电电压高 */
  kVoltLow = 0x2702,       /**< 供电电压低 */
};

/* ============================================================
 * 抱闸控制常量
 * ============================================================ */

constexpr uint16_t kCanevoBrakeOpen = 0xAA55;
constexpr uint16_t kCanevoBrakeClose = 0xBB66;

/* ============================================================
 * 关节状态结构体（TxPDO0 解析结果，工程单位）
 * ============================================================ */

struct JointStatus {
  uint16_t statusword = 0;
  float actual_pos_rad = 0.0f;   /**< 实际位置 (rad) */
  float actual_vel_rads = 0.0f;  /**< 实际速度 (rad/s) */
  float actual_cur_a = 0.0f;     /**< 实际转矩电流 (A) */
  float actual_acc_radss = 0.0f; /**< 实际加速度 (rad/s²) */
  float bus_voltage_v = 0.0f; /**< 母线电压 (V) — 由 raw*0.01 转换 */
  float pcb_temp_c = 0.0f;    /**< PCB温度 (℃) — 由 raw*0.1 转换 */
  float motor_temp_c = 0.0f;  /**< 电机温度 (℃) — 由 raw*0.1 转换 */
};

struct TaskConfig {
  int period_us = 5000; /**< 控制周期 (us, 微秒) */
  int priority = 90;    /**< 实时优先级 */
  int cpu_affinity = 2; /**< CPU 亲和性（绑定到哪个核心） */
};

/* ============================================================
 * 前置声明
 * ============================================================ */

class modi_joint_canevo;
/**
 * @brief 控制循环回调函数类型
 *
 * 用户实现此函数，在每个控制周期被调用一次。
 * 典型使用：读取关节状态、计算控制量、发送控制命令。
 *
 * @note 建议使用 Lambda 捕获关节对象和状态变量
 * @note 示例：auto callback = [&joint]() { joint.RtSetCspTargetPosition(...);
 * };
 */
using ControlLoopCallback = std::function<void()>;

/* ============================================================
 * modi_bus_canevo — 总线管理
 * ============================================================
 *
 * 代表一条 SocketCAN 总线（如 can0），职责：
 *   - 管理 SocketCAN socket 生命周期
 *   - 收包线程：持续接收帧并按 cob_id 分发到对应 Joint
 *   - 发包：SYNC 广播、PDO 发送队列
 *   - 多个 Joint 共享同一个 Bus 实例
 */

class modi_bus_canevo {
 public:

  class Impl; /**< PIMPL 前置声明（定义在内部头文件） */

  modi_bus_canevo();
  ~modi_bus_canevo();

  // 禁止拷贝
  modi_bus_canevo(const modi_bus_canevo&) = delete;
  modi_bus_canevo& operator=(const modi_bus_canevo&) = delete;

  /**
   * @brief 打开 SocketCAN 总线并配置任务参数
   * @param can_ifname SocketCAN 接口名，例如 "can0"
   * @param task_config
   * 任务配置（控制周期(us)、优先级、CPU亲和性），使用默认值则为
   * 5000us(5ms)、优先级99、CPU 2
   * @return CanEvoError::kOk 成功，其他为失败
   * @note 配置会应用于 Rx 线程和控制循环线程
   */
  int Open(const std::string& can_ifname,
           const TaskConfig& task_config = TaskConfig());

  /**
   * @brief 关闭总线，释放所有资源
   * @note 会自动停止控制循环线程和 Rx 线程
   */
  void Close();

  /**
   * @brief 总线是否已打开
   */
  bool IsOpen() const;

  /**
   * @brief 发送 SYNC 广播帧 (cob_id = 0x03F) [实时接口]
   *
   * 每个控制周期调用一次，counter 0~255 循环递增。
   * SYNC 使用独立 socket fd 发送，避免与 PDO 争抢锁。
   *
   * @param counter 同步计数器 (0~255)
   * @return CanEvoError::kOk 成功
   */
  int RtSendSync(const uint8_t counter);

  /**
   * @brief 设置 SDO 默认超时（作为后续创建 joint 的默认值）
   * @param ms 超时毫秒数
   */
  void SetSdoTimeoutMs(const int ms);

  /**
   * @brief 设置 PDO 默认超时
   * @param ms 超时毫秒数
   */
  void SetPdoTimeoutMs(const int ms);

  /**
   * @brief 启动实时控制循环线程
   * @param callback 每个控制周期调用的回调函数
   * @return CanEvoError::kOk 成功，其他为失败
   */
  int StartControlLoop(ControlLoopCallback callback);

  /**
   * @brief 等待控制循环线程结束,如果控制循环未启动,则立即返回
   */
  void Join();

 private:

  std::unique_ptr<Impl> impl_;
  friend class modi_joint_canevo;
};

/* ============================================================
 * modi_joint_canevo — 关节控制
 * ============================================================
 *
 * 代表总线上的一个关节节点 (node_id)，职责：
 *   - PDO 实时控制（非阻塞）：发送 RxPDO0~3，读取 TxPDO0 状态
 *   - SDO 配置/诊断（阻塞）：读写对象字典
 *   - 控制辅助：使能、失能、清故障、切模式
 *
 * 线程安全约定：
 *   - PDO 实时循环建议单线程调度（减少周期抖动）
 *   - SDO 同一 Joint 不建议并发（SDK 内部 per-joint 串行）
 */

class modi_joint_canevo {
 public:

  class Impl; /**< PIMPL 前置声明（定义在内部头文件） */

  modi_joint_canevo();
  ~modi_joint_canevo();

  // 禁止拷贝
  modi_joint_canevo(const modi_joint_canevo&) = delete;
  modi_joint_canevo& operator=(const modi_joint_canevo&) = delete;

  /* ============================================================
   * 生命周期
   * ============================================================ */

  /**
   * @brief 初始化关节，绑定到总线和节点ID [非实时接口]
   * @param bus 已打开的总线实例
   * @param node_id 节点 ID (1~62)
   * @return CanEvoError::kOk 成功
   */
  int NrtInit(modi_bus_canevo& bus, const uint8_t node_id);

  /**
   * @brief 释放关节资源，从总线注销 [非实时接口]
   */
  void NrtDestroy();

  /**
   * @brief 获取当前绑定的 node_id [非实时接口]
   */
  uint8_t NrtNodeId() const;

  /* ============================================================
   * PDO 实时控制（非阻塞）
   * ============================================================
   *
   * 所有 set*Target* 函数仅将帧入队（SPSC 无锁队列），
   * 由内部 Tx 线程异步执行 write()，不阻塞调用者。
   * controlword 由 SDK 内部缓存自动管理（通过 NrtEnable/NrtDisable
   * 等修改）。
   *
   * 典型调用顺序：
   *   1) bus.RtSendSync(counter)
   *   2) joint.RtGetJointStatus(st)  — 读取最新缓存
   *   3) joint.RtSetCspTargetPosition(pos_rad) 或其他模式
   */

  /**
   * @brief CSP 模式：发送目标位置 (RxPDO0, cob_id = 0x200 + node_id) [实时接口]
   * @param target_pos_rad 目标位置 (rad)
   * @return CanEvoError::kOk 成功
   * @note controlword 由内部缓存自动携带
   */
  int RtSetCspTargetPosition(const float target_pos_rad);

  /**
   * @brief CSV 模式：发送目标速度 (RxPDO1, cob_id = 0x240 + node_id) [实时接口]
   * @param target_vel_rads 目标速度 (rad/s)
   * @return CanEvoError::kOk 成功
   */
  int RtSetCsvTargetVelocity(const float target_vel_rads);

  /**
   * @brief CST 模式：发送目标转矩电流 (RxPDO2, cob_id = 0x280 + node_id)
   * [实时接口]
   * @param target_cur_a 目标转矩电流 (A)
   * @return CanEvoError::kOk 成功
   */
  int RtSetCstTargetCurrent(const float target_cur_a);

  /**
   * @brief PP 模式：发送轮廓位置目标 (RxPDO3, cob_id = 0x2C0 + node_id)
   * [实时接口]
   * @param target_pos_rad 目标位置 (rad)
   * @param profile_vel_rads 轮廓速度 (rad/s)
   * @param profile_acc_radss 轮廓加速度 (rad/s²)
   * @param profile_dec_radss 轮廓减速度 (rad/s²)
   * @return CanEvoError::kOk 成功
   */
  int RtSetPpTargetPosition(const float target_pos_rad,
                            const float profile_vel_rads,
                            const float profile_acc_radss,
                            const float profile_dec_radss);

  /**
   * @brief 获取最新关节状态（非阻塞，从 TxPDO0 缓存读取） [实时接口]
   * @param[out] out 输出状态结构体
   * @return CanEvoError::kOk 成功（有新数据），NotInitialized 未初始化
   */
  int RtGetJointStatus(JointStatus& out);

  /**
   * @brief 获取最新 EMCY 故障码（非阻塞，从 EMCY 队列消费） [实时接口]
   * @param[out] out_fault 输出故障码枚举
   * @return true 有新的 EMCY 报文，false 无
   */
  bool RtGetEmcy(CanEvoFault& out_fault);

  /* ============================================================
   * controlword 控制（非阻塞，修改内部缓存，下次 PDO 帧生效）
   * ============================================================ */

  /* RtEnable 和 RtDisable 已被移除，请使用 NrtEnable/NrtDisable */

  /**
   * @brief 清除故障（触发控制字 bit0 单周期脉冲） [实时接口]
   * @note 下次 PDO 发送时 bit0=1，之后自动清零
   * @return CanEvoError::kOk 成功，NotInitialized 未初始化
   */
  int RtClearFault();

  /**
   * @brief 触发急停（设置控制字 bit2 = 1） [实时接口]
   * @return CanEvoError::kOk 成功，NotInitialized 未初始化
   */
  int RtEstop();

  /**
   * @brief 清除急停（设置控制字 bit2 = 0） [实时接口]
   * @return CanEvoError::kOk 成功，NotInitialized 未初始化
   */
  int RtClearEstop();

  /* ============================================================
   * SDO 控制字操作（阻塞，通过 SDO 直接修改 0x21/0x00）
   * ============================================================
   *
   * 这些接口通过 SDO 读写控制字，每次只修改相应位，不改变其他位。
   * 操作完成后控制字立即生效，无需等待 PDO 发送。
   */

  /**
   * @brief SDO 方式伺服使能并设置工作模式（设置控制字 bit1 = 1 和 bit12~bit15）
   * [非实时接口]
   * @param mode 目标工作模式
   * @note 通过 SDO 修改控制字 bit1=1 和
   * bit12-15，操作完成后控制字立即生效，无需等待 PDO 发送
   * @return CanEvoError::kOk 成功
   */
  int NrtEnable(const CanEvoMode mode);

  /**
   * @brief SDO 方式伺服失能（先切换到 CSP 模式，再设置控制字 bit1 = 0）
   * [非实时接口]
   * @note 通过 SDO 修改控制字，先设置模式为
   * CSP（bit12~bit15），再失能（bit1=0） 操作完成后控制字立即生效，无需等待 PDO
   * 发送
   * @return CanEvoError::kOk 成功
   */
  int NrtDisable();

  /**
   * @brief SDO 方式清除故障（触发控制字 bit0 单周期脉冲） [非实时接口]
   * @note 通过 SDO 发送单周期脉冲(bit0=1然后自动清0)，仅在故障状态时有效
   * @return CanEvoError::kOk 成功
   */
  int NrtClearFault();

  /**
   * @brief SDO 方式触发急停（设置控制字 bit2 = 1） [非实时接口]
   * @note 通过 SDO 修改控制字 bit2=1，每次只修改急停位，不改变其他位
   * @return CanEvoError::kOk 成功
   */
  int NrtEstop();

  /**
   * @brief SDO 方式清除急停（设置控制字 bit2 = 0） [非实时接口]
   * @note 通过 SDO 修改控制字 bit2=0，每次只修改急停位，不改变其他位
   * @return CanEvoError::kOk 成功
   */
  int NrtClearEstop();

  /* ============================================================
   * SDO 命名化接口 — 设备信息 (Index 0x00)
   * ============================================================
   *
   * get 类接口直接返回结果值，SDO 通信失败时返回 0 / 默认值。
   */

  /** @brief 0x00/0x01 读取协议版本 (uint16, R) [非实时接口] */
  uint16_t NrtGetProtocolVersion();

  /** @brief 0x00/0x02 读取关节代号 (uint32, R) [非实时接口] */
  uint32_t NrtGetModuleType();

  /** @brief 0x00/0x03 读取厂商代号 (uint32, R) [非实时接口] */
  uint32_t NrtGetVendorCode();

  /**
   * @brief 0x00/0x04~0x07 读取关节全球ID (4×uint32, R) [非实时接口]
   * @param[out] id 输出数组（4 个 uint32）
   * @return CanEvoError::kOk 成功
   */
  int NrtGetGlobalId(uint32_t id[4]);

  /** @brief 0x00/0x08 读取故障代码 (SDO 阻塞, R)，失败返回 CanEvoFault::kNone
   * [非实时接口] */
  CanEvoFault NrtGetFaultCode();

  /** @brief 0x00/0x09 读取警告代码 (SDO 阻塞, R)，失败返回 CanEvoWarn::kNone
   * [非实时接口] */
  CanEvoWarn NrtGetWarnCode();

  /** @brief 0x00/0x0A 读取诊断信息 (uint16, R) [非实时接口] */
  uint16_t NrtGetDiagInfo();

  /** @brief 0x00/0x0B 读取 CAN 波特率编码 (uint16, RW) [非实时接口] */
  uint16_t NrtGetCanBaud();
  /** @brief 0x00/0x0B 写入 CAN 波特率编码 (uint16, RW) — 保存参数+重启后生效
   * [非实时接口] */
  int NrtSetCanBaud(const uint16_t baud);

  /** @brief 0x00/0x0C 读取 CAN ID (uint16, RW, 1~62) [非实时接口] */
  uint16_t NrtGetCanId();
  /** @brief 0x00/0x0C 写入 CAN ID (uint16, RW) — 保存参数+重启后生效
   * [非实时接口] */
  int NrtSetCanId(const uint16_t can_id);

  /** @brief 0x00/0x0D 读取固件版本 (uint16, R) [非实时接口] */
  uint16_t NrtGetFwVersion();

  /** @brief 0x00/0x0E 读取硬件版本 (uint16, R) [非实时接口] */
  uint16_t NrtGetHwVersion();

  /** @brief 0x00/0x0F 读取同步周期 (uint16, RW, 单位 us) [非实时接口] */
  uint16_t NrtGetSyncPeriod();

  /** @brief 0x00/0x10 读取通信超时时间 (uint16, RW, 单位 ms) [非实时接口] */
  uint16_t NrtGetCommTimeout();
  /** @brief 0x00/0x10 写入通信超时时间 (uint16, RW, 单位 ms) [非实时接口] */
  int NrtSetCommTimeout(const uint16_t ms);

  /* ============================================================
   * SDO 命名化接口 — 关节参数 (Index 0x01)
   * ============================================================ */

  /**
   * @brief 0x01/0x00~0x01 获取减速比 (uint16, R) [非实时接口]
   * @param[out] num 分子
   * @param[out] den 分母
   * @return CanEvoError::kOk 成功
   * @note 减速比 = num / den
   */
  int NrtGetGearRatio(uint16_t& num, uint16_t& den);

  /** @brief 0x01/0x02 获取最大速度 (float, RW, rad/s) [非实时接口] */
  float NrtGetMaxSpeed();
  /** @brief 0x01/0x02 设置最大速度 (float, RW, rad/s) [非实时接口] */
  int NrtSetMaxSpeed(const float rads);

  /** @brief 0x01/0x03 获取最大加速度 (float, RW, rad/s²) [非实时接口] */
  float NrtGetMaxAccel();
  /** @brief 0x01/0x03 设置最大加速度 (float, RW, rad/s²) [非实时接口] */
  int NrtSetMaxAccel(const float radss);

  /** @brief 0x01/0x04 获取最大减速度 (float, RW, rad/s²) [非实时接口] */
  float NrtGetMaxDecel();
  /** @brief 0x01/0x04 设置最大减速度 (float, RW, rad/s²) [非实时接口] */
  int NrtSetMaxDecel(const float radss);

  /** @brief 0x01/0x05 获取急停减速度 (float, RW, rad/s²) [非实时接口] */
  float NrtGetEstopDecel();
  /** @brief 0x01/0x05 设置急停减速度 (float, RW, rad/s²) [非实时接口] */
  int NrtSetEstopDecel(const float radss);

  /** @brief 0x01/0x06 获取最大位置 (float, RW, rad) [非实时接口] */
  float NrtGetMaxPos();
  /** @brief 0x01/0x06 设置最大位置 (float, RW, rad) [非实时接口] */
  int NrtSetMaxPos(const float rad);

  /** @brief 0x01/0x07 获取最小位置 (float, RW, rad) [非实时接口] */
  float NrtGetMinPos();
  /** @brief 0x01/0x07 设置最小位置 (float, RW, rad) [非实时接口] */
  int NrtSetMinPos(const float rad);

  /**
   * @brief 0x01/0x08 设置机械零点 (W, 写入 0x0001 触发) [非实时接口]
   * @note 建议在伺服失能状态下执行
   */
  int NrtSetMechZero();

  /** @brief 0x01/0x09 读取机械零点设置成功标志 (uint16, R) — 1=成功
   * [非实时接口] */
  uint16_t NrtGetMechZeroOk();

  /** @brief 0x01/0x0A 位置限制使能 (uint16, W) — 写 1 使能，写 0 禁止
   * [非实时接口] */
  int NrtSetPosLimitEnable(const uint16_t enable);

  /** @brief 0x01/0x0B 读取位置限制状态 (uint16, R) — 0=未使能，1=已使能
   * [非实时接口] */
  uint16_t NrtGetPosLimitStatus();

  /**
   * @brief 0x01/0x0C 抱闸控制 (uint16, W) [非实时接口]
   * @param val kCanevoBrakeOpen(0xAA55)=打开, kCanevoBrakeClose(0xBB66)=关闭
   */
  int NrtSetBrakeControl(const uint16_t val);

  /** @brief 0x01/0x0D 读取抱闸状态 (uint16, R) — 0=关闭，1=打开 [非实时接口] */
  uint16_t NrtGetBrakeStatus();

  /**
   * @brief 0x01/0xF0 保存参数 (W, 写入 0x0001 触发) [非实时接口]
   * @note 建议在伺服失能状态下执行；保存需要时间，之后查询 NrtGetSaveParamsOk()
   * 确认
   */
  int NrtSaveParams();

  /** @brief 0x01/0xF1 读取参数保存成功标志 (uint16, R) — 1=成功 [非实时接口] */
  uint16_t NrtGetSaveParamsOk();

  /* ============================================================
   * SDO 命名化接口 — 控制器参数 (Index 0x07, 全 uint16 RW)
   * ============================================================ */

  /**
   * @brief 通用读取控制器参数 0x07/sub (uint16, RW) [非实时接口]
   * @param sub 子索引 (0x00~0x0A)
   * @return 参数值，失败返回 0
   */
  uint16_t NrtGetCtrlParam(const uint8_t sub);

  /**
   * @brief 通用写入控制器参数 0x07/sub (uint16, RW) [非实时接口]
   * @param sub 子索引 (0x00~0x0A)
   * @param val 写入值
   */
  int NrtSetCtrlParam(const uint8_t sub, const uint16_t val);

  /* ============================================================
   * SDO 命名化接口 — 编码器参数 (Index 0x08, uint32 R)
   * ============================================================ */

  /** @brief 0x08/0x00 读取电机轴编码器分辨率 (uint32, R, Pulse/Rev)
   * [非实时接口] */
  uint32_t NrtGetMotorEncRes();

  /** @brief 0x08/0x01 读取输出轴编码器分辨率 (uint32, R, Pulse/Rev)
   * [非实时接口] */
  uint32_t NrtGetOutputEncRes();

  /* ============================================================
   * SDO 命名化接口 — PP 模式轮廓参数 (Index 0x21, float RW)
   * ============================================================ */

  /**
   * @brief 设置 PP 模式轮廓参数（一次性设置所有参数）[非实时接口]
   * @param target_pos_rad 目标位置 (rad)
   * @param profile_vel_rads 轮廓速度 (rad/s)
   * @param profile_acc_radss 轮廓加速度 (rad/s²)
   * @param profile_dec_radss 轮廓减速度 (rad/s²)
   * @return CanEvoError::kOk 成功
   * @note 通过 SDO 写入 0x21/0x02, 0x21/0x07, 0x21/0x08, 0x21/0x09
   */
  int NrtSetPpTargetPosition(const float target_pos_rad,
                             const float profile_vel_rads,
                             const float profile_acc_radss,
                             const float profile_dec_radss);

  /* ============================================================
   * SDO 命名化接口 — 实际量 (Index 0x20, 只读)
   * ============================================================ */

  /** @brief 0x20/0x00 读取实际位置 (float, R, rad) [非实时接口] */
  float NrtGetActualPosition();

  /** @brief 0x20/0x01 读取实际速度 (float, R, rad/s) [非实时接口] */
  float NrtGetActualVelocity();

  /** @brief 0x20/0x02 读取实际转矩电流 (float, R, A) [非实时接口] */
  float NrtGetActualCurrent();

  /** @brief 0x20/0x03 读取实际加速度 (float, R, rad/s²) [非实时接口] */
  float NrtGetActualAcceleration();

  /** @brief 0x20/0x04 读取实际转矩 (float, R, Nm) [非实时接口] */
  float NrtGetActualTorque();

  /** @brief 0x20/0x05 读取位置跟踪误差 (float, R, rad) [非实时接口] */
  float NrtGetPositionTrackingError();

  /** @brief 0x20/0x06 读取母线电压 (float, R, V) [非实时接口] */
  float NrtGetBusVoltage();

  /** @brief 0x20/0x07 读取 PCB 温度 (float, R, ℃) [非实时接口] */
  float NrtGetPcbTemperature();

  /** @brief 0x20/0x08 读取电机温度 (float, R, ℃) [非实时接口] */
  float NrtGetMotorTemperature();

  /** @brief 0x20/0x09 读取减速器温度 (float, R, ℃) [非实时接口] */
  float NrtGetGearboxTemperature();

  /** @brief 0x20/0x0A 读取电角度 (float, R, rad) [非实时接口] */
  float NrtGetElectricalAngle();

  /* ============================================================
   * 状态查询辅助（非阻塞，内部从 TxPDO0 缓存读取状态字）[实时接口]
   * ============================================================ */

  /**
   * @brief 获取伺服状态机状态 [实时接口]
   * @return CanEvoServoState 枚举
      */
  CanEvoServoState RtGetServoState();

  /**
   * @brief 获取当前工作模式 [实时接口]
   * @return CanEvoMode 枚举
   */
  CanEvoMode RtGetCurrentMode();

  /**
   * @brief 检查是否处于运行状态（伺服使能） [实时接口]
   */
  bool RtIsRunning();

  /**
   * @brief 检查是否有警告 [实时接口]
   */
  bool RtHasWarning();

  /**
   * @brief 通过 SDO 获取伺服状态机状态 [非实时接口]
   * @return CanEvoServoState 枚举
   * @note 通过 SDO 读取状态字（0x21/0x01），阻塞调用
   */
  CanEvoServoState NrtGetServoState();

  /**
   * @brief 通过 SDO 获取当前工作模式 [非实时接口]
   * @return CanEvoMode 枚举
   * @note 通过 SDO 读取状态字（0x21/0x01），阻塞调用
   */
  CanEvoMode NrtGetCurrentMode();

  /**
   * @brief 通过 SDO 检查是否处于运行状态（伺服使能） [非实时接口]
   * @return true 如果处于运行状态
   * @note 通过 SDO 读取状态字（0x21/0x01），阻塞调用
   */
  bool NrtIsRunning();

 private:

  std::unique_ptr<Impl> impl_;
};
