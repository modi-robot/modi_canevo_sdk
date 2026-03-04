/**
 * @file    example_csp.cpp
 * @brief   CSP 模式简单测试示例
 *
 * @details
 * 本示例演示如何使用 modi_joint_canevo C++ API 进行 CSP (周期同步位置)
 * 模式控制。
 *
 * CSP 模式特点：
 * - 周期同步位置控制，每个 SYNC 周期发送目标位置
 * - 适用于需要精确位置同步的应用场景
 * - 控制周期建议 200Hz~1000Hz
 *
 * 使用步骤：
 * 1. 打开 CAN 总线
 * 2. 初始化关节并绑定节点 ID
 * 3. 设置工作模式为 CSP
 * 4. 使能关节
 * 5. 控制循环：发送 SYNC + 设置目标位置
 * 6. 读取关节状态
 * 7. 清理资源
 *
 * @version 1.0
 * @date    2026-02-26
 */

#include <chrono>
#include <cmath>
#include <cstdio>
#include <thread>

#include "modi_joint_canevo.h"

namespace {

// CAN 接口名称
constexpr const char* kCanIfname = "can0";
// 节点 ID
constexpr uint8_t kNodeId = 1;
// 控制周期 (Hz)
constexpr int kControlRateHz = 200;
// 运行时长 (秒)
constexpr int kDurationSec = 10;
// 正弦波幅度 (rad)
constexpr float kAmplitudeRad = 0.5f;
// 正弦波频率 (Hz)
constexpr float kWaveFreqHz = 0.5f;

}  // namespace

int main() {
  std::printf("=== CSP Mode Test Example ===\n");
  std::printf("CAN interface: %s\n", kCanIfname);
  std::printf("Node ID: %d\n", kNodeId);
  std::printf("Control rate: %d Hz\n", kControlRateHz);
  std::printf("Duration: %d s\n", kDurationSec);
  std::printf("\n");

  // 1. 打开 CAN 总线
  modi_bus_canevo bus;
  int ret = bus.Open(kCanIfname);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::printf("[ERROR] Failed to open bus: %d\n", ret);
    return 1;
  }
  std::printf("[INFO] Bus opened successfully\n");

  // 设置超时时间
  bus.SetSdoTimeoutMs(50);
  bus.SetPdoTimeoutMs(50);

  // 2. 初始化关节
  modi_joint_canevo joint;
  ret = joint.NrtInit(bus, kNodeId);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::printf("[ERROR] Failed to init joint: %d\n", ret);
    bus.Close();
    return 1;
  }
  std::printf("[INFO] Joint initialized (node_id=%d)\n", kNodeId);

  // 4. 设置工作模式为 CSP 并使能关节
  ret = joint.RtEnable(CanEvoMode::kCsp);
  if (ret != static_cast<int>(CanEvoError::kOk)) {
    std::printf("[ERROR] Failed to enable joint: %d\n", ret);
    joint.NrtDestroy();
    bus.Close();
    return 1;
  }
  std::printf("[INFO] Joint enabled (CSP mode)\n");

  // 等待伺服进入运行状态
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::printf("\n[INFO] Starting control loop...\n\n");

  // 6. 控制循环
  const auto period_ms = 1000 / kControlRateHz;
  const auto start_time = std::chrono::steady_clock::now();
  const auto duration_ms = kDurationSec * 1000;
  uint32_t cycle_count = 0;
  uint8_t sync_counter = 0;

  while (true) {
    auto now = std::chrono::steady_clock::now();
    auto elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time)
            .count();

    if (elapsed >= duration_ms) {
      break;
    }

    // 发送 SYNC 帧（每个控制周期一次）
    ret = bus.RtSendSync(sync_counter++);
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::printf("[WARN] Failed to send SYNC: %d\n", ret);
    }

    // 读取关节状态（每 100 个周期打印一次）
    JointStatus status;
    const float t = static_cast<float>(elapsed) / 1000.0f;
    if (joint.RtGetJointStatus(status) ==
        static_cast<int>(CanEvoError::kOk)) {
      if (cycle_count % (kControlRateHz / 10) == 0) {  // 每 100ms 打印一次
        std::printf(
            "[Status] t=%.2fs, actual=%.4f rad, vel=%.4f rad/s, "
            "cur=%.3f A, state=0x%02X\n",
            t, status.actual_pos_rad, status.actual_vel_rads,
            status.actual_cur_a,
            static_cast<uint8_t>(joint.RtGetServoState()));
      }
    }

    // 计算目标位置（正弦波）
    const float target_pos_rad =
        kAmplitudeRad * std::sin(2.0f * M_PI * kWaveFreqHz * t);

    // 设置 CSP 目标位置
    ret = joint.RtSetCspTargetPosition(target_pos_rad);
    if (ret != static_cast<int>(CanEvoError::kOk)) {
      std::printf("[WARN] Failed to set target position: %d\n", ret);
    }

    // 检查 EMCY 故障
    CanEvoFault fault;
    if (joint.RtGetEmcy(fault)) {
      std::printf("[EMCY] Fault detected: 0x%04X\n",
                  static_cast<uint16_t>(fault));
    }

    cycle_count++;

    // 控制周期延时
    std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
  }

  std::printf("\n[INFO] Control loop finished (%u cycles)\n", cycle_count);

  // 7. 失能关节
  joint.RtDisable();
  std::printf("[INFO] Joint disabled\n");

  // 8. 清理资源
  joint.NrtDestroy();
  bus.Close();
  std::printf("[INFO] Resources cleaned up\n");

  return 0;
}
