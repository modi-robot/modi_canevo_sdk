/**
 * @file 05_csp_test.cpp
 * @brief CSP测试demo
 * 
 * 运动参数：
 * - 速度：30 °/s
 * - 距离：0° → 360° → 0°
 * - 单程时间：12秒
 */

#include "modi_joint_canevo.h"
#include <thread>
#include <chrono>
#include <cmath>

constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr int PERIOD_MS = 5;

constexpr float SPEED_DEG_PER_SEC = 30.0f;
constexpr float SPEED_RAD_PER_SEC = SPEED_DEG_PER_SEC * kDegToRad;
constexpr float TARGET_360_DEG = 360.0f;
constexpr float TARGET_360_RAD = TARGET_360_DEG * kDegToRad;
constexpr float TARGET_0_RAD = 0.0f;

constexpr float TIME_TO_360 = TARGET_360_DEG / SPEED_DEG_PER_SEC;
constexpr int STEPS_TO_360 = static_cast<int>(TIME_TO_360 * 1000 / PERIOD_MS);

int main() {
    modi_bus_canevo bus;
    modi_joint_canevo joint;
    
    bus.Open("can0");
    bus.SetSdoTimeoutMs(50);
    bus.SetPdoTimeoutMs(50);
    
    joint.NrtInit(bus, 1);
    
    uint8_t sync = 0;
    JointStatus status;
    
    // === 设置机械零点 ===
    joint.NrtDisable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    joint.NrtSetMechZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 获取当前位置
    for (int i = 0; i < 10; i++) {
        bus.RtSendSync(sync++);
        std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD_MS));
    }
    
    float current_pos_rad = 0.0f;
    joint.RtGetJointStatus(status);
    current_pos_rad = status.actual_pos_rad;
    
    // 使能
    joint.NrtEnable(CanEvoMode::kCsp);
    
    // 等待进入运行状态
    int wait = 0;
    while (joint.RtGetServoState() != CanEvoServoState::kRunning && wait < 50) {
        bus.RtSendSync(sync++);
        joint.RtSetCspTargetPosition(current_pos_rad);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        wait++;
    }
    
    // 0° → 360°
    for (int step = 0; step <= STEPS_TO_360; step++) {
        float target_rad = current_pos_rad + (TARGET_360_RAD - current_pos_rad) * step / STEPS_TO_360;
        
        bus.RtSendSync(sync++);
        joint.RtSetCspTargetPosition(target_rad);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD_MS));
    }
    
    // 保持位置2秒
    for (int i = 0; i < 400; i++) {
        bus.RtSendSync(sync++);
        joint.RtSetCspTargetPosition(TARGET_360_RAD);
        std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD_MS));
    }
    
    // 360° → 0°
    for (int step = 0; step <= STEPS_TO_360; step++) {
        float target_rad = TARGET_360_RAD - (TARGET_360_RAD - TARGET_0_RAD) * step / STEPS_TO_360;
        
        bus.RtSendSync(sync++);
        joint.RtSetCspTargetPosition(target_rad);
        
        std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD_MS));
    }
    
    // 失能
    joint.NrtDisable();
    for (int i = 0; i < 10; i++) {
        bus.RtSendSync(sync++);
        joint.RtSetCspTargetPosition(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(PERIOD_MS));
    }
    
    joint.NrtDestroy();
    bus.Close();
    
    return 0;
}
