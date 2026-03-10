/**
 * @file 03_csv_test.cpp
 * @brief CSV速度模式测试 (10 rpm 正转3秒 → 反转3秒)
 * 
 * 测试内容：
 * 1. 使能CSV模式
 * 2. 9.55 rpm 正转3秒
 * 3. 暂停1秒
 * 4. 9.55 rpm 反转3秒
 * 5. 停止
 */

#include "modi_joint_canevo.h"
#include <thread>
#include <chrono>

int main() {
    modi_bus_canevo bus;
    modi_joint_canevo joint;
    
    bus.Open("can0");
    joint.NrtInit(bus, 1);
    
    uint8_t sync = 0;
    
    // 使能CSV模式
    joint.NrtEnable(CanEvoMode::kCsv);
    joint.RtSetCsvTargetVelocity(0.0f);
    bus.RtSendSync(sync++);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
   
    float target_rads = 1.0f;  
    auto start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(
           std::chrono::steady_clock::now() - start_time).count() < 3) {
        bus.RtSendSync(sync++);
        joint.RtSetCsvTargetVelocity(target_rads);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 暂停1秒
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(
           std::chrono::steady_clock::now() - start_time).count() < 1) {
        bus.RtSendSync(sync++);
        joint.RtSetCsvTargetVelocity(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 9.55 rpm 反转3秒
    target_rads = -1.0f;  // -1.0 rad/s
    start_time = std::chrono::steady_clock::now();
    while (std::chrono::duration_cast<std::chrono::seconds>(
           std::chrono::steady_clock::now() - start_time).count() < 3) {
        bus.RtSendSync(sync++);
        joint.RtSetCsvTargetVelocity(target_rads);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 停止
    for (int i = 0; i < 10; i++) {
        bus.RtSendSync(sync++);
        joint.RtSetCsvTargetVelocity(0.0f);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    
    // 失能关节
    joint.NrtDisable();
    bus.RtSendSync(sync++);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    joint.NrtDestroy();
    bus.Close();
    
    return 0;
}
