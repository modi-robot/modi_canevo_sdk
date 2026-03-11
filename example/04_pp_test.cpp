/**
 * @file 04_pp_test.cpp
 * @brief PP模式测试 - 目标360°，轮廓参数全10
 * 
 * 参数：
 * - 轮廓速度：10 °/秒
 * - 轮廓加速度：10 °/秒²
 * - 轮廓减速度：10 °/秒²
 * - 目标位置：360°
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
    
     // 失能关节
    joint.NrtDisable(); 
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 设置机械零点
    joint.NrtSetMechZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 获取机械零点成功标志（通过SDO读取）
    joint.NrtGetMechZeroOk();  // 只触发SDO读取，不关心返回值
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 使能PP模式
    joint.NrtEnable(CanEvoMode::kPp);
    joint.RtSetPpTargetPosition(0.0f, 1.0f, 1.0f, 1.0f);
    bus.RtSendSync(sync++);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 运动到360°
    joint.RtSetPpTargetPosition(6.3f, 1.0f, 1.0f, 1.0f);
    bus.RtSendSync(sync++);
    std::this_thread::sleep_for(std::chrono::seconds(9));
    
    // 运动回0°
    joint.RtSetPpTargetPosition(0.0f, 1.0f, 1.0f, 1.0f);
    bus.RtSendSync(sync++);
    std::this_thread::sleep_for(std::chrono::seconds(9));
    
    // 失能关节
    joint.NrtDisable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    joint.NrtDestroy();
    bus.Close();
    
    return 0;
}
