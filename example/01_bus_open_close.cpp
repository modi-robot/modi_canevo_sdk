/**
 * @file 01_bus_test_simple.cpp
 * @brief 总线测试 
 */

#include "modi_joint_canevo.h"
#include <iostream>
#include <thread>

int main() {
    std::cout << "\n========== CanEvo 总线测试 ==========\n" << std::endl;
    
    // 1. 打开总线
    modi_bus_canevo bus;
    if (bus.Open("can0") != 0) {
        std::cout << "❌ 打开失败" << std::endl;
        return -1;
    }
    std::cout << "✅ 打开成功" << std::endl;
    
    bus.SetSdoTimeoutMs(100);
    bus.SetPdoTimeoutMs(50);
    
    // 2. 初始化关节
    modi_joint_canevo joint;
    if (joint.NrtInit(bus, 1) != 0) {
        std::cout << "❌ 初始化失败" << std::endl;
        bus.Close();
        return -1;
    }
    std::cout << "✅ 初始化成功" << std::endl;
    
    // 3. 使能关节
    if (joint.NrtEnable(CanEvoMode::kCsp) == 0)
        std::cout << "✅ 使能成功" << std::endl;
    else
        std::cout << "❌ 使能失败" << std::endl;
    
    // 4. 发送使能生效帧
    for (int i = 0; i < 5; i++) {
        bus.RtSendSync(i);
        joint.RtSetCspTargetPosition(0);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    
    // 5. 读取状态
    JointStatus status;
    bool ok = false;
    for (int i = 0; i < 5; i++) {
        bus.RtSendSync(i);
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        if (joint.RtGetJointStatus(status) == 0) {
            ok = true;
            break;
        }
    }
    
    if (ok) {
        std::cout << "\n【当前状态】" << std::endl;
        std::cout << "  状态字: 0x" << std::hex << status.statusword << std::dec << std::endl;
        std::cout << "  位置: " << status.actual_pos_rad << " rad" << std::endl;
        std::cout << "  速度: " << status.actual_vel_rads << " rad/s" << std::endl;
        std::cout << "  电流: " << status.actual_cur_a << " A" << std::endl;
    } else {
        std::cout << "❌ 状态读取失败" << std::endl;
    }
    
    // 6. 清理
    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();
    std::cout << "\n✅ 测试完成\n" << std::endl;
    
    return 0;
}
