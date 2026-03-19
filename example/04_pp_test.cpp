/**
 * @file 04_pp_test.cpp
 * @brief PP模式测试 - 目标360°，到达后退出
 */

#include <time.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <cmath>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = 3.1415926f / 180.0f;

int main() {
    // 1. 打开总线
    modi_bus_canevo bus;
    if (bus.Open("can0") != 0) {
        std::cerr << "打开失败" << std::endl;
        return -1;
    }

    // 2. 初始化关节
    modi_joint_canevo joint;
    if (joint.NrtInit(bus, 1) != 0) {
        std::cerr << "初始化失败" << std::endl;
        bus.Close();
        return -1;
    }

    // 3. 失能关节
    joint.NrtDisable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    // 4. 设置机械零点
    joint.NrtSetMechZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 5. 使能PP模式
    joint.NrtEnable(CanEvoMode::kPp);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // 6. 启动控制循环
    bus.StartControlLoop([&]() {
        joint.RtSetPpTargetPosition(6.283f, 0.524f, 0.524f, 0.524f);
    });

    // 7. 等待到达360°
    JointStatus status;
    while (true) {
        if (joint.RtGetJointStatus(status) == 0) {
            if (std::abs(status.actual_pos_rad - 6.283f) < 0.017f) {  // 小于1°
                break;
            }
        }
    }

    // 8. 清理
    bus.Close();
    joint.NrtDisable();
    joint.NrtDestroy();

    std::cout << "到达360°，完成" << std::endl;
    return 0;
}
