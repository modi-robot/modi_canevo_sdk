/**
 * @file 04_pp_test.cpp
 * @brief PP模式测试 - 通过 SDO 一次性下发轮廓参数，到达后退出
 */

#include <time.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = 3.1415926f / 180.0f;

int main() {
    modi_bus_canevo bus;
    if (bus.Open("can0") != 0) {
        std::cerr << "打开失败" << std::endl;
        return -1;
    }

    modi_joint_canevo joint;
    if (joint.NrtInit(bus, 1) != 0) {
        std::cerr << "初始化失败" << std::endl;
        bus.Close();
        return -1;
    }

    joint.NrtDisable();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    joint.NrtSetMechZero();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    joint.NrtEnable(CanEvoMode::kPp);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int ret = joint.NrtSetPpTargetPosition(6.283f, 0.524f, 0.524f, 0.524f);
    if (ret != 0) {
        std::cerr << "下发PP目标失败, ret=" << ret << std::endl;
        joint.NrtDisable();
        joint.NrtDestroy();
        bus.Close();
        return -1;
    }

    while (true) {
        float pos = joint.NrtGetActualPosition();
        if (std::abs(pos - 6.283f) < 0.017f) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    joint.NrtDisable();
    joint.NrtDestroy();
    bus.Close();

    std::cout << "到达360°，完成" << std::endl;
    return 0;
}
