/**
 * @file 01_bus_open_close.cpp
 * @brief 最简示例: 打开/关闭CAN总线，初始化/销毁关节
 */

#include "modi_joint_canevo.h"
#include <iostream>

int main() {
    std::cout << "\n========== 示例 01: 总线打开/关闭 ==========\n" << std::endl;

    // 1. 创建总线对象
    modi_bus_canevo bus;

    // 2. 打开 CAN 总线
    std::cout << "正在打开 CAN 总线 can0 ... " << std::flush;
    if (bus.Open("can0") != 0) {
        std::cerr << "失败" << std::endl;
        return -1;
    }
    std::cout << "成功" << std::endl;

    // 3. 创建关节对象并初始化（绑定到节点 ID 1）
    modi_joint_canevo joint;
    std::cout << "正在初始化关节 (ID=1) ... " << std::flush;
    if (joint.NrtInit(bus, 1) != 0) {
        std::cerr << "失败" << std::endl;
        bus.Close();
        return -1;
    }
    std::cout << "成功" << std::endl;

    // 4. 清理资源
    std::cout << "正在清理资源 ... " << std::flush;
    joint.NrtDestroy();  // 从总线注销关节
    bus.Close();         // 关闭 CAN 总线
    std::cout << "完成" << std::endl;

    std::cout << "\n========== 示例执行成功 ==========\n" << std::endl;
    return 0;
}
