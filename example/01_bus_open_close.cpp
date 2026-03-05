// 01_bus_test.cpp
#include "modi_joint_canevo.h"
#include <iostream>
#include <thread>

void PrintStatus(const modi_bus_canevo& bus, const std::string& msg) {
    std::cout << "[" << msg << "] IsOpen: " << bus.IsOpen() << std::endl;
}

int main() {
    std::cout << "\n========== CanEvo 总线测试 ==========\n" << std::endl;
    
    // ----- 1. 打开总线 -----
    modi_bus_canevo bus;
    PrintStatus(bus, "初始状态");
    
    std::cout << "\n>> 打开 can0 ..." << std::endl;
    if (bus.Open("can0") != 0) {
        std::cout << "× 打开失败" << std::endl;
        return -1;
    }
    std::cout << "√ 打开成功" << std::endl;
    PrintStatus(bus, "打开后");
    
    bus.SetSdoTimeoutMs(100);
    bus.SetPdoTimeoutMs(50);
    
    // ----- 2. 初始化关节 -----
    modi_joint_canevo joint;
    std::cout << "\n>> 初始化关节 (ID=1) ..." << std::endl;
    if (joint.NrtInit(bus, 1) != 0) {
        std::cout << "× 初始化失败" << std::endl;
        bus.Close();
        return -1;
    }
    std::cout << "√ 初始化成功" << std::endl;
    
    // ----- 3. 使能关节 -----
    std::cout << "\n>> 使能关节 ..." << std::endl;
    if (joint.RtEnable(CanEvoMode::kCsp) == 0)
        std::cout << "√ 使能成功" << std::endl;
    else
        std::cout << "× 使能失败" << std::endl;
    
    // ----- 4. 读取状态 -----
    std::cout << "\n>> 读取关节状态 ..." << std::endl;
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
        std::cout << "√ 状态读取成功" << std::endl;
        std::cout << "\n----- 关节状态 -----" << std::endl;
        std::cout << "  状态字: 0x" << std::hex << status.statusword << std::dec << std::endl;
        
        std::cout << "  伺服状态: ";
        switch(status.statusword & 0x0F) {
            case 0x00: std::cout << "初始化"; break;
            case 0x01: std::cout << "禁止使能"; break;
            case 0x02: std::cout << "准备使能"; break;
            case 0x03: std::cout << "运行中"; break;
            case 0x04: std::cout << "快速停机"; break;
            case 0x06: std::cout << "故障响应"; break;
            case 0x07: std::cout << "故障"; break;
            default: std::cout << "未知"; break;
        }
        std::cout << "\n  实际位置: " << status.actual_pos_rad << " rad"
                  << "\n  实际速度: " << status.actual_vel_rads << " rad/s"
                  << "\n  实际电流: " << status.actual_cur_a << " A"
                  << "\n  母线电压: " << status.bus_voltage_v << " V"
                  << "\n  PCB温度: " << status.pcb_temp_c << " ℃"
                  << "\n  电机温度: " << status.motor_temp_c << " ℃" << std::endl;
    } else {
        std::cout << "× 状态读取失败" << std::endl;
    }
    
    // ----- 5. 清理 -----
    std::cout << "\n>> 清理资源 ..." << std::endl;
    joint.RtDisable();
    joint.NrtDestroy();
    bus.Close();
    PrintStatus(bus, "关闭后");
    
    std::cout << "\n============ 测试完成 ============\n" << std::endl;
    return 0;
}
