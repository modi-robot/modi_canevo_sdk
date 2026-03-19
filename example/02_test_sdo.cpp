/**
 * @file 02_test_sdo.cpp
 * @brief SDO读取示例: 读取关节所有参数并打印
 */

#include "modi_joint_canevo.h"
#include <iostream>
#include <iomanip>
#include <thread>

int main() {
    std::cout << "\n========== 示例 02: SDO 参数读取 ==========\n" << std::endl;

    // 1. 打开总线
    modi_bus_canevo bus;
    if (bus.Open("can0") != 0) {
        std::cerr << "打开 CAN 总线失败" << std::endl;
        return -1;
    }
    bus.SetSdoTimeoutMs(100);

    // 2. 初始化关节
    modi_joint_canevo joint;
    if (joint.NrtInit(bus, 1) != 0) {
        std::cerr << "关节初始化失败" << std::endl;
        bus.Close();
        return -1;
    }

    // 3. 直接读取
    std::cout << "协议版本: " << joint.NrtGetProtocolVersion() << std::endl;
    std::cout << "关节代号: " << joint.NrtGetModuleType() << std::endl;
    std::cout << "厂商代号: " << joint.NrtGetVendorCode() << std::endl;
    
    uint32_t global_id[4] = {0};
    if (joint.NrtGetGlobalId(global_id) == 0) {
        std::cout << "全球ID: " << std::hex 
                  << global_id[0] << "-" << global_id[1] << "-" 
                  << global_id[2] << "-" << global_id[3] << std::dec << std::endl;
    }
    
    std::cout << "故障代码: " << static_cast<uint16_t>(joint.NrtGetFaultCode()) << std::endl;
    std::cout << "警告代码: " << static_cast<uint16_t>(joint.NrtGetWarnCode()) << std::endl;
    std::cout << "诊断信息: " << joint.NrtGetDiagInfo() << std::endl;
    std::cout << "CAN波特率编码: " << joint.NrtGetCanBaud() << std::endl;
    std::cout << "CAN ID: " << joint.NrtGetCanId() << std::endl;
    std::cout << "固件版本: " << joint.NrtGetFwVersion() << std::endl;
    std::cout << "硬件版本: " << joint.NrtGetHwVersion() << std::endl;
    std::cout << "同步周期(us): " << joint.NrtGetSyncPeriod() << std::endl;
    std::cout << "通信超时(ms): " << joint.NrtGetCommTimeout() << std::endl;

    uint16_t num = 0, den = 0;
    if (joint.NrtGetGearRatio(num, den) == 0) {
        std::cout << "减速比: " << num << "/" << den << std::endl;
    }
    std::cout << "最大速度(rad/s): " << joint.NrtGetMaxSpeed() << std::endl;
    std::cout << "最大加速度(rad/s²): " << joint.NrtGetMaxAccel() << std::endl;
    std::cout << "最大减速度(rad/s²): " << joint.NrtGetMaxDecel() << std::endl;
    std::cout << "急停减速度(rad/s²): " << joint.NrtGetEstopDecel() << std::endl;
    std::cout << "最大位置(rad): " << joint.NrtGetMaxPos() << std::endl;
    std::cout << "最小位置(rad): " << joint.NrtGetMinPos() << std::endl;
    std::cout << "机械零点标志: " << joint.NrtGetMechZeroOk() << std::endl;
    std::cout << "位置限制状态: " << joint.NrtGetPosLimitStatus() << std::endl;
    std::cout << "抱闸状态: " << joint.NrtGetBrakeStatus() << std::endl;
    std::cout << "参数保存标志: " << joint.NrtGetSaveParamsOk() << std::endl;

    std::cout << "速度环增益(Hz): " << joint.NrtGetCtrlParam(0x00) << std::endl;
    std::cout << "电流环比例增益(Hz): " << joint.NrtGetCtrlParam(0x09) << std::endl;

    std::cout << "电机编码器分辨率(P/R): " << joint.NrtGetMotorEncRes() << std::endl;
    std::cout << "输出轴编码器分辨率(P/R): " << joint.NrtGetOutputEncRes() << std::endl;

    std::cout << "实际位置(rad): " << joint.NrtGetActualPosition() << std::endl;
    std::cout << "实际速度(rad/s): " << joint.NrtGetActualVelocity() << std::endl;
    std::cout << "实际电流(A): " << joint.NrtGetActualCurrent() << std::endl;
    std::cout << "母线电压(V): " << joint.NrtGetBusVoltage() << std::endl;
    std::cout << "PCB温度(℃): " << joint.NrtGetPcbTemperature() << std::endl;
    std::cout << "电机温度(℃): " << joint.NrtGetMotorTemperature() << std::endl;

    // 4. 清理
    joint.NrtDestroy();
    bus.Close();

    std::cout << "\n完成" << std::endl;
    return 0;
}
