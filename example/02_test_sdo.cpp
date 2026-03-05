/**
 * @file 02_test_pcb_temperature.cpp
 * @brief 测试PCB温度读取
 */

#define _USE_MATH_DEFINES
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#include "modi_joint_canevo.h"
#include <iostream>
#include <iomanip>
#include <thread>
#include <vector>
#include <map>
#include <cstdio>

// ==================== 打印辅助函数 ====================
void PrintTitle(const std::string& title) {
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "  " << title << "\n";
    std::cout << std::string(50, '=') << "\n";
}

void PrintResult(const std::string& name, const std::string& value, bool success) {
    std::cout << "\n";
    std::cout << "  " << std::left << std::setw(16) << name << " : ";
    std::cout << std::left << std::setw(20) << value;
    std::cout << (success ? " ✓" : " ✗");
}

// ==================== 主函数 ====================
int main() {
    // 初始化
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "     CanEvo SDO 全部读取接口测试\n";
    std::cout << std::string(50, '=') << "\n";
    
    // ----- 1. 打开总线 -----
    modi_bus_canevo bus;
    std::cout << "\n【初始化】";
    std::cout << "\n  - 打开 can0 ... " << std::flush;
    
    if (bus.Open("can0") != 0) {
        std::cout << "失败" << std::endl;
        return -1;
    }
    std::cout << "成功";
    bus.SetSdoTimeoutMs(100);
    
    // ----- 2. 初始化关节 -----
    modi_joint_canevo joint;
    std::cout << "\n  - 初始化关节 (ID=1) ... " << std::flush;
    
    if (joint.NrtInit(bus, 1) != 0) {
        std::cout << "失败" << std::endl;
        bus.Close();
        return -1;
    }
    std::cout << "成功\n";
    
    // ==================== 1. 设备信息 ====================
    PrintTitle("1. 设备信息 (Index 0x00)");
    
    // 协议版本
    {
        uint16_t proto = joint.NrtGetProtocolVersion();
        if (proto != 0) {
            uint16_t major = (proto >> 12) & 0xF;
            uint16_t minor = (proto >> 7) & 0x1F;
            uint16_t bugfix = proto & 0x7F;
            PrintResult("协议版本", "v" + std::to_string(major) + "." + 
                       std::to_string(minor) + "." + std::to_string(bugfix), true);
        }
    }
    
    // 关节代号
    {
        uint32_t module = joint.NrtGetModuleType();
        if (module != 0) {
            char str[20];
            sprintf(str, "0x%08X", module);
            PrintResult("关节代号", str, true);
        }
    }
    
    // 厂商代码
    {
        uint32_t vendor = joint.NrtGetVendorCode();
        if (vendor != 0) {
            char str[20];
            sprintf(str, "0x%08X", vendor);
            std::string result = str;
            // ASCII解析
            if (vendor >= 0x20 && vendor <= 0x7E) {
                result += " ('";
                result += (char)(vendor & 0xFF);
                result += (char)((vendor >> 8) & 0xFF);
                result += (char)((vendor >> 16) & 0xFF);
                result += (char)((vendor >> 24) & 0xFF);
                result += "')";
            }
            PrintResult("厂商代码", result, true);
        }
    }
    
    // 全球ID
    {
        uint32_t id[4] = {0};
        if (joint.NrtGetGlobalId(id) == 0) {
            char str[100];
            sprintf(str, "%08X-%08X-%08X-%08X", id[0], id[1], id[2], id[3]);
            PrintResult("全球ID", str, true);
        }
    }
    
    // 故障代码
    {
        CanEvoFault fault = joint.NrtGetFaultCode();
        char str[20];
        sprintf(str, "0x%04X", static_cast<uint16_t>(fault));
        PrintResult("故障代码", str, true);
    }
    
    // 警告代码
    {
        CanEvoWarn warn = joint.NrtGetWarnCode();
        char str[20];
        sprintf(str, "0x%04X", static_cast<uint16_t>(warn));
        PrintResult("警告代码", str, true);
    }
    
    // 诊断信息
    {
        uint16_t diag = joint.NrtGetDiagInfo();
        if (diag != 0) {
            char str[20];
            sprintf(str, "0x%04X", diag);
            PrintResult("诊断信息", str, true);
        }
    }
    
    // CAN波特率
    {
        uint16_t baud = joint.NrtGetCanBaud();
        if (baud <= 6) {
            const char* names[] = {"250k", "500k", "1M", "2M", "3M", "4M", "5M"};
            PrintResult("CAN波特率", names[baud], true);
        }
    }
    
    // CAN ID
    {
        uint16_t id = joint.NrtGetCanId();
        if (id >= 1 && id <= 62) {
            PrintResult("CAN ID", std::to_string(id), true);
        }
    }
    
    // 固件版本
    {
        uint16_t fw = joint.NrtGetFwVersion();
        if (fw != 0) {
            uint16_t major = (fw >> 12) & 0xF;
            uint16_t minor = (fw >> 7) & 0x1F;
            uint16_t bugfix = fw & 0x7F;
            PrintResult("固件版本", "v" + std::to_string(major) + "." + 
                       std::to_string(minor) + "." + std::to_string(bugfix), true);
        }
    }
    
    // 硬件版本
    {
        uint16_t hw = joint.NrtGetHwVersion();
        if (hw != 0) {
            uint16_t major = (hw >> 12) & 0xF;
            uint16_t minor = (hw >> 7) & 0x1F;
            uint16_t bugfix = hw & 0x7F;
            PrintResult("硬件版本", "v" + std::to_string(major) + "." + 
                       std::to_string(minor) + "." + std::to_string(bugfix), true);
        }
    }
    
    // 同步周期
    {
        uint16_t sync = joint.NrtGetSyncPeriod();
        if (sync > 0) {
            PrintResult("同步周期", std::to_string(sync) + " us", true);
        }
    }
    
    // 通信超时
    {
        uint16_t timeout = joint.NrtGetCommTimeout();
        if (timeout > 0) {
            PrintResult("通信超时", std::to_string(timeout) + " ms", true);
        }
    }
    
    // ==================== 2. 关节参数 ====================
    PrintTitle("2. 关节参数 (Index 0x01)");
    
    // 减速比
    {
        uint16_t num = 0, den = 0;
        if (joint.NrtGetGearRatio(num, den) == 0 && num > 0 && den > 0) {
            char str[50];
            sprintf(str, "%d/%d = %.3f", num, den, (float)num/den);
            PrintResult("减速比", str, true);
        }
    }
    
    // 最大速度
    {
        float val = joint.NrtGetMaxSpeed();
        if (val > 0) {
            char str[50];
            sprintf(str, "%.3f rad/s (%.1f rpm)", val, val * 60 / (2*M_PI));
            PrintResult("最大速度", str, true);
        }
    }
    
    // 最大加速度
    {
        float val = joint.NrtGetMaxAccel();
        if (val > 0) {
            char str[30];
            sprintf(str, "%.3f rad/s²", val);
            PrintResult("最大加速度", str, true);
        }
    }
    
    // 最大减速度
    {
        float val = joint.NrtGetMaxDecel();
        if (val > 0) {
            char str[30];
            sprintf(str, "%.3f rad/s²", val);
            PrintResult("最大减速度", str, true);
        }
    }
    
    // 急停减速度
    {
        float val = joint.NrtGetEstopDecel();
        if (val > 0) {
            char str[30];
            sprintf(str, "%.3f rad/s²", val);
            PrintResult("急停减速度", str, true);
        }
    }
    
    // 最大位置
    {
        float val = joint.NrtGetMaxPos();
        char str[50];
        sprintf(str, "%.3f rad (%.1f°)", val, val * 180 / M_PI);
        PrintResult("最大位置", str, val != 0);
    }
    
    // 最小位置
    {
        float val = joint.NrtGetMinPos();
        char str[50];
        sprintf(str, "%.3f rad (%.1f°)", val, val * 180 / M_PI);
        PrintResult("最小位置", str, val != 0);
    }
    
    // 机械零点标志
    {
        uint16_t val = joint.NrtGetMechZeroOk();
        PrintResult("机械零点标志", std::to_string(val), true);
    }
    
    // 位置限制状态
    {
        uint16_t val = joint.NrtGetPosLimitStatus();
        PrintResult("位置限制状态", val ? "已使能" : "未使能", true);
    }
    
    // 抱闸状态
    {
        uint16_t val = joint.NrtGetBrakeStatus();
        PrintResult("抱闸状态", val ? "打开" : "关闭", true);
    }
    
    // 参数保存标志
    {
        uint16_t val = joint.NrtGetSaveParamsOk();
        PrintResult("参数保存标志", std::to_string(val), true);
    }
    
    // ==================== 3. 控制器参数 ====================
    PrintTitle("3. 控制器参数 (Index 0x07)");
    
    // 速度环增益
    PrintResult("速度环增益", std::to_string(joint.NrtGetCtrlParam(0x00)) + " Hz", true);
    PrintResult("速度环积分时间", std::to_string(joint.NrtGetCtrlParam(0x01)) + " 0.01ms", true);
    PrintResult("转矩滤波时间", std::to_string(joint.NrtGetCtrlParam(0x02)) + " 0.01ms", true);
    PrintResult("位置环增益", std::to_string(joint.NrtGetCtrlParam(0x03)) + " Hz", true);
    PrintResult("速度检测滤波", std::to_string(joint.NrtGetCtrlParam(0x04)) + " 0.01ms", true);
    PrintResult("位置环前馈增益", std::to_string(joint.NrtGetCtrlParam(0x05)) + " %", true);
    PrintResult("位置环前馈滤波", std::to_string(joint.NrtGetCtrlParam(0x06)) + " 0.01ms", true);
    PrintResult("速度环前馈增益", std::to_string(joint.NrtGetCtrlParam(0x07)) + " %", true);
    PrintResult("速度环前馈滤波", std::to_string(joint.NrtGetCtrlParam(0x08)) + " 0.01ms", true);
    PrintResult("电流环比例增益", std::to_string(joint.NrtGetCtrlParam(0x09)) + " Hz", true);
    PrintResult("电流环积分时间", std::to_string(joint.NrtGetCtrlParam(0x0A)) + " 0.1ms", true);
    
    // ==================== 4. 编码器参数 ====================
    PrintTitle("4. 编码器参数 (Index 0x08)");
    
    // 电机编码器分辨率
    {
        uint32_t val = joint.NrtGetMotorEncRes();
        if (val > 0) {
            char str[30];
            sprintf(str, "%u P/R", val);
            PrintResult("电机编码器分辨率", str, true);
        }
    }
    
    // 输出轴编码器分辨率
    {
        uint32_t val = joint.NrtGetOutputEncRes();
        if (val > 0) {
            char str[30];
            sprintf(str, "%u P/R", val);
            PrintResult("输出轴编码器分辨率", str, true);
        }
    }
    
    // ==================== 5. 实际值 ====================
    PrintTitle("5. 实际值 (Index 0x20)");
    
    // 发送SYNC触发更新
    bus.RtSendSync(0);
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    
    // 实际位置
    {
        float val = joint.NrtGetActualPosition();
        char str[50];
        sprintf(str, "%.4f rad (%.2f°)", val, val * 180 / M_PI);
        PrintResult("实际位置", str, true);
    }
    
    // 实际速度
    {
        float val = joint.NrtGetActualVelocity();
        char str[50];
        sprintf(str, "%.4f rad/s (%.2f rpm)", val, val * 60 / (2*M_PI));
        PrintResult("实际速度", str, true);
    }
    
    // 实际电流
    PrintResult("实际电流", std::to_string(joint.NrtGetActualCurrent()) + " A", true);
    
    // 实际加速度
    {
        float val = joint.NrtGetActualAcceleration();
        char str[30];
        sprintf(str, "%.3f rad/s²", val);
        PrintResult("实际加速度", str, true);
    }
    
    // 实际转矩
    PrintResult("实际转矩", std::to_string(joint.NrtGetActualTorque()) + " Nm", true);
    
    // 位置跟踪误差
    {
        float val = joint.NrtGetPositionTrackingError();
        char str[50];
        sprintf(str, "%.4f rad (%.2f°)", val, val * 180 / M_PI);
        PrintResult("位置跟踪误差", str, true);
    }
    
    // 母线电压
    {
        float val = joint.NrtGetBusVoltage();
        char str[30];
        sprintf(str, "%.2f V", val);
        PrintResult("母线电压", str, val > 0);
    }
    
    // PCB温度
    {
        float val = joint.NrtGetPcbTemperature();
        char str[30];
        sprintf(str, "%.1f ℃", val);
        PrintResult("PCB温度", str, val > 0);
    }
    
    // 电机温度
    {
        float val = joint.NrtGetMotorTemperature();
        char str[30];
        sprintf(str, "%.1f ℃", val);
        PrintResult("电机温度", str, val > 0);
    }
    
    // 减速器温度
    {
        float val = joint.NrtGetGearboxTemperature();
        char str[30];
        sprintf(str, "%.1f ℃", val);
        PrintResult("减速器温度", str, val > 0);
    }
    
    // 电角度
    {
        float val = joint.NrtGetElectricalAngle();
        char str[50];
        sprintf(str, "%.1f rad (%.1f°)", val, val * 180 / M_PI);
        PrintResult("电角度", str, true);
    }
    
    // ==================== 6. 清理 ====================
    std::cout << "\n" << std::string(50, '=') << "\n";
    std::cout << "  清理资源 ... " << std::flush;
    
    joint.NrtDestroy();
    bus.Close();
    
    std::cout << "完成\n";
    std::cout << std::string(50, '=') << "\n" << std::endl;
    
    return 0;
}
