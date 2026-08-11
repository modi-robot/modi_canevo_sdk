/**
 * @file example_sdo.cpp
 * @brief 最简示例: 打开/关闭CAN总线，初始化/销毁关节
 *
 * 运行前请先在外部手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 * Open() 只打开已配置好的 SocketCAN 接口。
 */

#include <iostream>

#include "modi_joint_canevo.h"

int main() {
  std::cout << "\n========== 示例 01: 总线打开/关闭 ==========\n" << std::endl;

  // 1. 创建总线对象
  modi_bus_canevo bus;

  // 2. 打开 CAN 总线
  TaskConfig cfg;
  cfg.sync_period_us = 5000;
  cfg.cpu_affinity = 2;
  if (bus.Open("can0", cfg) != 0) {
    std::cerr << "打开 CAN 总线 can0 失败" << std::endl;
    return -1;
  }
  std::cout << "打开 CAN 总线 can0 成功" << std::endl;
  const auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "发现 " << joint_ids.size() << " 个关节, ID: ";
  for (const auto id : joint_ids) {
    std::cout << static_cast<int>(id) << " ";
  }
  std::cout << std::endl;

  const uint8_t node_id = joint_ids.front();
  std::cout << "默认选择第一个关节 ID: " << static_cast<int>(node_id)
            << std::endl;

  // 3. 创建关节对象并初始化（绑定到扫描到的第一个节点）
  modi_joint_canevo joint;
  if (joint.NrtInit(bus, node_id) != 0) {
    std::cerr << "初始化关节 (ID=" << static_cast<int>(node_id) << ") 失败"
              << std::endl;
    bus.Close();
    return -1;
  }
  std::cout << "初始化关节 (ID=" << static_cast<int>(node_id) << ") 成功"
            << std::endl;

  // 3. 直接读取
  std::cout << "协议版本: " << joint.NrtGetProtocolVersion() << std::endl;
  std::cout << "关节代号: " << joint.NrtGetModuleType() << std::endl;
  std::cout << "厂商代号: " << joint.NrtGetVendorCode() << std::endl;

  uint32_t global_id[4] = {0};
  if (joint.NrtGetGlobalId(global_id) == 0) {
    std::cout << "全球ID: " << std::hex << global_id[0] << "-" << global_id[1]
              << "-" << global_id[2] << "-" << global_id[3] << std::dec
              << std::endl;
  }

  std::cout << "故障代码: " << static_cast<uint16_t>(joint.NrtGetFaultCode())
            << std::endl;
  std::cout << "警告代码: " << static_cast<uint16_t>(joint.NrtGetWarnCode())
            << std::endl;
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
  std::cout << "电流环比例增益(Hz): " << joint.NrtGetCtrlParam(0x09)
            << std::endl;

  std::cout << "电机编码器分辨率(P/R): " << joint.NrtGetMotorEncRes()
            << std::endl;
  std::cout << "输出轴编码器分辨率(P/R): " << joint.NrtGetOutputEncRes()
            << std::endl;

  std::cout << "实际位置(rad): " << joint.NrtGetActualPosition() << std::endl;
  std::cout << "实际速度(rad/s): " << joint.NrtGetActualVelocity() << std::endl;
  std::cout << "实际电流(A): " << joint.NrtGetActualCurrent() << std::endl;
  std::cout << "母线电压(V): " << joint.NrtGetBusVoltage() << std::endl;
  std::cout << "PCB温度(摄氏度): " << joint.NrtGetPcbTemperature() << std::endl;
  std::cout << "电机温度(摄氏度): " << joint.NrtGetMotorTemperature()
            << std::endl;

  // 4. 清理资源
  std::cout << "正在清理资源 ... " << std::flush;
  joint.NrtDestroy();  // 从总线注销关节
  bus.Close();         // 关闭 CAN 总线

  std::cout << "\n========== 示例执行完成 ==========\n" << std::endl;
  return 0;
}
