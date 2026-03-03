/**
 * @file    modi_joint_canevo.cpp
 * @brief   公开类 modi_bus_canevo / modi_joint_canevo 实现（PIMPL 转发层）
 *
 * @protocol CanEvo V1.2.3
 * @version  1.0
 * @date     2026-02-26
 */

#include "modi_joint_canevo.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <thread>

#include "canevo_impl.h"

using namespace canevo;

/* ---- 单位转换常量 ---- */
constexpr float kRadToDeg = 180.0f / static_cast<float>(M_PI);
constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;
constexpr float kRadsToRpm = 60.0f / (2.0f * static_cast<float>(M_PI));
constexpr float kRpmToRads = (2.0f * static_cast<float>(M_PI)) / 60.0f;

/* ============================================================
 *            modi_bus_canevo
 * ============================================================ */

/* ---- 构造 / 析构 ---- */
modi_bus_canevo::modi_bus_canevo() : impl_(std::make_unique<Impl>()) {}
modi_bus_canevo::~modi_bus_canevo() = default;

/* ---- 接口 ---- */
int modi_bus_canevo::Open(const std::string& can_ifname) {
  return impl_->Open(can_ifname);
}
void modi_bus_canevo::Close() { impl_->Close(); }
bool modi_bus_canevo::IsOpen() const { return impl_->IsOpen(); }
int modi_bus_canevo::RtSendSync(const uint8_t counter) {
  return impl_->SendSync(counter);
}
void modi_bus_canevo::SetSdoTimeoutMs(const int ms) {
  impl_->SetDefaultSdoTimeoutMs(ms);
}
void modi_bus_canevo::SetPdoTimeoutMs(const int ms) {
  impl_->SetDefaultPdoTimeoutMs(ms);
}

/* ============================================================
 *            modi_joint_canevo
 * ============================================================ */

/* ---- 构造 / 析构 ---- */
modi_joint_canevo::modi_joint_canevo() : impl_(std::make_unique<Impl>()) {}
modi_joint_canevo::~modi_joint_canevo() { NrtDestroy(); }

/* ---- 生命周期 ---- */
int modi_joint_canevo::NrtInit(modi_bus_canevo& bus, const uint8_t node_id) {
  if (!bus.IsOpen()) return static_cast<int>(CanEvoError::kBusNotOpen);
  if (node_id == 0 || node_id > 62)
    return static_cast<int>(CanEvoError::kInvalidParam);
  if (impl_->isInitialized()) NrtDestroy();  // 已初始化则先释放

  return impl_->Init(bus.impl_.get(), node_id);
}

void modi_joint_canevo::NrtDestroy() { impl_->Shutdown(); }

uint8_t modi_joint_canevo::NrtNodeId() const {
  return impl_->isInitialized() ? impl_->nodeId() : 0;
}

/* ============================================================
 * PDO 实时控制（非阻塞）
 * ============================================================ */

int modi_joint_canevo::RtSetCspTargetPosition(const float target_pos_rad) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  return impl_->SendRxPdo0(target_pos_rad * kRadToDeg);
}

int modi_joint_canevo::RtSetCsvTargetVelocity(const float target_vel_rads) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  return impl_->SendRxPdo1(target_vel_rads * kRadsToRpm);
}

int modi_joint_canevo::RtSetCstTargetCurrent(const float target_cur_a) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  return impl_->SendRxPdo2(target_cur_a);
}

int modi_joint_canevo::RtSetPpTargetPosition(const float target_pos_rad,
                                             const float profile_vel_rads,
                                             const float profile_acc_radss,
                                             const float profile_dec_radss) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  return impl_->SendRxPdo3(
      target_pos_rad * kRadToDeg, profile_vel_rads * kRadsToRpm,
      profile_acc_radss * kRadsToRpm, profile_dec_radss * kRadsToRpm);
}

int modi_joint_canevo::RtGetJointStatus(JointStatus& out) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  return impl_->GetStatus(out);
}

bool modi_joint_canevo::RtGetEmcy(CanEvoFault& out_fault) {
  if (!impl_->isInitialized()) return false;
  uint16_t raw = 0;
  bool ok = impl_->GetEmcy(raw);
  if (ok) out_fault = static_cast<CanEvoFault>(raw);
  return ok;
}

/* ============================================================
 * controlword 控制（非阻塞，修改内部缓存）
 * ============================================================ */

int modi_joint_canevo::RtEnable() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  impl_->SetEnable(true);
  return static_cast<int>(CanEvoError::kOk);
}

int modi_joint_canevo::RtDisable() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  impl_->SetEnable(false);
  return static_cast<int>(CanEvoError::kOk);
}

int modi_joint_canevo::RtClearFault() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  impl_->TriggerFaultClr();
  return static_cast<int>(CanEvoError::kOk);
}

int modi_joint_canevo::RtSetWorkMode(const CanEvoMode mode) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  impl_->SetMode(mode);
  return static_cast<int>(CanEvoError::kOk);
}

int modi_joint_canevo::RtEstop() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  impl_->SetEstop(true);
  return static_cast<int>(CanEvoError::kOk);
}

int modi_joint_canevo::RtClearEstop() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  impl_->SetEstop(false);
  return static_cast<int>(CanEvoError::kOk);
}

/* ============================================================
 * SDO 控制字操作（阻塞，通过 SDO 直接修改 0x21/0x00）
 * ============================================================
 */

int modi_joint_canevo::NrtEnable() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  uint16_t cw = 0;
  int ret = impl_->sdoReadU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  cw = static_cast<uint16_t>((cw & ~0x0002) | 0x0002);  // bit1 = 1
  return impl_->sdoWriteU16(0x21, 0x00, cw);
}

int modi_joint_canevo::NrtDisable() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  uint16_t cw = 0;
  int ret = impl_->sdoReadU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  cw = static_cast<uint16_t>(cw & ~0x0002);  // bit1 = 0
  return impl_->sdoWriteU16(0x21, 0x00, cw);
}

int modi_joint_canevo::NrtSetWorkMode(const CanEvoMode mode) {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  uint16_t cw = 0;
  int ret = impl_->sdoReadU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  // bit12~bit15 为模式位，清除后设置新值
  cw = static_cast<uint16_t>((cw & ~0xF000) |
                             (static_cast<uint16_t>(mode) << 12));
  return impl_->sdoWriteU16(0x21, 0x00, cw);
}

int modi_joint_canevo::NrtClearFault() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  uint16_t cw = 0;
  int ret = impl_->sdoReadU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  // 脉冲：先置位 bit0
  cw = static_cast<uint16_t>(cw | 0x0001);
  ret = impl_->sdoWriteU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  // 短延时后清零 bit0
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  cw = static_cast<uint16_t>(cw & ~0x0001);
  return impl_->sdoWriteU16(0x21, 0x00, cw);
}

int modi_joint_canevo::NrtEstop() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  uint16_t cw = 0;
  int ret = impl_->sdoReadU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  cw = static_cast<uint16_t>((cw & ~0x0004) | 0x0004);  // bit2 = 1
  return impl_->sdoWriteU16(0x21, 0x00, cw);
}

int modi_joint_canevo::NrtClearEstop() {
  if (!impl_->isInitialized())
    return static_cast<int>(CanEvoError::kNotInitialized);
  uint16_t cw = 0;
  int ret = impl_->sdoReadU16(0x21, 0x00, cw);
  if (ret != 0) return ret;
  cw = static_cast<uint16_t>(cw & ~0x0004);  // bit2 = 0
  return impl_->sdoWriteU16(0x21, 0x00, cw);
}

/* ============================================================
 * SDO 命名化接口 — 设备信息 (Index 0x00)
 * ============================================================ */

uint16_t modi_joint_canevo::NrtGetProtocolVersion() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x01, v);
  return v;
}
uint32_t modi_joint_canevo::NrtGetModuleType() {
  uint32_t v = 0;
  impl_->sdoReadU32(0x00, 0x02, v);
  return v;
}
uint32_t modi_joint_canevo::NrtGetVendorCode() {
  uint32_t v = 0;
  impl_->sdoReadU32(0x00, 0x03, v);
  return v;
}

int modi_joint_canevo::NrtGetGlobalId(uint32_t id[4]) {
  int ret;
  ret = impl_->sdoReadU32(0x00, 0x04, id[0]);
  if (ret) return ret;
  ret = impl_->sdoReadU32(0x00, 0x05, id[1]);
  if (ret) return ret;
  ret = impl_->sdoReadU32(0x00, 0x06, id[2]);
  if (ret) return ret;
  ret = impl_->sdoReadU32(0x00, 0x07, id[3]);
  return ret;
}

CanEvoFault modi_joint_canevo::NrtGetFaultCode() {
  uint16_t raw = 0;
  impl_->sdoReadU16(0x00, 0x08, raw);
  return static_cast<CanEvoFault>(raw);
}

CanEvoWarn modi_joint_canevo::NrtGetWarnCode() {
  uint16_t raw = 0;
  impl_->sdoReadU16(0x00, 0x09, raw);
  return static_cast<CanEvoWarn>(raw);
}

uint16_t modi_joint_canevo::NrtGetDiagInfo() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x0A, v);
  return v;
}

uint16_t modi_joint_canevo::NrtGetCanBaud() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x0B, v);
  return v;
}
int modi_joint_canevo::NrtSetCanBaud(const uint16_t v) {
  return impl_->sdoWriteU16(0x00, 0x0B, v);
}

uint16_t modi_joint_canevo::NrtGetCanId() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x0C, v);
  return v;
}
int modi_joint_canevo::NrtSetCanId(const uint16_t v) {
  return impl_->sdoWriteU16(0x00, 0x0C, v);
}

uint16_t modi_joint_canevo::NrtGetFwVersion() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x0D, v);
  return v;
}
uint16_t modi_joint_canevo::NrtGetHwVersion() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x0E, v);
  return v;
}

uint16_t modi_joint_canevo::NrtGetSyncPeriod() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x0F, v);
  return v;
}
int modi_joint_canevo::NrtSetSyncPeriod(const uint16_t v) {
  return impl_->sdoWriteU16(0x00, 0x0F, v);
}

uint16_t modi_joint_canevo::NrtGetCommTimeout() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x00, 0x10, v);
  return v;
}
int modi_joint_canevo::NrtSetCommTimeout(const uint16_t v) {
  return impl_->sdoWriteU16(0x00, 0x10, v);
}

/* ============================================================
 * SDO 命名化接口 — 关节参数 (Index 0x01)
 * ============================================================ */

int modi_joint_canevo::NrtGetGearRatio(uint16_t& num, uint16_t& den) {
  int ret = impl_->sdoReadU16(0x01, 0x00, num);
  if (ret) return ret;
  return impl_->sdoReadU16(0x01, 0x01, den);
}

float modi_joint_canevo::NrtGetMaxSpeed() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x01, 0x02, v) == 0) v *= kRpmToRads;
  return v;
}
int modi_joint_canevo::NrtSetMaxSpeed(const float v) {
  return impl_->sdoWriteF32(0x01, 0x02, v * kRadsToRpm);
}

float modi_joint_canevo::NrtGetMaxAccel() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x01, 0x03, v) == 0) v *= kRpmToRads;
  return v;
}
int modi_joint_canevo::NrtSetMaxAccel(const float v) {
  return impl_->sdoWriteF32(0x01, 0x03, v * kRadsToRpm);
}

float modi_joint_canevo::NrtGetMaxDecel() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x01, 0x04, v) == 0) v *= kRpmToRads;
  return v;
}
int modi_joint_canevo::NrtSetMaxDecel(const float v) {
  return impl_->sdoWriteF32(0x01, 0x04, v * kRadsToRpm);
}

float modi_joint_canevo::NrtGetEstopDecel() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x01, 0x05, v) == 0) v *= kRpmToRads;
  return v;
}
int modi_joint_canevo::NrtSetEstopDecel(const float v) {
  return impl_->sdoWriteF32(0x01, 0x05, v * kRadsToRpm);
}

float modi_joint_canevo::NrtGetMaxPos() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x01, 0x06, v) == 0) v *= kDegToRad;
  return v;
}
int modi_joint_canevo::NrtSetMaxPos(const float v) {
  return impl_->sdoWriteF32(0x01, 0x06, v * kRadToDeg);
}

float modi_joint_canevo::NrtGetMinPos() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x01, 0x07, v) == 0) v *= kDegToRad;
  return v;
}
int modi_joint_canevo::NrtSetMinPos(const float v) {
  return impl_->sdoWriteF32(0x01, 0x07, v * kRadToDeg);
}

int modi_joint_canevo::NrtSetMechZero() {
  return impl_->sdoWriteU16(0x01, 0x08, 0x0001);
}
uint16_t modi_joint_canevo::NrtGetMechZeroOk() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x01, 0x09, v);
  return v;
}

int modi_joint_canevo::NrtSetPosLimitEnable(const uint16_t v) {
  return impl_->sdoWriteU16(0x01, 0x0A, v);
}
uint16_t modi_joint_canevo::NrtGetPosLimitStatus() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x01, 0x0B, v);
  return v;
}

int modi_joint_canevo::NrtSetBrakeControl(const uint16_t v) {
  return impl_->sdoWriteU16(0x01, 0x0C, v);
}
uint16_t modi_joint_canevo::NrtGetBrakeStatus() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x01, 0x0D, v);
  return v;
}

int modi_joint_canevo::NrtSaveParams() {
  return impl_->sdoWriteU16(0x01, 0xF0, 0x0001);
}
uint16_t modi_joint_canevo::NrtGetSaveParamsOk() {
  uint16_t v = 0;
  impl_->sdoReadU16(0x01, 0xF1, v);
  return v;
}

/* ============================================================
 * SDO 命名化接口 — 控制器参数 (Index 0x07)
 * ============================================================ */

uint16_t modi_joint_canevo::NrtGetCtrlParam(const uint8_t sub) {
  uint16_t v = 0;
  impl_->sdoReadU16(0x07, sub, v);
  return v;
}
int modi_joint_canevo::NrtSetCtrlParam(const uint8_t sub, const uint16_t v) {
  return impl_->sdoWriteU16(0x07, sub, v);
}

/* ============================================================
 * SDO 命名化接口 — 编码器参数 (Index 0x08)
 * ============================================================ */

uint32_t modi_joint_canevo::NrtGetMotorEncRes() {
  uint32_t v = 0;
  impl_->sdoReadU32(0x08, 0x00, v);
  return v;
}
uint32_t modi_joint_canevo::NrtGetOutputEncRes() {
  uint32_t v = 0;
  impl_->sdoReadU32(0x08, 0x01, v);
  return v;
}

/* ============================================================
 * SDO 命名化接口 — PP 模式轮廓参数 (Index 0x21)
 * ============================================================ */

int modi_joint_canevo::NrtSetPpTargetPosition(const float target_pos_rad,
                                              const float profile_vel_rads,
                                              const float profile_acc_radss,
                                              const float profile_dec_radss) {
  int ret = impl_->sdoWriteF32(0x21, 0x02, target_pos_rad * kRadToDeg);
  if (ret != 0) return ret;
  ret = impl_->sdoWriteF32(0x21, 0x07, profile_vel_rads * kRadsToRpm);
  if (ret != 0) return ret;
  ret = impl_->sdoWriteF32(0x21, 0x08, profile_acc_radss * kRadsToRpm);
  if (ret != 0) return ret;
  return impl_->sdoWriteF32(0x21, 0x09, profile_dec_radss * kRadsToRpm);
}

/* ============================================================
 * SDO 命名化接口 — 实际量 (Index 0x20, 只读)
 * ============================================================ */

float modi_joint_canevo::NrtGetActualPosition() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x20, 0x00, v) == 0) v *= kDegToRad;
  return v;
}

float modi_joint_canevo::NrtGetActualVelocity() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x20, 0x01, v) == 0) v *= kRpmToRads;
  return v;
}

float modi_joint_canevo::NrtGetActualCurrent() {
  float v = 0.0f;
  impl_->sdoReadF32(0x20, 0x02, v);
  return v;
}

float modi_joint_canevo::NrtGetActualAcceleration() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x20, 0x03, v) == 0) v *= kRpmToRads;
  return v;
}

float modi_joint_canevo::NrtGetActualTorque() {
  float v = 0.0f;
  impl_->sdoReadF32(0x20, 0x04, v);
  return v;
}

float modi_joint_canevo::NrtGetPositionTrackingError() {
  float v = 0.0f;
  if (impl_->sdoReadF32(0x20, 0x05, v) == 0) v *= kDegToRad;
  return v;
}

float modi_joint_canevo::NrtGetBusVoltage() {
  uint16_t raw = 0;
  if (impl_->sdoReadU16(0x20, 0x06, raw) == 0) {
    return static_cast<float>(raw) * 0.01f;  // raw * 0.01 -> V
  }
  return 0.0f;
}

float modi_joint_canevo::NrtGetPcbTemperature() {
  int16_t raw = 0;
  if (impl_->sdoReadI16(0x20, 0x07, raw) == 0) {
    return static_cast<float>(raw) * 0.1f;  // raw * 0.1 -> ℃
  }
  return 0.0f;
}

float modi_joint_canevo::NrtGetMotorTemperature() {
  int16_t raw = 0;
  if (impl_->sdoReadI16(0x20, 0x08, raw) == 0) {
    return static_cast<float>(raw) * 0.1f;  // raw * 0.1 -> ℃
  }
  return 0.0f;
}

float modi_joint_canevo::NrtGetGearboxTemperature() {
  int16_t raw = 0;
  if (impl_->sdoReadI16(0x20, 0x09, raw) == 0) {
    return static_cast<float>(raw) * 0.1f;  // raw * 0.1 -> ℃
  }
  return 0.0f;
}

float modi_joint_canevo::NrtGetElectricalAngle() {
  uint16_t raw = 0;
  if (impl_->sdoReadU16(0x20, 0x0A, raw) == 0) {
    return static_cast<float>(raw) * 0.1f * kDegToRad;  // raw * 0.1 deg -> rad
  }
  return 0.0f;
}

/* ============================================================
 * 状态查询辅助（非阻塞，内部读取缓存状态字）
 * ============================================================ */

CanEvoServoState modi_joint_canevo::RtGetServoState() {
  if (!impl_->isInitialized()) return CanEvoServoState::kInit;
  uint16_t sw = impl_->GetStatusword();
  return static_cast<CanEvoServoState>(sw & kSwStateMask);
}

CanEvoMode modi_joint_canevo::RtGetCurrentMode() {
  if (!impl_->isInitialized()) return CanEvoMode::kCsp;
  uint16_t sw = impl_->GetStatusword();
  return static_cast<CanEvoMode>((sw & kSwModeMask) >> kSwModeShift);
}

bool modi_joint_canevo::RtIsRunning() {
  if (!impl_->isInitialized()) return false;
  uint16_t sw = impl_->GetStatusword();
  return (sw & kSwStateMask) ==
         static_cast<uint16_t>(CanEvoServoState::kRunning);
}

bool modi_joint_canevo::RtHasWarning() {
  if (!impl_->isInitialized()) return false;
  uint16_t sw = impl_->GetStatusword();
  return (sw & kSwWarnBit) != 0;
}
