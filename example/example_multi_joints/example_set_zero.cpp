/**
 * @file example_set_zero.cpp
 * @brief 设置电机零点的交互式示例
 *
 * 功能:
 *   1. 打开 CAN 总线并扫描所有在线关节
 *   2. 实时刷新每个关节的当前 ID 和角度（逐行清写，原地刷新，任何终端都稳）
 *   3. 键盘交互选择当前关节，将其当前位置设为机械零点
 *
 * 按键说明:
 *   + / ↑ / 数字键:  切换选中的关节 ID
 *   - / ↓:           反向切换选中的关节
 *   空格 / Enter:    将当前选中关节的当前位置设为机械零点
 *   s:               保存参数到 Flash
 *   q / ESC:         退出
 *
 * 运行前请先手动配置并拉起 CAN-FD 接口:
 *   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
 *   sudo ip link set can0 up
 */

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "modi_joint_canevo.h"

constexpr float kRadToDeg = 180.0f / 3.14159265358979323846f;

/* ANSI — 只用最基础的, 任何终端都支持 */
namespace ansi {
constexpr const char* kSave    = "\x1b[s";
constexpr const char* kRestore = "\x1b[u";
constexpr const char* kClearLn = "\x1b[2K";   /* 清当前整行 */
constexpr const char* kDownHome= "\x1b[1B\x1b[G"; /* 下移一行 + 回到列 1 */
constexpr const char* kHideCur = "\x1b[?25l";
constexpr const char* kShowCur = "\x1b[?25h";
constexpr const char* kReset   = "\x1b[0m";
constexpr const char* kBold    = "\x1b[1m";
constexpr const char* kDim     = "\x1b[2m";
constexpr const char* kYellow  = "\x1b[33m";
constexpr const char* kCyan    = "\x1b[36m";
constexpr const char* kGray    = "\x1b[90m";
}  // namespace ansi

static std::atomic<bool> g_running{true};
void SignalHandler(int /*signum*/) { g_running.store(false); }

/* ---------- 终端 raw mode ---------- */
class RawTerm {
 public:
  RawTerm() {
    tcgetattr(STDIN_FILENO, &orig_);
    struct termios raw = orig_;
    cfmakeraw(&raw);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags_ | O_NONBLOCK);
  }
  ~RawTerm() {
    fcntl(STDIN_FILENO, F_SETFL, flags_);
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
  }
 private:
  struct termios orig_ {};
  int flags_ = 0;
};

/* ---------- ANSI 宽度感知的对齐 ---------- */
static size_t AnsiPrintLen(const std::string& s) {
  size_t w = 0;
  for (size_t i = 0; i < s.size();) {
    if (s[i] == '\x1b' && i + 1 < s.size() && s[i + 1] == '[') {
      i += 2;
      while (i < s.size() && ((s[i] >= '0' && s[i] <= '9') || s[i] == ';')) ++i;
      if (i < s.size() && s[i] >= 'A' && s[i] <= 'z') ++i;
    } else { ++w; ++i; }
  }
  return w;
}
static std::string PadLeft(const std::string& s, int w) {
  const size_t L = AnsiPrintLen(s);
  return (int)L >= w ? s : std::string(w - L, ' ') + s;
}
static std::string PadRight(const std::string& s, int w) {
  const size_t L = AnsiPrintLen(s);
  return (int)L >= w ? s : s + std::string(w - L, ' ');
}
static std::string Col(const char* color, const std::string& plain) {
  return std::string(color) + plain + ansi::kReset;
}
template <typename T>
static std::string Fmt(T val, int width, int prec) {
  std::ostringstream o;
  o << std::fixed << std::setprecision(prec) << std::setw(width) << val;
  return o.str();
}

/* ---------- 渲染: 逐行清行 + 写 + 下移下一行首列 ---------- */
static void DrawFrame(
    const std::string& can_iface,
    int n, int selected_idx,
    const std::vector<uint8_t>& joint_ids,
    std::vector<modi_joint_canevo>& joints,
    const std::string& status_msg)
{
  /* 回到保存的起始位置 (第一行第一列) */
  std::cout << ansi::kRestore;

  /* 每一行: 清行 → 写内容 → 下移到下一行第一列 */
  auto put_line = [](const std::string& content) {
    std::cout << ansi::kClearLn << content << ansi::kDownHome;
  };

  /* 1. 标题 */
  std::string info_plain =
      "CanEvo 电机零点设置  |  CAN: " + can_iface +
      "  |  关节数: " + std::to_string(n) +
      "  |  选中: " + std::to_string(static_cast<int>(joint_ids[selected_idx]));
  put_line(Col(ansi::kBold, Col(ansi::kCyan, info_plain)));

  /* 2. 表头 */
  put_line("  " +
           PadRight(Col(ansi::kDim, "ID"), 6) +
           PadRight(Col(ansi::kDim, "rad"), 12) +
           Col(ansi::kDim, "deg"));

  /* 3. 分隔线 */
  put_line(Col(ansi::kDim, std::string(50, '-')));

  /* 4. 每关节一行 */
  for (int i = 0; i < n; ++i) {
    const uint8_t id = joint_ids[i];
    const float pos = joints[i].NrtGetActualPosition();
    const float deg = pos * kRadToDeg;
    const bool sel = (i == selected_idx);

    std::string row_plain =
        " " + std::string(sel ? "▶" : " ") + " " +
        PadRight(" " + std::to_string(static_cast<int>(id)) + " ", 6) +
        PadLeft(Fmt(pos, 10, 3), 12) + "  " +
        PadLeft(Fmt(deg, 10, 2), 10);

    put_line(sel
        ? Col(ansi::kBold, Col(ansi::kYellow, row_plain))
        : row_plain);
  }

  /* 5. 分隔线 */
  put_line(Col(ansi::kDim, std::string(50, '-')));

  /* 6. 状态消息 (空消息也打空行, 保证行数一致) */
  put_line(status_msg.empty()
      ? std::string()
      : "  " + Col(ansi::kBold, status_msg));

  /* 7. 空行 */
  put_line(std::string());

  /* 8. 操作提示 */
  put_line("  " + Col(ansi::kDim,
      "[+/↑/数字] 选择   [-/↓] 反向   [空格] 设零点   [s] 保存   [q/ESC] 退出"));

  std::cout << std::flush;
}

/* ---------- 主程序 ---------- */

int main(int argc, char* argv[]) {
  if (argc != 2) {
    std::cerr << "用法: " << argv[0] << " <CAN接口>\n"
              << "示例: " << argv[0] << " can0" << std::endl;
    return -1;
  }
  const std::string can_iface = argv[1];
  std::signal(SIGINT, SignalHandler);
  std::signal(SIGTERM, SignalHandler);

  modi_bus_canevo bus;
  TaskConfig cfg;
  cfg.sync_period_us = 5000;
  cfg.cpu_affinity = 2;
  if (bus.Open(can_iface, cfg) != static_cast<int>(CanEvoError::kOk)) {
    std::cerr << "打开 CAN 总线失败: " << can_iface << std::endl;
    return -1;
  }
  bus.SetSdoTimeoutMs(100);

  auto joint_ids = bus.NrtScanJoints();
  if (joint_ids.empty()) {
    std::cerr << "未扫描到在线关节" << std::endl;
    bus.Close();
    return -1;
  }

  std::vector<modi_joint_canevo> joints(joint_ids.size());
  for (size_t i = 0; i < joint_ids.size(); ++i) {
    if (joints[i].NrtInit(bus, joint_ids[i]) !=
        static_cast<int>(CanEvoError::kOk)) {
      std::cerr << "初始化关节 ID=" << static_cast<int>(joint_ids[i])
                << " 失败" << std::endl;
      for (size_t j = 0; j < i; ++j) joints[j].NrtDestroy();
      bus.Close();
      return -1;
    }
  }

  int selected_idx = 0;
  const int n = static_cast<int>(joint_ids.size());
  std::string status_msg;
  auto status_expire = std::chrono::steady_clock::now();

  /* 上次快照 — 只有数据真变了才重绘 */
  int last_sel = -1;
  std::vector<float> last_positions(n, 0.0f);
  std::string last_status;
  constexpr float kRadDiffThreshold = 0.001f;

  RawTerm raw;

  /* 先打一屏空行占位置, 然后保存光标到第一行 — 后续所有重绘都从这里逐行覆盖 */
  std::cout << std::string(20, '\n');     /* 占 20 行 */
  std::cout << "\x1b[20A";                /* 光标上移 20 行到第一行 */
  std::cout << ansi::kSave << ansi::kHideCur << std::flush;

  bool first_draw = true;
  auto last_refresh = std::chrono::steady_clock::now();

  while (g_running.load()) {
    auto now = std::chrono::steady_clock::now();

    /* --- 键盘 --- */
    pollfd pfd{};
    pfd.fd = STDIN_FILENO;
    pfd.events = POLLIN;
    const int poll_ret = poll(&pfd, 1, 10);
    if (poll_ret > 0 && (pfd.revents & POLLIN)) {
      char buf[16];
      const ssize_t nr = read(STDIN_FILENO, buf, sizeof(buf));
      if (nr == 1) {
        char c = buf[0];
        if (c == 'q' || c == 'Q') {
          g_running.store(false); break;
        } else if (c == 27) {
          char more[2];
          ssize_t n2 = read(STDIN_FILENO, more, 2);
          if (n2 == 2 && more[0] == '[') {
            if (more[1] == 'A' || more[1] == 'C')
              selected_idx = (selected_idx + 1) % n;
            else if (more[1] == 'B' || more[1] == 'D')
              selected_idx = (selected_idx - 1 + n) % n;
          } else { g_running.store(false); break; }
        } else if (c == ' ' || c == '\n' || c == '\r') {
          modi_joint_canevo& j = joints[selected_idx];
          const uint8_t nid = joint_ids[selected_idx];
          const int ret = j.NrtSetMechZero();
          if (ret != static_cast<int>(CanEvoError::kOk)) {
            status_msg = "[ID=" + std::to_string(nid) + "] 设零点失败 ret=" +
                         std::to_string(ret);
          } else {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            const uint16_t ok = j.NrtGetMechZeroOk();
            status_msg = (ok == 1)
                ? "[ID=" + std::to_string(nid) + "] ✓ 机械零点已设置"
                : "[ID=" + std::to_string(nid) + "] 未确认 flag=" +
                  std::to_string(ok);
          }
          status_expire = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(2500);
        } else if (c == 's' || c == 'S') {
          modi_joint_canevo& j = joints[selected_idx];
          const uint8_t nid = joint_ids[selected_idx];
          const int ret = j.NrtSaveParams();
          if (ret != static_cast<int>(CanEvoError::kOk)) {
            status_msg = "[ID=" + std::to_string(nid) + "] 保存失败 ret=" +
                         std::to_string(ret);
          } else {
            bool saved = false;
            for (int k = 0; k < 50; ++k) {
              std::this_thread::sleep_for(std::chrono::milliseconds(100));
              if (j.NrtGetSaveParamsOk() == 1) { saved = true; break; }
            }
            status_msg = saved
                ? "[ID=" + std::to_string(nid) + "] ✓ 参数已保存到 Flash"
                : "[ID=" + std::to_string(nid) + "] 保存超时";
          }
          status_expire = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(2500);
        } else if (c == '+' || c == '=') {
          selected_idx = (selected_idx + 1) % n;
        } else if (c == '-' || c == '_') {
          selected_idx = (selected_idx - 1 + n) % n;
        } else if (c >= '1' && c <= '9') {
          int dig = c - '1';
          if (dig < n) selected_idx = dig;
        }
      }
    }

    /* --- 每 200ms 检查, 只在数据变化时重绘 --- */
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_refresh).count() >= 200) {
      last_refresh = now;
      if (!status_msg.empty() && now >= status_expire) status_msg.clear();

      bool dirty = first_draw;
      if (!dirty && selected_idx != last_sel) dirty = true;
      if (!dirty && status_msg != last_status) dirty = true;
      if (!dirty) {
        for (int i = 0; i < n; ++i) {
          const float p = joints[i].NrtGetActualPosition();
          if (std::fabs(p - last_positions[i]) > kRadDiffThreshold) {
            dirty = true; break;
          }
        }
      }

      if (dirty) {
        last_sel = selected_idx;
        last_status = status_msg;
        for (int i = 0; i < n; ++i)
          last_positions[i] = joints[i].NrtGetActualPosition();
        first_draw = false;
        DrawFrame(can_iface, n, selected_idx, joint_ids, joints, status_msg);
      }
    }
  }

  /* 退出: 回到保存位置, 逐行清掉占用的 20 行, 再打印收尾 */
  std::cout << ansi::kRestore;
  for (int i = 0; i < 20; ++i)
    std::cout << ansi::kClearLn << ansi::kDownHome;
  std::cout << ansi::kRestore << ansi::kShowCur << ansi::kReset;
  std::cout << "正在清理资源 ... " << std::flush;
  for (auto& j : joints) j.NrtDestroy();
  bus.Close();
  std::cout << "完成\n";
  return 0;
}
