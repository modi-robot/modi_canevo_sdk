/**
 * @file 08_csp_trajectory_from_file.cpp
 * @brief CSP轨迹测试 - 从CSV文件读取轨迹并执行
 * 
 * 输出CSV只有一列：时间戳（秒.纳秒格式）
 * 使用POSIX系统调用，不用C++ chrono
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <time.h>
#include <signal.h>
#include <unistd.h>

#include <cmath>
#include <fstream>
#include <iostream>
#include <thread>
#include <vector>
#include <atomic>
#include <sstream>
#include <iomanip>

#include "modi_joint_canevo.h"

constexpr float kDegToRad = static_cast<float>(M_PI) / 180.0f;

// 轨迹点结构
struct TrajectoryPoint {
    float time_s;
    float position_rad;
};

// 全局变量
static modi_bus_canevo* g_bus = nullptr;
static modi_joint_canevo* g_joint = nullptr;
static std::vector<TrajectoryPoint> g_trajectory;
static std::atomic<bool> g_running{true};
static std::vector<struct timespec> g_timestamps;  // 存原始timespec

// 信号处理函数
void SignalHandler(int) {
    g_running = false;
}

// 从CSV文件读取轨迹
bool LoadTrajectory(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return false;
    
    std::string line;
    std::getline(file, line);  // 跳过表头
    
    g_trajectory.clear();
    g_trajectory.reserve(3000);
    
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        
        TrajectoryPoint point;
        
        // 读取时间
        std::getline(ss, token, ',');
        point.time_s = std::stof(token);
        
        // 读取位置（度）并转换为弧度
        std::getline(ss, token, ',');
        point.position_rad = std::stof(token) * kDegToRad;
        
        g_trajectory.push_back(point);
    }
    
    file.close();
    return true;
}

int main() {
    // 1. 锁定内存
    mlockall(MCL_CURRENT | MCL_FUTURE);
    
    // 2. 加载轨迹文件
    if (!LoadTrajectory("trajectory.csv")) {
        return -1;
    }

    // 3. 配置任务参数
    TaskConfig task_config;
    task_config.period_us = 5000;
    task_config.priority = 99;
    task_config.cpu_affinity = 2;

    // 4. 创建对象
    modi_bus_canevo bus;
    modi_joint_canevo joint;

    g_bus = &bus;
    g_joint = &joint;

    signal(SIGINT, SignalHandler);
    signal(SIGTERM, SignalHandler);

    // 5. 打开总线
    if (bus.Open("can0", task_config) != static_cast<int>(CanEvoError::kOk)) {
        return -1;
    }

    bus.SetSdoTimeoutMs(50);
    bus.SetPdoTimeoutMs(50);

    // 6. 初始化关节
    if (joint.NrtInit(bus, 1) != static_cast<int>(CanEvoError::kOk)) {
        bus.Close();
        return -1;
    }

    // 7. 设置机械零点
    joint.NrtDisable();
    while (joint.NrtGetServoState() != CanEvoServoState::kReady) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    
    joint.NrtSetMechZero();
    while (joint.NrtGetMechZeroOk() != 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 8. 使能关节
    if (joint.NrtEnable(CanEvoMode::kCsp) != static_cast<int>(CanEvoError::kOk)) {
        bus.Close();
        return -1;
    }
    while (joint.NrtGetServoState() != CanEvoServoState::kRunning) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // 9. 预分配空间
    g_timestamps.reserve(100000);  // 直接分配10万个空间

    // 10. 启动控制循环
    struct timespec start_time;
    clock_gettime(CLOCK_MONOTONIC, &start_time);
    
    size_t trajectory_index = 0;
    float last_position = g_trajectory[0].position_rad;
    
    auto ret = bus.StartControlLoop([&]() {
        if (!g_running.load(std::memory_order_relaxed)) return;
        
        // 获取当前时间
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        g_timestamps.push_back(now);
        
        // 计算经过时间（秒）
        double elapsed = (now.tv_sec - start_time.tv_sec) + 
                         (now.tv_nsec - start_time.tv_nsec) * 1e-9;
        
        // 查找当前时间对应的轨迹点
        while (trajectory_index < g_trajectory.size() && 
               g_trajectory[trajectory_index].time_s <= elapsed) {
            last_position = g_trajectory[trajectory_index].position_rad;
            trajectory_index++;
        }
        
        // 发送当前位置指令
        joint.RtSetCspTargetPosition(last_position);
        
        // 轨迹完成时自动停止
        if (trajectory_index >= g_trajectory.size()) {
            g_running.store(false, std::memory_order_relaxed);
        }
    });

    if (ret != static_cast<int>(CanEvoError::kOk)) {
        return -1;
    }

    // 11. 等待轨迹完成
    while (g_running.load(std::memory_order_relaxed)) {
        sched_yield();
    } 
    // 13. 失能关节
    joint.NrtDisable();

    // 14. 停止控制循环
    bus.Close();

    // 15. 输出CSV文件（秒.纳秒格式）
    std::ofstream csv_file("timestamps.csv");
    if (csv_file.is_open()) {
        csv_file << "timestamp\n";  // 表头
        csv_file.rdbuf()->pubsetbuf(nullptr, 0);
        for (const auto& ts : g_timestamps) {
            csv_file << ts.tv_sec << "." 
                     << std::setw(9) << std::setfill('0') << ts.tv_nsec << '\n';
        }
        csv_file.close();
    }

    // 16. 清理
    joint.NrtDestroy();
    munlockall();

    return 0;
}
