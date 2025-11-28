// Underwater-robot-navigation/nav_core/include/nav_core/imu_types.h
#pragma once

#include <cstdint>

namespace nav_core {

/**
 * @brief 原始寄存器更新信息（来自 Wit SDK 的回调）
 *
 * 注意：这里并没有拷贝寄存器数组本身（sReg[] 仍由 wit_c_sdk 管理），
 * 只携带这次更新的起始寄存器与数量，便于上层做调试记录或统计。
 */
struct ImuRawRegs {
    double   timestamp_s = 0.0;  ///< 回调触发时的粗略系统时间（秒）
    uint32_t start_reg   = 0;    ///< 本次更新的起始寄存器地址
    uint32_t count       = 0;    ///< 连续寄存器数量
};

/**
 * @brief IMU 标准化数据帧（供导航/滤波/控制使用）
 *
 * 单位约定：
 *  - ang_vel: rad/s
 *  - lin_acc: m/s^2
 *  - euler:   rad
 *  - temperature: °C
 *
 * 时间戳：
 *  - mono_ns: 单调时钟（纳秒），用于系统内部时间对齐
 *  - est_ns:  估计 UNIX 时间（纳秒），用于记录/日志
 */
struct ImuFrame {
    int64_t mono_ns = 0;       ///< 单调时钟时间戳（纳秒）
    int64_t est_ns  = 0;       ///< 估计 UNIX 时间戳（纳秒）

    float ang_vel[3]  = {0.f, 0.f, 0.f}; ///< 角速度 [wx, wy, wz] (rad/s)
    float lin_acc[3]  = {0.f, 0.f, 0.f}; ///< 线加速度 [ax, ay, az] (m/s^2)
    float euler[3]    = {0.f, 0.f, 0.f}; ///< 欧拉角 [roll, pitch, yaw] (rad)
    float temperature = 0.f;             ///< 温度 (°C)

    bool  valid  = false;                ///< 该帧是否通过过滤
    int   status = 0;                    ///< 预留状态位（自检、错误码等）
};

/**
 * @brief IMU 过滤配置
 *
 * 用于简单去除明显异常数据（尖刺、NaN 等），避免污染后端滤波器。
 */
struct ImuFilterConfig {
    bool enable_accel = true;   ///< 是否启用加速度的有效性检查
    bool enable_gyro  = true;   ///< 是否启用角速度的有效性检查
    bool enable_euler = false;  ///< 是否启用欧拉角的有效性检查

    double max_abs_accel = 50.0; ///< 单轴最大 |a| (m/s^2)，约 5g
    double max_abs_gyro  = 10.0; ///< 单轴最大 |ω| (rad/s)
    double max_euler_abs = 3.5;  ///< 单轴最大 |姿态| (rad)，约 200°
};

} // namespace nav_core
