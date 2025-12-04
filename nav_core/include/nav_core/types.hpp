// nav_core/include/nav_core/types.hpp
#pragma once

#include <cstdint>
#include <string>
#include <array>

namespace nav_core {

/// 单调时钟时间戳（纳秒），用于内部对齐 / ESKF
using MonoTimeNs = std::int64_t;

/// 估计的 UNIX 时间（纳秒），用于日志 / 对外时间轴
using SysTimeNs  = std::int64_t;

// ===================== 1. 通用向量类型 =====================

struct Vec3f {
    float x{0.f};
    float y{0.f};
    float z{0.f};
};

struct Vec3d {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

// ===================== 2. IMU 相关类型 =====================

/**
 * @brief 原始寄存器更新信息（来自 Wit SDK 的回调）
 *
 * 注意：
 *  - 这里并没有拷贝寄存器数组本身（sReg[] 仍由 wit_c_sdk 管理），
 *  - 只携带这次更新的起始寄存器与数量，便于上层做调试记录或统计。
 *
 * 时间：
 *  - 为兼容老代码，保留 timestamp_s（double 秒）；
 *  - 推荐新代码优先使用 host_mono_ns。
 */
struct ImuRawRegs {
    // 旧接口：粗略系统时间（秒），来自回调触发时的 system_clock
    double     timestamp_s = 0.0;

    // 新接口：更加精确的主机单调时间（纳秒）
    MonoTimeNs host_mono_ns{0};

    std::uint32_t start_reg = 0;  ///< 本次更新的起始寄存器地址
    std::uint32_t count     = 0;  ///< 连续寄存器数量
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
 *  - mono_ns: 单调时钟（纳秒），用于系统内部时间对齐（ESKF 主用）
 *  - est_ns:  估计 UNIX 时间（纳秒），用于记录/日志（可选）
 */
struct ImuFrame {
    MonoTimeNs mono_ns{0};  ///< 单调时钟时间戳（纳秒）
    SysTimeNs  est_ns{0};   ///< 估计 UNIX 时间戳（纳秒）

    float ang_vel[3]  = {0.f, 0.f, 0.f}; ///< 角速度 [wx, wy, wz] (rad/s)
    float lin_acc[3]  = {0.f, 0.f, 0.f}; ///< 线加速度 [ax, ay, az] (m/s^2)
    float euler[3]    = {0.f, 0.f, 0.f}; ///< 欧拉角 [roll, pitch, yaw] (rad)
    float temperature = 0.f;             ///< 温度 (°C)

    bool  valid  = false;                ///< 该帧是否通过 IMU 层过滤
    int   status = 0;                    ///< 预留状态位（自检、错误码等）
};

/**
 * @brief IMU 过滤配置
 *
 * 用于简单去除明显异常数据（尖刺、NaN 等），避免污染后端滤波器。
 */
struct ImuFilterConfig {
    bool   enable_accel = true;   ///< 是否启用加速度的有效性检查
    bool   enable_gyro  = true;   ///< 是否启用角速度的有效性检查
    bool   enable_euler = false;  ///< 是否启用欧拉角的有效性检查

    double max_abs_accel = 50.0;  ///< 单轴最大 |a| (m/s^2)，约 5g
    double max_abs_gyro  = 10.0;  ///< 单轴最大 |ω| (rad/s)
    double max_euler_abs = 3.5;   ///< 单轴最大 |姿态| (rad)，约 200°
};

// 为后端算法统一命名，可以直接把 ImuFrame 当作“采样”
using ImuSample = ImuFrame;

// ===================== 3. DVL 相关类型 =====================

/**
 * @brief DVL 标准化速度帧
 *
 * 单位约定：
 *  - vel_mps: m/s, 导航坐标系 ENU 速度 [vE, vN, vU]
 *  - altitude: m，声束测得的高度（离底距离）
 *
 * 时间戳：
 *  - mono_ns: 单调时间戳（建议来自 timebase::stamp().corrected_time_ns）
 *  - est_ns:  日志用 UNIX 时间戳
 */
struct DvlFrame {
    MonoTimeNs mono_ns{0};  ///< 统一后的时间戳（ns）
    SysTimeNs  est_ns{0};   ///< 估计 UNIX 时间戳（ns）

    float vel_mps[3] = {0.f, 0.f, 0.f}; ///< [vE, vN, vU] ENU 速度 (m/s)

    float altitude_m{0.f};   ///< 高度（离底距离）(m)
    float fom{0.f};          ///< Figure of Merit / 质量指标，0~1 或厂商定义
    // 新增：统一的 ENU 速度数组（m/s）
    float vel[3]{0.f, 0.f, 0.f};


    bool  bottom_lock{false};///< 是否锁底成功
    
    // 新增：有效标志与质量（保留原有 valid 的话，两者都保留也没问题）
    bool  valid{false};      ///< 该帧是否有效（锁底/水体模式等综合判断）
    

    int  quality{0};         ///< 预留状态位（自检、错误码等）
};

/**
 * @brief DVL 过滤配置
 *
 * 用于滤除明显异常的速度 / 高度数据，并做质量门控。
 *
 * 典型策略：
 *  - enable_velocity = true 时，对速度范围做检查；
 *  - enable_altitude = true 时，对高度范围做检查；
 *  - only_valid_flag = true 时，仅接受 valid == A 的帧；
 *  - require_all_axes = true 时，三轴速度都必须是有限数；
 *  - 若 quality < min_quality，则丢弃。
 */
struct DvlFilterConfig {
    bool  enable_velocity = true;   ///< 是否检查速度幅值
    bool  enable_altitude = true;   ///< 是否检查高度范围

    bool  only_valid_flag  = true;  ///< 仅接受 valid==A 的帧（依赖 raw->valid 语义）
    bool  require_all_axes = true;  ///< 要求三轴速度均为有限数

    float max_abs_vel_mps = 5.0f;   ///< 单轴最大 |v|，根据量程设置；<=0 表示不检查
    float max_altitude_m  = 100.0f; ///< 最大有效高度；<=0 表示不检查
    float min_altitude_m  = 0.1f;   ///< 最小有效高度（太近可能不准）

    int   min_quality     = 0;      ///< 质量门限（0 = 不启用）
};

// 同样给一个统一别名
using DvlSample = DvlFrame;

} // namespace nav_core
