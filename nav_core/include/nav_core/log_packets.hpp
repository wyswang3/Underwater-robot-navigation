#pragma once

#include <cstdint>
#include <array>

#include <nav_core/types.hpp>

namespace nav_core {

/**
 * @brief IMU 日志数据包（旧版完整字段）
 *
 * 该结构与 nav_daemon.cpp 中的使用完全对应。
 * NOTE: 字段顺序与大小保持二进制稳定（不轻易调整）。
 */
struct ImuLogPacket
{
    MonoTimeNs mono_ns;            // 单调时钟时间（来自驱动）
    SysTimeNs  est_ns;             // 估计时间戳（驱动补偿）

    std::array<float,3> lin_acc;   // 线加速度（m/s^2）
    std::array<float,3> ang_vel;   // 角速度（rad/s）
    std::array<float,3> euler;     // 欧拉角 (roll pitch yaw，rad)

    float temperature;             // 温度（摄氏度）
    std::uint8_t valid;            // 有效标志：1=有效，0=无效

    std::uint8_t reserved1{0};
    std::uint16_t reserved2{0};
};

/**
 * @brief DVL 日志数据包（旧版结构）
 */
struct DvlLogPacket
{
    MonoTimeNs mono_ns;   // 单调时间
    SysTimeNs  est_ns;    // 延迟补偿后的时间

    float vel[3];          // ENU 或 NED 速度（m/s）
    float depth;           // 深度（m）
    float e;               // 东向位移（可选）
    float n;               // 北向位移
    float u;               // 上向位移
    int   quality;         // 数据质量（协议帧中的 A/V 或具体等级）

    std::uint8_t valid;    // 1=有效，0=无效

    std::uint8_t reserved1{0};
    std::uint16_t reserved2{0};
};

/**
 * @brief ESKF 日志包（旧版字段）
 */
struct EskfLogPacket
{
    MonoTimeNs mono_ns;    // 写日志时间（导航内部时间）
    SysTimeNs  est_ns;     // 状态估计时间（IMU+外部融合基准）

    float pos[3];          // 位置（m）
    float vel[3];          // 速度（m/s）
    float euler[3];        // 姿态（rad）
    float ang_vel[3];      // 角速度（rad/s）

    float bias_accel[3];   // 加速度偏置估计
    float bias_gyro[3];    // 陀螺仪偏置估计

    std::uint8_t valid;    // 状态是否有效
    std::uint8_t status;   // bitmask 状态
    std::uint16_t reserved{0};
};

} // namespace nav_core
