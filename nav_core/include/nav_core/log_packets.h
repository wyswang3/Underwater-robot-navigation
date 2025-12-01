#pragma once

/**
 * @file   log_packets.h
 * @brief  导航二进制日志包结构定义（IMU / DVL / ESKF）
 *
 * 说明：
 *   - 这些结构体用于：
 *       1) nav_daemon.cpp 中写二进制日志（imu.bin / dvl.bin / eskf.bin）
 *       2) 离线解析工具（例如 dump_imu_bin / 你现在的解析程序）读取日志
 *   - 必须保持 pack(1) 对齐，以保证跨平台、跨编译器二进制布局一致；
 *   - 一旦正式使用，请避免随意改动字段顺序或类型，
 *     如需扩展可通过末尾 reserved[]、版本号等方式。
 */

#include <cstdint>

#pragma pack(push, 1)

/// IMU 日志包（与 nav_daemon 中写入逻辑保持严格一致）
struct ImuLogPacket {
    int64_t mono_ns;          ///< 单调时钟时间戳（ns）
    int64_t est_ns;           ///< 估计时间基（与其他传感器对齐）

    float   lin_acc[3];       ///< 线加速度（m/s^2）
    float   ang_vel[3];       ///< 角速度（rad/s）
    float   euler[3];         ///< 欧拉角（rad）：(roll, pitch, yaw)

    float   temperature;      ///< IMU 温度
    uint8_t valid;            ///< 标志：1=有效，0=无效
    uint8_t reserved[3];      ///< 对齐保留
};

/// DVL 日志包（简化版：速度 + 质量标志）
struct DvlLogPacket {
    int64_t mono_ns;          ///< 单调时钟时间戳（ns）
    int64_t est_ns;           ///< 估计时间基（与 IMU 等对齐）

    float   vel[3];           ///< 速度（m/s），坐标系由 DvlDriver 定义
    int32_t valid;            ///< 1=有效，0=无效
    int32_t quality;          ///< DVL 质量指标（设备定义）
};

/// ESKF 导航状态日志包（位置/速度/姿态/偏置）
struct EskfLogPacket {
    int64_t mono_ns;          ///< 单调时钟时间戳（ns）
    int64_t est_ns;           ///< 估计时间基

    float   pos[3];           ///< 位置（m）
    float   vel[3];           ///< 速度（m/s）
    float   euler[3];         ///< 姿态（rad）

    float   bias_accel[3];    ///< 加速度偏置估计
    float   bias_gyro[3];     ///< 陀螺偏置估计

    uint8_t valid;            ///< 状态是否有效
    uint8_t status;           ///< 状态码（bitmask，用于诊断）
    uint8_t reserved[2];      ///< 对齐保留
};

#pragma pack(pop)
