// nav_core/include/nav_core/io/log_packets.hpp
//
// @file  log_packets.hpp
// @brief 在线导航 / 采集进程写入二进制日志文件使用的 POD 报文定义。
//
// 设计定位：
//   - 提供 IMU / DVL / ESKF 等模块写盘用的“日志数据包”结构；
//   - 所有结构均为 POD、平凡布局，便于直接 fwrite / read；
//   - 字段顺序与类型已经在旧版代码中使用，**禁止随意调整**；
//   - 后续若需要新增字段，应：
//       * 只能在结构末尾追加；
//       * 同时 bump 版本号或使用新的 *_V2 结构，避免混用。
//
// 典型使用位置：
//   - nav_daemon.cpp 中的二进制日志写入；
//   - 离线 Python / C++ 解析工具按相同结构读取。
//   - 在线导航 / estimator 内部推荐使用 ImuSample / DvlSample，
//     日志只在“边缘”模块做一次结构转换即可。
// 

#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include <nav_core/core/types.hpp>  // MonoTimeNs / SysTimeNs

namespace nav_core::io {

/// @brief IMU 日志数据包（旧版完整字段，V1 格式）
///
/// 语义：
///   - mono_ns / est_ns：
///       * mono_ns：写入这一帧时的单调时钟时间（驱动时间基）；
///       * est_ns ：估计的 UNIX 时间（可选，用于对时与离线分析）；
///   - lin_acc/ang_vel/euler：
///       * 单位与 ImuFrame 保持一致：
///           lin_acc: m/s^2
///           ang_vel: rad/s
///           euler  : rad （roll, pitch, yaw）
///   - temperature：IMU 报告的温度（℃）；
///   - valid：1=有效，0=无效（过滤层已丢弃的帧可以写 0，仅做时间戳参考）。
///
/// 警告：
///   - 字段顺序与大小已经被旧版日志解析脚本依赖，不可改动；
///   - 若需新增字段，请在末尾追加，并采用新的日志标签或版本。
struct ImuLogPacket
{
    MonoTimeNs mono_ns;                 ///< 单调时钟时间（来自驱动，ns）
    SysTimeNs  est_ns;                  ///< 估计 UNIX 时间（ns）

    std::array<float, 3> lin_acc;       ///< 线加速度 [ax, ay, az] (m/s^2)
    std::array<float, 3> ang_vel;       ///< 角速度 [wx, wy, wz] (rad/s)
    std::array<float, 3> euler;         ///< 欧拉角 [roll, pitch, yaw] (rad)

    float       temperature;            ///< 温度 (℃)
    std::uint8_t valid;                 ///< 有效标志：1=有效，0=无效

    std::uint8_t  reserved1{0};
    std::uint16_t reserved2{0};
};

static_assert(std::is_trivially_copyable_v<ImuLogPacket>,
              "ImuLogPacket must be trivially copyable for binary logging");


/// @brief DVL 日志数据包（旧版 V1 结构）
///
/// 语义与注意事项：
///   - mono_ns / est_ns：与 ImuLogPacket 含义一致；
///   - vel[3]：
///       * 约定为“导航坐标系速度”，通常为 ENU [vE, vN, vU] (m/s)；
///       * 早期日志可能存在 NED，离线解析时需结合实验日期确认；
///   - depth：深度 [m]，下为正；
///   - e/n/u：累计位移 [E, N, U] (m)（若 DVL 帧不含则为 0）；
///   - quality：
///       * 可直接保存协议中的质量指标（如 FOM 或 A/V 映射的等级）；
///   - valid：1=有效，0=无效（例如锁底失败 / 水团帧被过滤掉时可置 0）。
///
/// 同样，字段顺序 / 类型不可变。
struct DvlLogPacket
{
    MonoTimeNs mono_ns;    ///< 单调时间（ns）
    SysTimeNs  est_ns;     ///< 延迟补偿后的“估计时间”（ns）

    float vel[3];          ///< 导航坐标系速度 (m/s)，典型为 ENU [vE, vN, vU]
    float depth;           ///< 深度 [m]（向下为正）
    float e;               ///< 东向位移 [m]
    float n;               ///< 北向位移 [m]
    float u;               ///< 上向位移 [m]

    int   quality;         ///< 数据质量指标（协议字段映射）

    std::uint8_t  valid;   ///< 1=有效，0=无效
    std::uint8_t  reserved1{0};
    std::uint16_t reserved2{0};
};

static_assert(std::is_trivially_copyable_v<DvlLogPacket>,
              "DvlLogPacket must be trivially copyable for binary logging");


/// @brief ESKF 日志包（旧版 V1 字段）
///
/// 语义：
///   - mono_ns / est_ns：
///       * mono_ns：写入该状态时的内部时间基；
///       * est_ns ：状态对应的物理时间（由滤波器内部确定）；
///   - pos/vel/euler/ang_vel：全局位置 / 速度 / 姿态 / 机体系角速度；
///   - bias_accel/bias_gyro：滤波器估计的零偏；
///   - valid：滤波状态是否可用；
///   - status：bitmask 状态（具体位含义由 ESKF 模块约定）。
///
/// 目前 ESKF 在工程上可能暂未启用，但为兼容历史日志格式保留。
struct EskfLogPacket
{
    MonoTimeNs mono_ns;      ///< 写日志时间（滤波内部时间基，ns）
    SysTimeNs  est_ns;       ///< 状态估计对应时间（ns）

    float pos[3];            ///< 位置 [x, y, z] (m)
    float vel[3];            ///< 速度 [vx, vy, vz] (m/s)
    float euler[3];          ///< 姿态 [roll, pitch, yaw] (rad)
    float ang_vel[3];        ///< 机体系角速度 [wx, wy, wz] (rad/s)

    float bias_accel[3];     ///< 加速度偏置估计
    float bias_gyro[3];      ///< 陀螺仪偏置估计

    std::uint8_t  valid;     ///< 状态是否有效
    std::uint8_t  status;    ///< bitmask 状态（由 ESKF 模块定义）
    std::uint16_t reserved{0};
};

static_assert(std::is_trivially_copyable_v<EskfLogPacket>,
              "EskfLogPacket must be trivially copyable for binary logging");

} // namespace nav_core::io
