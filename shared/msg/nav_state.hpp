#pragma once
#ifndef UWSYS_SHARED_MSG_NAV_STATE_HPP
#define UWSYS_SHARED_MSG_NAV_STATE_HPP

/**
 * @file   nav_state.hpp
 * @brief  导航状态在「导航进程 ↔ 控制进程」之间的共享消息定义（跨项目通用）
 *
 * 设计目标：
 *   - 仅依赖标准头文件，可在任意 C++17 项目中直接 include；
 *   - POD / 平凡布局，便于通过共享内存 / ZeroMQ / UDP 直接按字节发送；
 *   - 字段语义清晰，既能满足 PID，也能满足后续 MPC / 轨迹控制需求；
 *   - 导航项目（Underwater-robot-navigation）和控制项目
 *     （OrangePi_STM32_for_ROV）统一使用此定义，避免结构漂移。
 *
 * 时间基准约定：
 *   - t_ns 使用「单调时钟」的纳秒计数（std::chrono::steady_clock）；
 *   - 不要求与系统墙钟对齐，只要求在一台机子上单调递增；
 *   - 控制程序收到后，用 (now_ns() - t_ns) 可估计观测延迟。
 */

#include <cstdint>

namespace shared::msg {

/**
 * @brief 导航整体健康状况（粗粒度）
 */
enum class NavHealth : std::uint8_t {
    UNINITIALIZED = 0,   ///< 尚未完成初始化 / 尚无有效状态
    OK            = 1,   ///< 工作正常（IMU + DVL + 深度等基本可用）
    DEGRADED      = 2,   ///< 降级模式（例如 DVL 丢失，仅惯导）
    INVALID       = 3    ///< 状态不可用，控制侧建议进入 failsafe
};

/**
 * @brief 传感器 / 估计器状态的 bitmask（细粒度）
 *
 * 可按需组合：
 *   - IMU / DVL / 深度 / USBL 单独是否可用；
 *   - ESKF 当前是否正常收敛等。
 */
enum NavStatusFlags : std::uint16_t {
    NAV_FLAG_NONE        = 0,

    NAV_FLAG_IMU_OK      = 1u << 0,   ///< IMU 数据正常
    NAV_FLAG_DVL_OK      = 1u << 1,   ///< DVL 速度有效
    NAV_FLAG_DEPTH_OK    = 1u << 2,   ///< 深度传感器正常
    NAV_FLAG_USBL_OK     = 1u << 3,   ///< USBL / 外部定位正常

    NAV_FLAG_ESKF_OK     = 1u << 4,   ///< ESKF / 状态估计算法正常
    NAV_FLAG_ALIGN_DONE  = 1u << 5,   ///< 完成初始对准 / 零偏估计

    NAV_FLAG_RESERVED6   = 1u << 6,
    NAV_FLAG_RESERVED7   = 1u << 7,
    NAV_FLAG_RESERVED8   = 1u << 8,
    NAV_FLAG_RESERVED9   = 1u << 9,
    NAV_FLAG_RESERVED10  = 1u << 10,
    NAV_FLAG_RESERVED11  = 1u << 11,
    NAV_FLAG_RESERVED12  = 1u << 12,
    NAV_FLAG_RESERVED13  = 1u << 13,
    NAV_FLAG_RESERVED14  = 1u << 14,
    NAV_FLAG_RESERVED15  = 1u << 15
};

/**
 * @brief 导航状态主结构（跨进程共享的“语言”）
 *
 * 坐标系约定（建议，具体可在文档中明确）：
 *   - pos: NED 或 ENU 局部坐标系（由导航侧统一定义）
 *   - vel: 同 pos 坐标系的线速度
 *   - rpy: 机体系相对于导航坐标系的欧拉角 [rad]，顺序 (roll, pitch, yaw)
 *
 * 高频动力学量：
 *   - omega_b: 机体系角速度 [rad/s]，对应 IMU / ESKF 输出的 ω_body
 *   - acc_b:   机体系线加速度 [m/s^2]，重力是否剔除在导航文档中说明
 *
 * 深度：
 *   - depth: 一般为“水面向下为正”的深度，单位 [m]；
 *   - 如果暂无有效深度，可填 NaN 或约定值，并在 status_flags 中取消 DEPTH_OK。
 */
struct NavState
{
    std::uint64_t t_ns;      ///< 单调时间戳 (ns)

    // ---- 1. 导航层基础输出 (NED/ENU) ----
    double pos[3];           ///< 位置: [x, y, z]
    double vel[3];           ///< 速度: [vx, vy, vz]
    double rpy[3];           ///< 姿态: roll, pitch, yaw (rad)

    double depth;            ///< 深度 [m]，下为正

    // ---- 2. 高频动力学量（body 坐标系）----
    double omega_b[3];       ///< 机体系角速度 [wx, wy, wz], rad/s
    double acc_b[3];         ///< 机体系线加速度 [ax, ay, az], m/s^2

    // ---- 3. 状态标志 ----
    NavHealth     health;        ///< 整体健康状态（粗粒度）
    std::uint8_t  reserved;      ///< 对齐 / 未来扩展（例如导航模式编号）
    std::uint16_t status_flags;  ///< bitmask（NavStatusFlags 的组合）
};

// 对齐检查：保证结构至少是 4 字节对齐，便于跨语言/跨进程传输
static_assert(sizeof(NavState) % 4 == 0,
              "NavState size should be 4-byte aligned for efficient transport");

} // namespace shared::msg

#endif // UWSYS_SHARED_MSG_NAV_STATE_HPP
