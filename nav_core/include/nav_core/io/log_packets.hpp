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
//   - 离线 Python / C++ 解析工具按相同结构读取；
//   - 在线导航 / estimator 内部推荐使用 ImuSample / DvlSample，
//     日志只在“边缘”模块做一次结构转换即可。
// 

#pragma once

#include <array>
#include <cstdint>
#include <type_traits>

#include "nav_core/core/types.hpp"  // MonoTimeNs / SysTimeNs

namespace nav_core::io {

// ============================================================================
// 1. 旧版日志包（V1）——保持兼容，不得更改字段顺序/类型
// ============================================================================

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


// ============================================================================
// 1.x DVL 日志包
// ============================================================================

/// @brief DVL 日志数据包（旧版 V1 结构，Legacy）
///
/// 说明：
///   - 这是历史代码使用的混合结构（vel + depth + e/n/u）；
///   - 新导航系统已经引入 DvlFrame（区分 BI / BE / BD），
///     因此**建议新代码不再写入本结构，只保留兼容旧日志解析**。
///
/// 语义与注意事项（Legacy）：
///   - mono_ns / est_ns：与 ImuLogPacket 含义一致；
///   - vel[3]：
///       * 早期约定为“导航坐标系速度”，可能是 ENU 或 NED，
///         具体需结合实验日期 / 代码版本判断；
///   - depth：深度 [m]，下为正；
///   - e/n/u：累计位移 [E, N, U] (m)；
///   - quality/valid：质量指标 & 有效标志。
struct DvlLogPacket
{
    MonoTimeNs mono_ns;    ///< 单调时间（ns）
    SysTimeNs  est_ns;     ///< 延迟补偿后的“估计时间”（ns）

    float vel[3];          ///< Legacy: 导航速度 (可能为 ENU/NED，视旧版实现而定)
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


// ==================== 新版 V2：按 BI / BE 拆分 ====================

/// @brief DVL 体坐标速度日志包（V2）：BI → body=FRD
///
/// 坐标系与单位：
///   - 速度：body=FRD（X 前、Y 右、Z 下），单位 m/s；
///   - 高度：altitude_m，通常为“探头到底的距离”，Down 为正（与 DVL 协议一致）；
///
/// 用途：
///   - 记录“原始体速度 + 锁底 / 跟踪模式 / FOM / 质量”等底层信息；
///   - 上层 ESKF 可以重放 BI + 当时 IMU 姿态，重新做 body→ENU 旋转。
struct DvlBodyLogPacketV2
{
    MonoTimeNs mono_ns{0};   ///< 单调时间（ns），来自 DvlFrame::mono_ns
    SysTimeNs  est_ns{0};    ///< 估计 UNIX 时间（ns），来自 DvlFrame::est_ns

    // ---- 体速度 B=FRD ----
    float vx_b_mps{0.f};     ///< 前向速度 (m/s)
    float vy_b_mps{0.f};     ///< 右向速度 (m/s)
    float vz_b_mps{0.f};     ///< 下向速度 (m/s)
    std::uint8_t has_body_vel{0}; ///< 1=当前帧包含有效 BI/体速度，对应 DvlFrame::has_body_vel

    // ---- 锁底 & 跟踪模式 ----
    std::uint8_t bottom_lock{0};  ///< 1=锁底成功，对应 DvlFrame::bottom_lock
    std::uint8_t track_type{0};   ///< DvlTrackType 枚举值（bottom/water/unknown）
    std::uint8_t reserved0{0};

    // ---- 高度与质量 ----
    float altitude_m{0.f};        ///< 离底高度（Down 为正），对应 DvlFrame::altitude_m
    std::uint8_t has_altitude{0}; ///< 1=有效高度
    std::uint8_t valid{0};        ///< 1=该帧通过 DvlFilterConfig，被认为可用于导航
    std::uint16_t quality{0};     ///< 质量等级（从 DvlFrame::quality 映射）

    float fom{0.f};               ///< Figure of Merit / 原始 FOM 指标

    std::uint32_t reserved1{0};
};

static_assert(std::is_trivially_copyable_v<DvlBodyLogPacketV2>,
              "DvlBodyLogPacketV2 must be trivially copyable");


/// @brief DVL ENU 速度 / 位移日志包（V2）：BE / BD → ENU
///
/// 坐标系：
///   - 全局：ENU（East, North, Up）；
///   - vel_*：典型来源为 BE 帧（Earth frame），或 BI+IMU 拼出来的 ENU；
///   - d*：典型来源为 BD 帧（Earth frame 下累计 ENU 位移）。
///
/// 用途：
///   - 记录“已经在 DVL 层/预处理层得到的 ENU 速度 / 累计位移”；
///   - 便于离线直接做 ENU 轨迹重构和对比，不必重走 body→ENU。
struct DvlEnuLogPacketV2
{
    MonoTimeNs mono_ns{0};
    SysTimeNs  est_ns{0};

    // ---- ENU 速度 ----
    float vE_mps{0.f};
    float vN_mps{0.f};
    float vU_mps{0.f};
    std::uint8_t has_enu_vel{0};   ///< 1=当前帧包含有效 ENU 速度，对应 DvlFrame::has_enu_vel
    std::uint8_t reserved0{0};
    std::uint16_t reserved1{0};

    // ---- ENU 累计位移 ----
    float dE_m{0.f};
    float dN_m{0.f};
    float dU_m{0.f};
    std::uint8_t has_dist_enu{0};  ///< 1=当前帧包含有效 BD 累计位移，对应 DvlFrame::has_dist_enu
    std::uint8_t bottom_lock{0};   ///< 再记录一次锁底状态，方便单独分析 ENU 轨迹
    std::uint8_t track_type{0};    ///< 与 Body 包体一致
    std::uint8_t reserved2{0};

    // ---- 高度 / 质量 / FOM ----
    float altitude_m{0.f};
    std::uint8_t has_altitude{0};
    std::uint8_t valid{0};
    std::uint16_t quality{0};

    float fom{0.f};
};

static_assert(std::is_trivially_copyable_v<DvlEnuLogPacketV2>,
              "DvlEnuLogPacketV2 must be trivially copyable");


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
/// 目前旧版 ESKF 在工程上可能暂未启用，但为兼容历史日志格式保留。
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


// ============================================================================
// 1.y 时间语义追踪日志包（P0 baseline）
// ============================================================================

enum class TimingTraceKind : std::uint16_t {
    kImuConsumed = 1,
    kDvlConsumed = 2,
    kNavPublished = 3,
};

enum TimingTraceFlags : std::uint16_t {
    kTimingTraceNone      = 0,
    kTimingTraceFresh     = 1u << 0,
    kTimingTraceAccepted  = 1u << 1,
    kTimingTraceValid     = 1u << 2,
    kTimingTraceStale     = 1u << 3,
    kTimingTraceDegraded  = 1u << 4,
};

/**
 * @brief 统一记录“采样时刻 / 接收时刻 / 消费时刻 / 发布时间”的时间追踪包。
 *
 * 用途：
 *   - P0 阶段先把时间语义写实，后续 P1 回放工具可直接基于该文件检查
 *     stale 传播、乱序样本、主线程消费年龄与发布年龄是否一致。
 *   - 该结构只记录时间与最小语义，不承载完整传感器数值。
 */
struct TimingTracePacketV1
{
    std::uint32_t version{1};
    std::uint16_t kind{0};
    std::uint16_t flags{kTimingTraceNone};

    MonoTimeNs sensor_time_ns{0};
    MonoTimeNs recv_mono_ns{0};
    MonoTimeNs consume_mono_ns{0};
    MonoTimeNs publish_mono_ns{0};

    std::uint32_t age_ms{0xFFFFFFFFu};
    std::uint32_t fault_code{0};
};

static_assert(std::is_trivially_copyable_v<TimingTracePacketV1>,
              "TimingTracePacketV1 must be trivially copyable");


// ============================================================================
// 2. 新版 ESKF 监控日志包 —— 健康监控 / 调参专用
//    不影响旧 V1 解析；新代码优先使用这些结构。
// ============================================================================

/// @brief ESKF 状态快照日志包（V2）
///
/// 坐标与单位：
///   - 位置/速度：ENU [E, N, U], [vE, vN, vU] (m / m/s)；
///   - 姿态：nav←body 的 RPY (rad)；
///   - 零偏：body=FRD 下的 accel/gyro bias；
///   - 协方差：从 15x15 P 中抽取若干关键对角线（工程调参够用）。
struct EskfStatePacketV2
{
    MonoTimeNs mono_ns{0};
    SysTimeNs  est_ns{0};

    // 位置 ENU
    float E_m{0.f};
    float N_m{0.f};
    float U_m{0.f};

    // 速度 ENU
    float vE_mps{0.f};
    float vN_mps{0.f};
    float vU_mps{0.f};

    // 姿态（nav←body）
    float roll_rad{0.f};
    float pitch_rad{0.f};
    float yaw_rad{0.f};

    // 零偏（body）
    float ba_x_mps2{0.f};
    float ba_y_mps2{0.f};
    float ba_z_mps2{0.f};

    float bg_x_rad_s{0.f};
    float bg_y_rad_s{0.f};
    float bg_z_rad_s{0.f};

    // 关键协方差对角线（按 ESKF 误差状态索引抽取）
    float P_posE{0.f};
    float P_posN{0.f};
    float P_posU{0.f};

    float P_velE{0.f};
    float P_velN{0.f};
    float P_velU{0.f};

    float P_yaw{0.f};
    float P_bgz{0.f};

    std::uint8_t valid{1};       ///< 当前状态是否认为“可用”
    std::uint8_t reserved1{0};
    std::uint16_t reserved2{0};
};

static_assert(std::is_trivially_copyable_v<EskfStatePacketV2>,
              "EskfStatePacketV2 must be trivially copyable");


/// @brief IMU 传播过程日志包（V1）
///
/// 用于检查：dt 是否稳定、加速度/角速度是否正常、
/// 速度积分是否出现明显漂移。
struct EskfImuPropagatePacketV1
{
    MonoTimeNs mono_ns{0};
    SysTimeNs  est_ns{0};

    float dt_s{0.f};          ///< 本次传播的时间间隔（s）
    std::uint8_t used{0};     ///< 1=本帧用于传播，0=被拒绝（dt 异常等）
    std::uint8_t reserved1{0};
    std::uint16_t reserved2{0};

    // IMU 输入（body=FRD，线加速度已扣重力）
    float wx_rad_s{0.f};
    float wy_rad_s{0.f};
    float wz_rad_s{0.f};

    float ax_mps2{0.f};
    float ay_mps2{0.f};
    float az_mps2{0.f};

    // 水平/垂向速度：传播前后对比
    float vE_before{0.f};
    float vN_before{0.f};
    float vU_before{0.f};

    float vE_after{0.f};
    float vN_after{0.f};
    float vU_after{0.f};
};

static_assert(std::is_trivially_copyable_v<EskfImuPropagatePacketV1>,
              "EskfImuPropagatePacketV1 must be trivially copyable");


/// @brief DVL 水平更新（BI → ENU）日志包（V1）
///
/// 对应 EskfUpdateDiagDvlXY 的缩写版本，以 float 存储：
///   - 预测/测量水平速度模长、比值；
///   - 创新 r 和 S 对角线；
///   - NIS（膨胀前/后）、R 膨胀倍数；
///   - 预测/测量的 vE/vN 便于对比。
struct EskfDvlXYUpdatePacketV1
{
    MonoTimeNs mono_ns{0};
    SysTimeNs  est_ns{0};

    std::uint8_t ok{0};        ///< 1=本次观测用于更新；0=被拒绝
    std::uint8_t is_zupt{0};   ///< 1=走 ZUPT 路径（z=[0,0]）
    std::uint8_t reserved1{0};
    std::uint8_t reserved2{0};
    std::uint16_t reserved3{0};
    std::uint16_t reserved4{0};

    // 预测/测量水平速度模长
    float speed_pred_h{0.f};
    float speed_meas_h{0.f};
    float ratio_pred_over_meas{0.f};

    // 创新及创新协方差对角线
    float rE{0.f};
    float rN{0.f};
    float S11{0.f};
    float S22{0.f};

    // NIS（0: 初始, 1: 膨胀后, 当前）
    float nis{0.f};
    float nis0{0.f};
    float nis1{0.f};

    // R 膨胀倍数与能量比
    float R_inflate{1.f};
    float HPHt_over_R{0.f};

    // 预测/测量的 vE/vN
    float vE_pred{0.f};
    float vN_pred{0.f};
    float vE_meas{0.f};
    float vN_meas{0.f};
};

static_assert(std::is_trivially_copyable_v<EskfDvlXYUpdatePacketV1>,
              "EskfDvlXYUpdatePacketV1 must be trivially copyable");


/// @brief DVL 垂向 vU 更新日志包（V1）
///
/// 对应 EskfUpdateDiagDvlZ：
///   - 预测/测量 vU；
///   - 残差 r、创新协方差 S、NIS。
struct EskfDvlZUpdatePacketV1
{
    MonoTimeNs mono_ns{0};
    SysTimeNs  est_ns{0};

    std::uint8_t ok{0};     ///< 1=更新成功；0=观测被拒绝
    std::uint8_t reserved1{0};
    std::uint16_t reserved2{0};
    std::uint32_t reserved3{0};

    float vU_pred{0.f};
    float vU_meas{0.f};

    float r{0.f};
    float S{0.f};
    float nis{0.f};
};

static_assert(std::is_trivially_copyable_v<EskfDvlZUpdatePacketV1>,
              "EskfDvlZUpdatePacketV1 must be trivially copyable");


/// @brief Yaw 伪观测更新日志包（V1）
///
/// 对应 EskfUpdateDiagYaw：
///   - 状态 yaw / 测量 yaw；
///   - 残差、测量噪声、NIS。
struct EskfYawUpdatePacketV1
{
    MonoTimeNs mono_ns{0};
    SysTimeNs  est_ns{0};

    std::uint8_t ok{0};     ///< 1=本次 yaw 观测用于更新
    std::uint8_t reserved1{0};
    std::uint16_t reserved2{0};
    std::uint32_t reserved3{0};

    float yaw_state_rad{0.f};
    float yaw_meas_rad{0.f};
    float yaw_residual_rad{0.f};
    float sigma_yaw_meas_rad{0.f};

    float nis{0.f};
};

static_assert(std::is_trivially_copyable_v<EskfYawUpdatePacketV1>,
              "EskfYawUpdatePacketV1 must be trivially copyable");

} // namespace nav_core::io
