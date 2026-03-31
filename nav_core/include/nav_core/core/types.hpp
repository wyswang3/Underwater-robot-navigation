// nav_core/include/nav_core/types.hpp
#pragma once

#include <cstdint>

namespace nav_core {

// ===================== 0. 时间类型 =====================

/// 单调时钟时间戳（纳秒），用于内部对齐 / 插值 / 导航解算主时间轴。
/// 一般来自 timebase 模块（例如 monotonic_clock_ns()）。
using MonoTimeNs = std::int64_t;

/// 兼容旧实现保留的“对外时间标量”（纳秒）。
///
/// 说明：
///  - 当前 P0 基线中，部分链路仍把它当作 sample 对应时间的兼容字段使用；
///  - 在建立真正的 epoch / wall-clock 对齐前，不应仅凭字段名把它解释为 UNIX 时间。
using SysTimeNs  = std::int64_t;


// ===================== 1. 通用向量类型 =====================

/// 轻量 3D 向量（float 版），用于不想引入 Eigen 时的基础类型。
struct Vec3f {
    float x{0.f};
    float y{0.f};
    float z{0.f};
};

/// 轻量 3D 向量（double 版），仅在需要双精度时使用。
struct Vec3d {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};


// ===================== 2. IMU 相关类型 =====================

/**
 * @brief 原始寄存器更新信息（来自 Wit SDK 的底层回调）
 *
 * 用途：
 *  - 仅用于调试 / 统计（例如观察寄存器更新频率、掉帧情况），
 *  - 不参与导航计算（导航使用 ImuFrame / ImuSample）。
 *
 * 注意：
 *  - 不拷贝寄存器数组本身（sReg[] 仍由 wit_c_sdk 管理），
 *  - 只记录这次更新的起始寄存器与数量。
 *
 * 时间：
 *  - timestamp_s：兼容旧代码的粗略系统时间（秒，不推荐新代码依赖），
 *  - host_mono_ns：推荐新代码使用的单调时间戳（纳秒）。
 */
struct ImuRawRegs {
    double     timestamp_s = 0.0;  ///< 旧接口：system_clock 下的粗略时间（秒）

    MonoTimeNs host_mono_ns{0};    ///< 新接口：主机单调时间（纳秒）

    std::uint32_t start_reg = 0;   ///< 本次更新的起始寄存器地址
    std::uint32_t count     = 0;   ///< 连续寄存器数量
};

/**
 * @brief IMU 标准化数据帧（供导航 / 滤波 / 控制使用）
 *
 * 这是整个 nav_core 中 IMU 数据的“统一表示”，驱动层与滤波层
 * 都应产出 / 消费该结构。
 *
 * 单位约定（始终一致）：
 *  - ang_vel: rad/s
 *  - lin_acc: m/s^2
 *  - euler:   rad
 *  - temperature: °C
 *
 * 坐标系约定（由调用链约定，不在此类型强制）：
 *  - 在“驱动层 → 预处理层”路径上：
 *      * ImuDriver 输出的 ImuFrame 一般仍处于“传感器坐标系 S（厂商约定 RFU）”，
 *        即：
 *          +X_s: Right, +Y_s: Forward, +Z_s: Up；
 *  - 经过 ImuRtPreprocessor 之后：
 *      * 推荐将 ang_vel / lin_acc / euler 视为“体坐标系 B=FRD”：
 *          +X_b: Forward, +Y_b: Right, +Z_b: Down。
 *  - 上层 ESKF / MHE 只假设“进入导航层的 ImuSample 已统一为 B=FRD”，
 *    因此坐标变换应在驱动 / 预处理层完成。
 *
 * 时间戳：
 *  - sensor_time_ns:
 *      * 样本真正对应的采样时刻（steady/mono 时间轴）；
 *      * 若设备本身不提供硬件时间戳，驱动层可退化为“按固定延迟回推的采样时刻”；
 *  - recv_mono_ns:
 *      * 驱动线程收到并解码该帧时的主机单调时间；
 *  - consume_mono_ns:
 *      * 主线程真正接受并用于预处理 / ESKF / 发布语义判断的时间；
 *      * 驱动输出时通常为 0，由主线程在消费路径填写；
 *  - mono_ns:
 *      * 采样时间在导航时间轴上的规范化表示；
 *      * freshness / stale / out-of-order 判断统一使用该字段；
 *  - est_ns:
 *      * 兼容旧代码保留的时间字段；
 *      * 在当前基线中要求与 mono_ns 保持一致，禁止再把它当成“独立 UNIX 时间”。
 *
 * 质量标志：
 *  - valid:  该帧是否通过 IMU 层基本过滤（NaN / 溢出 / 明显异常）；
 *  - status: 预留状态位（自检结果 / 错误码等，上层可扩展）。
 */
struct ImuFrame {
    MonoTimeNs sensor_time_ns{0};   ///< 样本对应采样时刻（steady ns）
    MonoTimeNs recv_mono_ns{0};     ///< 驱动线程收到/解码时刻（steady ns）
    MonoTimeNs consume_mono_ns{0};  ///< 主线程消费时刻（steady ns）
    MonoTimeNs mono_ns{0};          ///< 规范化样本时间（steady ns）
    SysTimeNs  est_ns{0};           ///< 兼容字段：当前必须与 mono_ns 保持一致

    float ang_vel[3]  = {0.f, 0.f, 0.f}; ///< 角速度 [wx, wy, wz] (rad/s)
    float lin_acc[3]  = {0.f, 0.f, 0.f}; ///< 线加速度 [ax, ay, az] (m/s^2)
    float euler[3]    = {0.f, 0.f, 0.f}; ///< 欧拉角 [roll, pitch, yaw] (rad)
    float temperature = 0.f;             ///< 温度 (°C)

    bool  valid  = false;                ///< 基本有效性检查是否通过
    int   status = 0;                    ///< 预留状态位（可编码自检状态 / 错误码）
};

/**
 * @brief IMU 基本过滤配置
 *
 * 用途：
 *  - 在进入 ESKF / 在线估计之前，对原始 IMU 数据做一次“硬门控”，
 *    去掉明显不合理的尖刺 / NaN / 溢出，防止污染后端解算。
 *
 * 典型策略：
 *  - 如果某轴违反 max_abs_accel / max_abs_gyro，则整帧视为 invalid；
 *  - 如果 enable_euler=true，则对欧拉角范围做额外检查。
 */
struct ImuFilterConfig {
    bool   enable_accel = true;   ///< 是否启用加速度的有效性检查
    bool   enable_gyro  = true;   ///< 是否启用角速度的有效性检查
    bool   enable_euler = false;  ///< 是否启用欧拉角的有效性检查

    double max_abs_accel = 50.0;  ///< 单轴最大 |a| (m/s^2)，约 5g
    double max_abs_gyro  = 10.0;  ///< 单轴最大 |ω| (rad/s)
    double max_euler_abs = 3.5;   ///< 单轴最大 |姿态| (rad)，约 200°
};

/// 为后端算法统一命名，可以直接把 ImuFrame 当作“采样”
using ImuSample = ImuFrame;


// ===================== 3. DVL 相关类型 =====================

/// DVL 观测的跟踪类型：未知 / 底跟踪 / 水团跟踪
///
/// 注意：
///   - 在你的工程中，在线导航目前主要使用 BottomTrack；
///   - WaterTrack 可以留给未来“流场估计 / 船速补偿”等扩展用途。
enum class DvlTrackType : std::uint8_t {
    Unknown     = 0,  ///< 未知或未提供
    BottomTrack = 1,  ///< 底跟踪（锁底）
    WaterTrack  = 2,  ///< 水团跟踪
};

/**
 * @brief DVL 标准化数据帧（供导航 / 滤波 / 轨迹解算使用）
 *
 * 这是“已解码 & 做过基本 sanity check”之后的统一格式，
 * 不再直接暴露厂商原始字符串。
 *
 * 坐标系约定：
 *  - vel_body_mps:
 *      * 体坐标系 B=FRD（X 前、Y 右、Z 下）；
 *      * 通常来自 BI / BS 帧（Instrument / Ship frame），
 *        在驱动 / 预处理层完成必要的旋转 / 安装标定后统一到 B=FRD；
 *  - vel_enu_mps:
 *      * 导航坐标系 ENU：[vE, vN, vU]；
 *      * 通常来自 BE 帧（Earth frame），或由 body 速度 + IMU 姿态计算得到；
 *  - dist_enu_m:
 *      * ENU 下累计位移 [dE, dN, dU]，一般来自 BD 帧。
 *
 * ENU 约定：
 *  - X_n = East, Y_n = North, Z_n = Up（Z 轴向上为正）。
 *
 * 时间戳：
 *  - sensor_time_ns / recv_mono_ns / consume_mono_ns 的语义与 ImuFrame 一致；
 *  - mono_ns 统一表示该 DVL 样本在导航时间轴上的采样时刻；
 *  - est_ns 为兼容字段，当前必须与 mono_ns 保持一致。
 *
 * 有效性：
 *  - has_body_vel: 本帧是否含有有效的体坐标速度；
 *  - has_enu_vel:  本帧是否含有有效的 ENU 速度；
 *  - has_dist_enu: 本帧是否含有有效的 ENU 累计位移；
 *  - has_altitude: 本帧是否含有有效的离底高度；
 *  - bottom_lock:  是否锁底成功（典型意义：bottom-track vs water-track）；
 *  - valid:        该帧是否通过 DVL 层整体过滤（结合 DvlFilterConfig）。
 */
struct DvlFrame {
    MonoTimeNs sensor_time_ns{0};   ///< 样本对应采样时刻（steady ns）
    MonoTimeNs recv_mono_ns{0};     ///< 驱动线程收到/解码时刻（steady ns）
    MonoTimeNs consume_mono_ns{0};  ///< 主线程消费时刻（steady ns）
    MonoTimeNs mono_ns{0};          ///< 统一样本时间戳（steady ns）
    SysTimeNs  est_ns{0};           ///< 兼容字段：当前必须与 mono_ns 保持一致

    // ---- 1. 底跟踪 / 水团 & 锁底状态 ----
    bool          bottom_lock{false};   ///< 是否锁底成功（底跟踪）
    DvlTrackType  track_type{DvlTrackType::Unknown};

    // ---- 2. 体坐标系速度（Body frame / Ship frame → 已统一为 B=FRD）----
    //
    // 来源：
    //   - BI / BS 帧（Instrument / Ship 坐标系）经过安装标定 → B=FRD；
    //   - 单位：m/s
    //
    float vel_body_mps[3]  = {0.f, 0.f, 0.f};  ///< [vx_b, vy_b, vz_b]
    bool  has_body_vel{false};

    // ---- 3. 地理 ENU 速度（Earth frame）----
    //
    // 来源：
    //   - 直接来自 BE 帧（Earth 坐标系 ENU）；
    //   - 或在 estimator 中由 vel_body_mps + IMU yaw/姿态旋转得到。
    //
    float vel_enu_mps[3]   = {0.f, 0.f, 0.f};  ///< [vE, vN, vU]
    bool  has_enu_vel{false};

    // ---- 4. ENU 积分位移（距离）----
    //
    // 来源：
    //   - BD 帧（Earth frame 下的累计位移）；
    //   - 若当前时刻没有 BD 则 has_dist_enu=false。
    //
    float dist_enu_m[3]    = {0.f, 0.f, 0.f};  ///< [dE, dN, dU]
    bool  has_dist_enu{false};

    // ---- 5. 高度 / 质量指标 ----
    //
    // altitude_m:
    //   - 通常为“探头到底的距离”（Down 为正），
    //   - 与 ENU 的 Up 方向可以在上层通过约定做符号转换。
    //
    float altitude_m{0.f};   ///< 离底高度（m），若无则 0
    bool  has_altitude{false}; ///< 本帧是否包含有效高度信息

    float fom{0.f};          ///< Figure of Merit / 厂家质量指标（数值越小或越大由协议定义）

    // ---- 6. 有效性与质量 ----
    bool  valid{false};      ///< 当前帧是否被认为“可用于导航”（结合 filter 配置）
    int   quality{0};        ///< 预留质量等级（可由原始 FOM / 标志映射）
};


/**
 * @brief DVL 基本过滤配置
 *
 * 用途：
 *  - 在进入在线估计 / 因子图平滑前，对 DVL 数据做基本门控；
 *  - 结合厂商提供的 valid 标志 / A/V 标志 / FOM / 高度范围等，决定
 *    哪些帧设置为 DvlFrame::valid=true。
 *
 * 典型策略：
 *  - enable_velocity = true 时，对速度范围做检查；
 *  - enable_altitude = true 时，对高度范围做检查；
 *  - only_valid_flag = true 时，仅接受底层解析出的“有效”帧；
 *  - require_all_axes = true 时，三轴速度都必须是有限数；
 *  - 若 quality < min_quality，则视为无效。
 *
 * 注意：
 *  - 这些门限只影响 DvlFrame::valid / has_* 标志的赋值，
 *    “是否在积分中置零 / 延续上一帧”由上层 estimator 决定。
 */
struct DvlFilterConfig {
    bool  enable_velocity   = true;   ///< 是否检查速度幅值
    bool  enable_altitude   = true;   ///< 是否检查高度范围

    bool  only_valid_flag   = true;   ///< 仅接受底层标记为有效的帧（依赖厂商 A/V 语义）
    bool  require_all_axes  = true;   ///< 要求三轴速度均为有限数

    float max_abs_vel_mps   = 5.0f;   ///< 单轴最大 |v|，根据量程设置；<=0 表示不检查
    float max_altitude_m    = 100.0f; ///< 最大有效高度；<=0 表示不检查
    float min_altitude_m    = 0.1f;   ///< 最小有效高度（太近可能不准）

    int   min_quality       = 0;      ///< 质量门限（0 = 不启用）
};

/// 与 IMU 一样，给后端导航算法一个统一别名。
using DvlSample = DvlFrame;

} // namespace nav_core
