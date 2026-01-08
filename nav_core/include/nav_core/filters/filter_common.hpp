// nav_core/include/nav_core/filters/filter_common.hpp
//
// @file  filter_common.hpp
// @brief 导航相关传感器（IMU / DVL）的噪声与阈值配置集中定义。
//        - 以结构体形式描述 IMU / DVL 的噪声模型与工程阈值；
//        - 给出一组“工程经验默认值”，便于在 YAML 中整体调参；
//        - 不直接参与滤波实现，仅作为配置容器被 ImuRtFilter / estimator 使用。
//
// 设计定位：
//   - 把“噪声参数 / deadband / ZUPT 阈值 / 健康阈值”等放在一个统一位置；
//   - C++ 代码中只 include 本头文件，即可拿到一整组可调参数；
//   - 默认值根据当前硬件与实验经验设置，实际项目中可通过配置文件覆盖；
//   - 不依赖任何平台特定 API，纯配置 + 内联函数，无需对应 .cpp 文件。
//

#pragma once

#include <cstdint>

namespace nav_core::filters {

// ============================ 1. IMU 噪声与阈值 ============================

/**
 * @brief IMU 噪声/阈值配置
 *
 * 用途：
 *   - 给 ImuRtFilter / ESKF 等模块提供统一的噪声与阈值参数；
 *   - 其中一部分字段通常映射到过程噪声 Q / 观测噪声 R；
 *   - 一部分用于静止检测 / deadband 等工程策略。
 *
 * 说明：
 *   - 数值单位均为 SI 制（rad/s, m/s^2 等）；
 *   - 默认值是“保守工程经验”，可以在 YAML 配置中覆盖。
 */
struct ImuNoiseThresholds {
    // --- 1) 白噪声 / 随机游走（用于 Q/R）---

    // 角速度测量白噪声标准差 [rad/s]
    float gyro_noise_std_rad_s        = 0.01f;

    // 角速度 bias 随机游走标准差 [rad/s / sqrt(Hz)]
    float gyro_bias_rw_std_rad_s_sqrtHz = 1e-4f;

    // 线加速度测量白噪声标准差 [m/s^2]
    float accel_noise_std_m_s2        = 0.05f;

    // 线加速度 bias 随机游走标准差 [m/s^2 / sqrt(Hz)]
    float accel_bias_rw_std_m_s2_sqrtHz = 5e-4f;

    // --- 2) 静止检测阈值（配合 ImuRtFilter 使用）---

    // 静止判定：|ω| < static_gyro_max_rad_s 即认为角速度足够小
    float static_gyro_max_rad_s       = 0.03f;   ///< ≈ 1.7°/s

    // 静止判定：|a| 与 g_ref 的偏差 < static_accel_dev_max_m_s2
    float static_accel_dev_max_m_s2   = 0.2f;    ///< ≈ 0.02g

    // 静止参考重力加速度 [m/s^2]
    float g_ref_m_s2                  = 9.81f;

    // --- 3) deadband（抑制随机游走 / 积分漂移）---

    // 对 |ω| < gyro_deadband_rad_s 的分量直接视为 0
    float gyro_deadband_rad_s         = 0.002f;

    // 对 |a| < accel_deadband_m_s2 的分量直接视为 0
    float accel_deadband_m_s2         = 0.01f;
};

/**
 * @brief 生成一组面向“水池实验 / 中低动态”的 IMU 默认噪声/阈值。
 *
 * 说明：
 *   - 与 ImuRtFilterConfig 的默认值保持大致一致；
 *   - 适用于你目前的 50~100 Hz IMU + 水池实验场景；
 *   - 若后续数据表明 IMU 噪声更小/更大，可统一在此处调整。
 */
[[nodiscard]] inline ImuNoiseThresholds make_default_imu_thresholds_pool()
{
    ImuNoiseThresholds t{};

    // 可根据实际 IMU 测试结果进行细调，这里给出一个保守默认
    t.gyro_noise_std_rad_s              = 0.01f;
    t.gyro_bias_rw_std_rad_s_sqrtHz     = 1e-4f;
    t.accel_noise_std_m_s2              = 0.05f;
    t.accel_bias_rw_std_m_s2_sqrtHz     = 5e-4f;

    t.static_gyro_max_rad_s             = 0.03f;
    t.static_accel_dev_max_m_s2         = 0.2f;
    t.g_ref_m_s2                        = 9.81f;

    t.gyro_deadband_rad_s               = 0.002f;
    t.accel_deadband_m_s2               = 0.01f;

    return t;
}


// ============================ 2. DVL 噪声与阈值 ============================

/**
 * @brief DVL 噪声 / 阈值配置
 *
 * 结合 Hover H1000 精度指标与工程策略：
 *   - 厂家指标：速度精度约 0.4% ± 5 mm/s；
 *   - 工程策略：将小于 ~0.08 m/s 的合速度视为“噪声地板”，用于 ZUPT；
 *   - 速度 / 高度 / 质量 FOM 的门限放在这里统一管理。
 */
struct DvlNoiseThresholds {
    // --- 1) 厂家给出的指标（供参考）---

    // 相对误差（例如 0.4% = 0.004）
    float spec_rel_error               = 0.004f;

    // 绝对误差 [m/s]（例如 ±5 mm/s = 0.005）
    float spec_abs_error_m_s           = 0.005f;

    // --- 2) 工程级噪声地板与限幅 ----

    // 将 |v| < speed_noise_floor_mps 视为噪声（例如 ~0.08 m/s）
    float speed_noise_floor_mps        = 0.08f;

    // 单轴最大允许速度幅值，用于粗暴限幅（<=0 表示不限制）
    float max_abs_speed_mps            = 5.0f;

    // 高度量程与有效高度范围
    float min_altitude_m               = 0.2f;   ///< 太近可能不可靠
    float max_altitude_m               = 100.0f; ///< 视设备量程而定（<=0 不检查）

    // DVL 底跟踪质量指标门限（0 表示不检查）
    int   min_quality                  = 0;

    // --- 3) A/V 标志策略 ---

    // 若 true，则仅接受 valid == 'A' 的速度作为“高质量速度”；
    // 若 false，则允许 valid == 'V' 的帧参与“时间轴连续性、速度置零”等策略。
    bool only_accept_valid_A           = false;
};

/**
 * @brief 生成 DVL 在“水池试验 / 中低速运动”下的默认噪声与阈值。
 *
 * 说明：
 *   - speed_noise_floor_mps = 0.08 m/s：
 *       * 来自 0.4% ± 5mm/s 的指标上加一层保守放大，用作 ZUPT / deadband；
 *   - max_abs_speed_mps = 5 m/s：
 *       * 对目前 ROV 工况已经足够大，主要是防止解析出荒谬值；
 *   - min_altitude_m / max_altitude_m：
 *       * 与 DvlFilterConfig 中的高度门限保持一致。
 */
[[nodiscard]] inline DvlNoiseThresholds make_default_dvl_thresholds_pool()
{
    DvlNoiseThresholds t{};

    t.spec_rel_error            = 0.004f;
    t.spec_abs_error_m_s        = 0.005f;

    t.speed_noise_floor_mps     = 0.08f;
    t.max_abs_speed_mps         = 5.0f;

    t.min_altitude_m            = 0.2f;
    t.max_altitude_m            = 100.0f;

    t.min_quality               = 0;
    t.only_accept_valid_A       = false;

    return t;
}


// ============================ 3. ZUPT / 速度阈值配置 ============================

/**
 * @brief 基于 DVL 速度的 ZUPT（Zero Velocity Update）配置。
 *
 * 用途：
 *   - estimator / math.hpp 在进行“速度积分 → 轨迹”时决定是否在某一段时间内
 *     将速度强制为 0，从而压制积分漂移；
 *   - 常见做法：对 |v| < threshold_mps 且持续时间 > min_duration_s 的段落，
 *     认为机器人近似静止，位置保持不变。
 */
struct ZuptConfig {
    // 合速度阈值：|v| < speed_threshold_mps 则判为“候选静止”
    double speed_threshold_mps      = 0.08;  ///< 建议略高于 DVL 噪声地板

    // 速度阈值的滞回带（可选，用于避免在边缘抖动）
    double speed_hysteresis_mps     = 0.02;

    // ZUPT 判定的最小持续时间 [s]
    double min_duration_s           = 3;

    // 是否允许在“纯 IMU 段”（无 DVL）进行 ZUPT（通常 false）
    bool   allow_imu_only_zupt      = false;
};

/**
 * @brief 生成一组面向水池轨迹实验的 ZUPT 默认参数。
 *
 * 说明：
 *   - speed_threshold_mps = 0.08 m/s 对应“明显低于正常巡航速度”的段；
 *   - min_duration_s = 0.5 s 避免短暂过零段被错误认为静止；
 *   - allow_imu_only_zupt 一般保持 false，防止 IMU 漂移主导停驻判断。
 */
[[nodiscard]] inline ZuptConfig make_default_zupt_config_pool()
{
    ZuptConfig z{};
    z.speed_threshold_mps  = 0.08;
    z.speed_hysteresis_mps = 0.02;
    z.min_duration_s       = 0.5;
    z.allow_imu_only_zupt  = false;
    return z;
}


// ============================ 4. 导航健康阈值（在线 vs 因子图） ============================

/**
 * @brief 导航健康评估阈值（在线轨迹 vs 因子图平滑轨迹对比）。
 *
 * 用途：
 *   - nav_health_monitor 模块在周期性地用因子图重建参考轨迹后，
///     使用这些阈值来判断当前在线导航是否“可接受 / 降级 / 失效”；
///   - 一旦误差指标超过硬阈值，可触发停机 / 重定位建议。
 */
struct NavHealthThresholds {
    // 在某一时间窗口内，在线轨迹与参考轨迹的 RMS 位置误差 [m]
    double max_rms_pos_error_m      = 1.0;   ///< 超过则认为“明显偏离”

    // 长时间航行中，平均水平漂移速度上线 [m/s]（例如 0.08 m/s ≈ 4.8 m/min）
    double max_drift_speed_mps      = 0.08;

    // 航向偏差阈值 [deg]（在线 yaw vs 参考 yaw，超过则判为偏航严重）
    double max_yaw_error_deg        = 10.0;

    // 统计窗口长度 [s]（例如 300 s = 5 min）
    double eval_window_s            = 300.0;
};

/**
 * @brief 生成一组默认的导航健康阈值。
 *
 * 说明：
 *   - 这些值可以看作“最低可接受导航质量”的粗略定义；
 *   - 实际工程上可在 YAML 中按任务需求进行调整。
 */
[[nodiscard]] inline NavHealthThresholds make_default_nav_health_thresholds()
{
    NavHealthThresholds h{};
    h.max_rms_pos_error_m   = 1.0;
    h.max_drift_speed_mps   = 0.03;
    h.max_yaw_error_deg     = 10.0;
    h.eval_window_s         = 300.0;
    return h;
}

} // namespace nav_core::filters
