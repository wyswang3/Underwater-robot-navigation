// nav_core/include/nav_core/estimator/nav_health_monitor.hpp
//
// @file  nav_health_monitor.hpp
// @brief ESKF 导航健康监测：基于 IMU / DVL 心跳与 ESKF 更新统计，给出
//        NavHealth / status_flags 以及控制层可用的“停机/降级建议”。
//
// 设计定位：
//   - 作为 estimator 层的“监督模块”，不直接参与状态估计；
//   - 消费三个主要信息源：
//       1) 传感器心跳（IMU / DVL 最近一次有效观测时间）；
//       2) 在线导航输出（NavState，仅用于辅助判定）；
//       3) ESKF 内部更新诊断（NIS / 残差范数 / 接受率等）；
//   - 输出：
//       * shared::msg::NavHealth（OK / DEGRADED / INVALID）；
//       * status_flags 中与导航质量相关的 bit；
//       * 工程化建议（是否建议停机 / 建议减速 / 建议重定位等）；
//       * 一组可用于日志与 GCS 展示的数值指标。
//
// 注意：
//   - 本模块不负责“因子图平滑 / 离线基准解算”，仅针对在线 ESKF；
//   - 不做任何 IO（不写日志、不发 UDP），纯计算 + 状态机；
//   - ESKF 过程量的落盘由 nav_daemon / 其他模块利用 log_packets 完成；
//   - 评估阈值由 NavHealthConfig 配置，可在运行时调整。
// 

#pragma once

#include <cstdint>
#include <cstddef>

#include "nav_core/core/types.hpp"   // MonoTimeNs / SysTimeNs 等别名
#include "nav_core/core/status.hpp"  // 通用 Status，如有需要可扩展
#include "shared/msg/nav_state.hpp"  // NavHealth / NavStatusFlags / NavState

namespace nav_core::estimator {

// ============================ 1. 配置结构 ============================

/**
 * @brief ESKF 导航健康监测配置（阈值 & 策略参数）
 *
 * 说明：
 *   - 所有单位与注释中保持一致，便于调参；
 *   - IMU / DVL 超时阈值用于判断“传感器掉线”；
 *   - NIS / 残差阈值用于判断“观测更新是否稳定 / 是否频繁被拒绝”；
 *   - 分档策略：通常先判定 INVALID，再判定 DEGRADED，最后剩下的视为 OK。
 */
struct NavHealthConfig {
    // -------- 传感器心跳超时（ns）--------
    //
    // 若 (now_ns - last_imu_ns) > imu_timeout_ns，则认为 IMU 掉线；
    // 若 (now_ns - last_dvl_ns) > dvl_timeout_ns，则认为 DVL 掉线。
    //
    std::int64_t imu_timeout_ns{200'000'000};   ///< 0.2s
    std::int64_t dvl_timeout_ns{3'000'000'000}; ///< 3.0s

    // -------- DVL 水平速度更新 NIS 阈值 --------
    //
    // 典型策略（自由度为 2 的卡方分布）：
    //   - dvl_nis <= dvl_nis_ok_max        -> 认为更新正常；
    //   - dvl_nis > dvl_nis_reject_max     -> 建议滤波器拒绝该测量；
    //
    // 本模块只用这些阈值来统计“超阈比率 / 近期 NIS 均值”等。
    //
    double dvl_nis_ok_max{9.21};       ///< 2 自由度，约为 0.99 置信度
    double dvl_nis_reject_max{16.0};   ///< 明显异常区间，可视为 outlier

    // -------- 垂向观测（DVL vU / 深度）NIS 阈值 --------
    //
    // 这里不强制区分来源，仅作为“垂向更新”的健康参考。
    //
    double z_nis_ok_max{9.21};         ///< 自由度 ~1..2 的大致 OK 区
    double z_nis_reject_max{16.0};

    // -------- 更新接受率阈值（统计一定时间窗口内）--------
    //
    // 若某类观测（例如 DVL 水平速度）在统计窗口中实际被采样了 N 次，
    // 仅有 accept_ratio * N 次被滤波器接受，则认为导航质量退化。
    //
    double dvl_accept_ratio_degraded{0.7};  ///< 接受率低于此值视为退化
    double dvl_accept_ratio_invalid{0.4};   ///< 接受率低于此值视为严重异常

    double z_accept_ratio_degraded{0.7};
    double z_accept_ratio_invalid{0.4};

    // -------- 统计窗口长度 --------
    //
    // ESKF 健康评估使用一个滑动时间窗口统计 NIS / 接受率指标。
    //
    double stats_window_duration_s{60.0}; ///< 默认 60s 时间窗

    // -------- 工程建议策略开关 --------
    //
    // 建议停机 / 重定位等是否启用由这些开关控制，
    // 避免在早期调试阶段过于敏感。
    //
    bool enable_stop_suggestion{true};        ///< 允许根据 INVALID 状态建议停机
    bool enable_reloc_suggestion{true};       ///< 允许根据观测严重异常建议重定位
    bool enable_speed_reduce_suggestion{true};///< 允许 DEGRADED 时建议减速
};

// ============================ 2. 指标与决策结构 ============================

/**
 * @brief 导航健康原始指标（便于日志与 GCS 展示）
 *
 * 注：这些字段主要来源于：
 *   - 传感器心跳 + 滤波器初始化状态；
 *   - 近期 NIS 统计（均值 / 最大值 / 超阈比率）；
 *   - 观测更新计数与接受率。
 */
struct NavHealthMetrics {
    // 评估时刻（mono ns）
    MonoTimeNs eval_mono_ns{0};

    // -------- 滤波器整体状态 --------
    bool eskf_initialized{false};    ///< ESKF 是否已完成初始对准 / 初始化

    // -------- 传感器心跳信息 --------
    bool   imu_alive{false};
    bool   dvl_alive{false};
    double imu_age_s{0.0};   ///< IMU 距离上次有效观测的时间（秒）
    double dvl_age_s{0.0};   ///< DVL 距离上次有效观测的时间（秒）

    // -------- DVL 水平速度更新统计（vE,vN）--------
    //
    // 统计窗口内的计数与 NIS 指标。
    //
    std::size_t dvl_xy_updates_total{0};      ///< 时间窗内 DVL XY 更新尝试总数
    std::size_t dvl_xy_updates_accepted{0};   ///< 其中被接受的次数

    double      dvl_xy_nis_mean{0.0};         ///< NIS 均值
    double      dvl_xy_nis_max{0.0};          ///< NIS 最大值
    double      dvl_xy_nis_last{0.0};         ///< 最近一次 NIS

    double      dvl_xy_accept_ratio{0.0};     ///< 接受率（0..1）

    // -------- 垂向观测更新统计（vU / depth / pressure）--------
    std::size_t z_updates_total{0};           ///< 时间窗内垂向更新总数
    std::size_t z_updates_accepted{0};        ///< 接受次数

    double      z_nis_mean{0.0};              ///< NIS 均值
    double      z_nis_max{0.0};               ///< NIS 最大值
    double      z_nis_last{0.0};              ///< 最近一次 NIS

    double      z_accept_ratio{0.0};          ///< 接受率（0..1）

    // -------- 预留扩展：位置 / 速度漂移估计等 --------
    //
    // 若将来需要，可由外部模块计算误差（例如与锚点 / 浮标对齐）后填充。
    //
    double est_horizontal_drift_m{0.0};       ///< 估计水平漂移（工程意义）
    double est_vertical_drift_m{0.0};         ///< 估计垂向漂移

    // 最近一次在线 NavState 的部分信息（只读辅助展示用）
    double last_yaw_rad{0.0};                 ///< 最近一次 NavState 中的 yaw
    double last_speed_xy_mps{0.0};            ///< 水平速度模长估计
};

/**
 * @brief 健康决策 & 工程建议。
 *
 * 说明：
 *   - health        : 映射到 shared::msg::NavHealth（OK / DEGRADED / INVALID）；
 *   - status_flags  : 对 NavStatusFlags 中与导航质量相关的 bit 做填充/清空；
 *   - 建议字段供控制层采纳或在 GCS 上展示。
 */
struct NavHealthDecision {
    shared::msg::NavHealth health{shared::msg::NavHealth::UNINITIALIZED};
    std::uint16_t          status_flags{0}; ///< NavStatusFlags 的组合

    bool recommend_stop_motion{false};   ///< 建议立即停机（进入 failsafe）
    bool recommend_reduce_speed{false};  ///< 建议降低速度（安全保守）
    bool recommend_relocalize{false};    ///< 建议执行重定位（回池边 / 上浮等）
};

/**
 * @brief 导航健康评估结果（一次 evaluate 的完整输出）
 */
struct NavHealthReport {
    NavHealthMetrics  metrics;   ///< 数值指标（可用于日志和 GCS 展示）
    NavHealthDecision decision;  ///< 健康等级与控制建议
};

// ============================ 3. 健康监测主类 ============================

/**
 * @brief 基于 ESKF 的导航健康监测器。
 *
 * 使用方式（典型流程）：
 *
 *   NavHealthMonitor monitor(cfg);
 *
 *   // 1) 滤波器初始化 / 重置时：
 *   monitor.notify_eskf_reset(now_mono_ns);
 *
 *   // 2) 每当 ESKF 执行一次 IMU 预测步（可选，仅统计频率）：
 *   monitor.notify_imu_propagate(now_mono_ns);
 *
 *   // 3) 每当 ESKF 执行一次 DVL 水平速度更新：
 *   monitor.notify_dvl_xy_update(now_mono_ns, nis, accepted);
 *
 *   // 4) 每当 ESKF 执行一次垂向观测更新（DVL vU / 深度 / 压力）：
 *   monitor.notify_z_update(now_mono_ns, nis, accepted);
 *
 *   // 5) 控制循环周期性更新传感器心跳与在线 NavState：
 *   monitor.update_sensor_heartbeat(now_mono_ns, last_imu_ns, last_dvl_ns);
 *   monitor.update_online_state(nav_state);
 *
 *   // 6) 周期性进行健康评估：
 *   NavHealthReport rep = monitor.evaluate(now_mono_ns);
 *
 *   // 7) 控制层根据 rep.decision.health / recommend_* 做模式切换或停机。
 */
class NavHealthMonitor {
public:
    explicit NavHealthMonitor(const NavHealthConfig& cfg = NavHealthConfig{});

    /// @brief 更新配置（完整替换）。
    void setConfig(const NavHealthConfig& cfg) noexcept;

    /// @brief 获取当前配置（只读引用）。
    const NavHealthConfig& config() const noexcept { return cfg_; }

    // -------------------- 滤波器整体事件 --------------------

    /// @brief 通知监视器“ESKF 已完成初始化 / 重置”。
    void notify_eskf_reset(MonoTimeNs now_mono_ns) noexcept;

    /// @brief 通知监视器“执行了一次 IMU 预测步”（可选，主要用于频率统计）。
    void notify_imu_propagate(MonoTimeNs now_mono_ns) noexcept;

    // -------------------- ESKF 观测更新事件 --------------------

    /**
     * @brief 通知监视器“执行了一次 DVL 水平速度更新（vE,vN）”。
     *
     * @param now_mono_ns 当前时间（mono ns）
     * @param nis         本次更新的 NIS（若未计算可传负值并在实现里忽略）
     * @param accepted    滤波器是否最终接受该观测（true=接受，false=拒绝）
     */
    void notify_dvl_xy_update(MonoTimeNs now_mono_ns,
                              double nis,
                              bool accepted) noexcept;

    /**
     * @brief 通知监视器“执行了一次垂向观测更新（vU / depth / pressure）”。
     *
     * @param now_mono_ns 当前时间（mono ns）
     * @param nis         本次更新的 NIS
     * @param accepted    是否被接受
     */
    void notify_z_update(MonoTimeNs now_mono_ns,
                         double nis,
                         bool accepted) noexcept;

    // -------------------- 传感器心跳更新 --------------------

    /**
     * @brief 更新传感器心跳信息。
     *
     * @param now_mono_ns 当前评估时间（mono ns）
     * @param last_imu_ns IMU 最近一次有效观测时间（mono ns；若未初始化可为 0）
     * @param last_dvl_ns DVL 最近一次有效观测时间（mono ns；若未初始化可为 0）
     */
    void update_sensor_heartbeat(MonoTimeNs now_mono_ns,
                                 MonoTimeNs last_imu_ns,
                                 MonoTimeNs last_dvl_ns) noexcept;

    // -------------------- 在线导航状态更新 --------------------

    /**
     * @brief 更新最近的在线导航状态（NavState）。
     *
     * 说明：
     *   - 本接口不会做复杂计算，只是缓存最近一次 NavState；
     *   - evaluate() 时可以结合 yaw / 速度等信息做辅助判断或展示；
     *   - 若不需要，可选择不调用，本模块仍然可以工作。
     */
    void update_online_state(const shared::msg::NavState& nav) noexcept;

    // -------------------- 主评估接口 --------------------

    /**
     * @brief 在给定时间点进行一次导航健康评估。
     *
     * @param now_mono_ns 当前评估时间（mono ns）
     * @return NavHealthReport 评估结果（含指标与决策）
     *
     * 典型评估逻辑（在 .cpp 中实现）：
     *   1) 基于传感器心跳计算 imu_alive/dvl_alive 及 age_s；
     *   2) 在配置的时间窗口内统计 DVL / 垂向观测更新的 NIS 与接受率；
     *   3) 根据 NavHealthConfig 中的 NIS / 接受率阈值，判定健康档位：
     *        - 有严重传感器掉线或接受率极低 → INVALID；
     *        - 指标处于中间区间 → DEGRADED；
     *        - 其余情况 → OK；
     *   4) 结合 health 档位与传感器状态，设置 recommend_* 建议；
     *   5) 根据 health & 传感器状态填充 status_flags（NavStatusFlags）。
     */
    NavHealthReport evaluate(MonoTimeNs now_mono_ns) const noexcept;

private:
    NavHealthConfig cfg_{};

    // 传感器心跳缓存
    MonoTimeNs last_eval_mono_ns_{0};
    MonoTimeNs last_imu_ns_{0};
    MonoTimeNs last_dvl_ns_{0};

    // 最近一次在线导航状态（可选）
    shared::msg::NavState last_nav_state_{};
    bool                  has_nav_state_{false};

    // ESKF 初始化标志
    bool      eskf_initialized_{false};
    MonoTimeNs eskf_last_reset_ns_{0};

    // -------------------- 统计窗口内部状态 --------------------
    //
    // 简化设计：仅记录“当前统计窗口”的汇总量，具体窗口对齐策略
    // 由 .cpp 中根据 stats_window_duration_s 和时间戳实现。
    //
    // DVL XY 更新统计
    std::size_t dvl_xy_updates_total_{0};
    std::size_t dvl_xy_updates_accepted_{0};
    double      dvl_xy_nis_sum_{0.0};
    double      dvl_xy_nis_max_{0.0};
    double      dvl_xy_nis_last_{0.0};

    // 垂向更新统计
    std::size_t z_updates_total_{0};
    std::size_t z_updates_accepted_{0};
    double      z_nis_sum_{0.0};
    double      z_nis_max_{0.0};
    double      z_nis_last_{0.0};

    // 统计窗口起点（mono ns）
    MonoTimeNs stats_window_begin_ns_{0};

    // 辅助：重置统计窗口
    void reset_stats_window(MonoTimeNs new_begin_ns) noexcept;
};

} // namespace nav_core::estimator
