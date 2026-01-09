// nav_core/include/nav_core/estimator/nav_health_monitor.hpp
//
// @file  nav_health_monitor.hpp
// @brief 导航健康监测：综合传感器心跳 + 在线导航状态 + 因子图 baseline，
//        给出 NavHealth / status_flags 以及控制层可用的“停机/降级建议”。
//
// 设计定位：
//   - 作为 estimator 层的“监督模块”，不直接参与状态估计；
//   - 消费三个主要信息源：
//       1) 传感器心跳（IMU / DVL 最近一次观测时间）；
//       2) 在线导航输出（NavState）；
//       3) 离线/准在线因子图平滑得到的 baseline 评估指标；
//   - 输出：
//       * shared::msg::NavHealth（OK / DEGRADED / INVALID）；
//       * status_flags 中与导航质量相关的 bit；
//       * 工程化建议（是否建议停机 / 建议减速 / 建议重定位等）；
//       * 一组可用于日志与 GCS 展示的数值指标。
//
// 注意：
//   - 本模块不负责“跑因子图”，只消耗 GraphSmoother2DResult 等结果；
//   - 不做任何 IO（不写日志、不发 UDP），纯计算 + 状态机；
//   - 评估策略（阈值、分档规则）由 NavHealthConfig 控制，可在运行时调整。
// 

#pragma once

#include <cstdint>

#include "nav_core/core/types.hpp"          // MonoTimeNs / SysTimeNs 等别名
#include "nav_core/core/status.hpp"    // 通用 Status
#include "shared/msg/nav_state.hpp"    // NavHealth / NavStatusFlags / NavState

namespace nav_core::estimator {

// 前向声明：避免在头文件中强依赖 graph_smoother_2d.hpp
struct GraphSmoother2DResult;

// ============================ 1. 配置结构 ============================

/**
 * @brief 导航健康监测配置（阈值 & 策略参数）
 *
 * 说明：
 *   - 所有单位与注释中保持一致，便于调参；
 *   - IMU / DVL 超时阈值用于判断“传感器掉线”；
 *   - baseline 相关阈值用于比较“在线导航 vs 因子图平滑参考”的一致性；
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

    // -------- baseline 有效性要求 --------
    //
    // 因子图 baseline 若时间窗太短 / 样本太少，可能不具代表性；
    // 只有满足以下条件时，才把 baseline 作为“可信参考”参与评估。
    //
    double       min_baseline_duration_s{60.0}; ///< baseline 最小时间跨度（秒）
    std::size_t  min_baseline_samples{100};      ///< baseline 最小样本数

    // -------- 在线 vs baseline 位置误差阈值（XY 平面）--------
    //
    // 典型策略：
    //   - rms_xy_m <= max_rms_xy_ok_m          -> OK
    //   - max_rms_xy_ok_m < rms_xy_m <= max_rms_xy_degraded_m -> DEGRADED
    //   - rms_xy_m > max_rms_xy_degraded_m    -> INVALID
    //
    double max_rms_xy_ok_m{0.5};        ///< OK 区间上界（米）
    double max_rms_xy_degraded_m{2.0};  ///< DEGRADED 区间上界（米）

    // -------- 在线 vs baseline 航向误差阈值 --------
    //
    // 单位 rad；可按 3° / 10° 等经验值设置。
    //
    double max_rms_yaw_ok_rad{3.0 * 3.14159265358979323846 / 180.0};   ///< ≈3°
    double max_rms_yaw_degraded_rad{10.0 * 3.14159265358979323846 / 180.0}; ///< ≈10°

    // -------- 漂移速率限制（可选）--------
    //
    // 用于约束“位置漂移速度”，例如每分钟不能超过多少米。
    //
    double max_drift_rate_ok_m_per_min{1.0};       ///< OK 区间最大漂移速率
    double max_drift_rate_degraded_m_per_min{5.0}; ///< DEGRADED 区间最大漂移速率

    // -------- 工程建议策略开关 --------
    //
    // 建议停机 / 重定位等是否启用由这些开关控制，
    // 避免在早期调试阶段过于敏感。
    //
    bool enable_stop_suggestion{true};        ///< 允许根据 INVALID 状态建议停机
    bool enable_reloc_suggestion{true};       ///< 允许根据大漂移建议重定位
    bool enable_speed_reduce_suggestion{true};///< 允许 DEGRADED 时建议减速
};

// ============================ 2. 指标与决策结构 ============================

/**
 * @brief 导航健康原始指标（便于日志与 GCS 展示）
 */
struct NavHealthMetrics {
    // 评估时刻（mono ns）
    MonoTimeNs eval_mono_ns{0};

    // 传感器心跳信息
    bool   imu_alive{false};
    bool   dvl_alive{false};
    double imu_age_s{0.0};   ///< IMU 距离上次观测的时间（秒）
    double dvl_age_s{0.0};   ///< DVL 距离上次观测的时间（秒）

    // baseline 相关指标
    bool   has_baseline{false};         ///< 是否有可用 baseline
    double baseline_duration_s{0.0};    ///< baseline 时间跨度
    double baseline_path_length_m{0.0}; ///< baseline 轨迹总长度（XY）

    // 在线 vs baseline 对齐后的误差指标
    double rms_xy_m{0.0};        ///< 平面位置 RMS 误差（米）
    double rms_yaw_rad{0.0};     ///< 航向 RMS 误差（rad）
    double drift_rate_m_per_min{0.0}; ///< 漂移速率估计（米/分钟）

    // 预留扩展字段（例如 Z 方向误差、速度误差等）
    double rms_z_m{0.0};
    double rms_speed_mps{0.0};
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
    bool recommend_relocalize{false};    ///< 建议执行重定位（如回池边 / 上浮）
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
 * @brief 导航健康监测器。
 *
 * 使用方式（典型流程）：
 *
 *   NavHealthMonitor monitor(cfg);
 *
 *   // 1) 每当收到 NavState（来自 online estimator）：
 *   monitor.update_online_state(nav_state);
 *
 *   // 2) 每当完成一轮因子图平滑：
 *   monitor.update_baseline_from_graph(window_begin_ns,
 *                                      window_end_ns,
 *                                      smoother_result);
 *
 *   // 3) 在控制循环中周期性更新传感器心跳和评估：
 *   monitor.update_sensor_heartbeat(now_ns, last_imu_ns, last_dvl_ns);
 *   NavHealthReport rep = monitor.evaluate(now_ns);
 *
 *   // 4) 控制层根据 rep.decision.health / recommend_* 做模式切换或停机。
 */
class NavHealthMonitor {
public:
    explicit NavHealthMonitor(const NavHealthConfig& cfg = NavHealthConfig{});

    /// @brief 更新配置（完整替换）。
    void setConfig(const NavHealthConfig& cfg) noexcept;

    /// @brief 获取当前配置（只读引用）。
    const NavHealthConfig& config() const noexcept { return cfg_; }

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
     *   - evaluate() 时可以结合该状态（例如 yaw、速度等）做额外判断；
     *   - 若不需要，可选择不调用，本模块仍然可以基于 baseline 和心跳工作。
     */
    void update_online_state(const shared::msg::NavState& nav) noexcept;

    // -------------------- baseline / 因子图结果输入 --------------------

    /**
     * @brief 使用因子图平滑结果更新 baseline 评估。
     *
     * @param window_begin_ns 对应数据时间窗的起始（mono ns）
     * @param window_end_ns   对应数据时间窗的结束（mono ns）
     * @param result          因子图平滑结果（内部会提取误差指标）
     *
     * 约定：
     *   - 此接口由“离线/准在线平滑模块”在每次完成因子图后调用；
     *   - NavHealthMonitor 内部只保留一份最近的 baseline 概要指标，
     *     不保存完整轨迹数组，以控制内存占用；
     *   - 若 result 不满足 min_baseline_duration_s / min_baseline_samples 等条件，
     *     内部会标记 has_baseline=false。
     */
    void update_baseline_from_graph(MonoTimeNs window_begin_ns,
                                    MonoTimeNs window_end_ns,
                                    const GraphSmoother2DResult& result);

    // -------------------- 主评估接口 --------------------

    /**
     * @brief 在给定时间点进行一次导航健康评估。
     *
     * @param now_mono_ns 当前评估时间（mono ns）
     * @return NavHealthReport 评估结果（含指标与决策）
     *
     * 典型评估逻辑（在 .cpp 中实现）：
     *   1) 基于传感器心跳计算 imu_alive/dvl_alive 及 age_s；
     *   2) 检查 baseline 是否存在且满足配置要求；
     *   3) 综合 baseline 中的 rms_xy_m / rms_yaw_rad / drift_rate 等指标，
     *      与 NavHealthConfig 阈值比较，得出 health 档位；
     *   4) 结合传感器掉线情况、health 档位，设置 recommend_* 建议；
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

    // 最近一次 baseline 概要信息
    bool        has_baseline_{false};
    MonoTimeNs  baseline_begin_ns_{0};
    MonoTimeNs  baseline_end_ns_{0};
    NavHealthMetrics baseline_metrics_snapshot_{}; ///< 仅使用其中 baseline 相关字段
};

} // namespace nav_core::estimator
