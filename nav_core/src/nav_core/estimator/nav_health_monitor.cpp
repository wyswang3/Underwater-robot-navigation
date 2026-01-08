// nav_core/src/estimator/nav_health_monitor.cpp
//
// @file  nav_health_monitor.cpp
// @brief 导航健康监测实现：综合传感器心跳 + 在线导航状态 +
//        （预留）因子图 baseline 指标，给出 NavHealth / status_flags
//        以及一组工程化控制建议。
// 
// 当前版本说明：
//   - 已实现：
//       * IMU / DVL 心跳超时判断；
//       * NavHealth 的基础分档（UNINITIALIZED / OK / DEGRADED / INVALID）；
//       * 基于健康等级和掉线情况的停机 / 降速建议；
//       * baseline 时间窗信息缓存（但尚未使用 RMS 指标参与决策）。
//   - 尚未实现：
//       * 从 GraphSmoother2DResult 中提取 rms_xy / rms_yaw / 漂移速率等；
//       * 基于 baseline 的 OK / DEGRADED / INVALID 精细分档。
//     这些留待 GraphSmoother2DResult 结构最终确定后补充。

#include "nav_core/estimator/nav_health_monitor.hpp"

#include <cmath>
#include <limits>

namespace nav_core::estimator {

using shared::msg::NavHealth;
using shared::msg::NavStatusFlags;
using shared::msg::nav_flag_set;

// ============================ 构造 & 配置 ============================

NavHealthMonitor::NavHealthMonitor(const NavHealthConfig& cfg)
    : cfg_(cfg)
{
    // baseline_metrics_snapshot_ 已经通过 {} 零初始化
}

void NavHealthMonitor::setConfig(const NavHealthConfig& cfg) noexcept
{
    cfg_ = cfg;
}

// ============================ 传感器心跳更新 ============================

void NavHealthMonitor::update_sensor_heartbeat(MonoTimeNs now_mono_ns,
                                               MonoTimeNs last_imu_ns,
                                               MonoTimeNs last_dvl_ns) noexcept
{
    last_eval_mono_ns_ = now_mono_ns;
    last_imu_ns_       = last_imu_ns;
    last_dvl_ns_       = last_dvl_ns;
}

// ============================ 在线 NavState 更新 ============================

void NavHealthMonitor::update_online_state(const shared::msg::NavState& nav) noexcept
{
    last_nav_state_ = nav;
}

// ============================ baseline / 因子图结果更新 ============================

void NavHealthMonitor::update_baseline_from_graph(MonoTimeNs window_begin_ns,
                                                  MonoTimeNs window_end_ns,
                                                  const GraphSmoother2DResult& /*result*/)
{
    baseline_begin_ns_ = window_begin_ns;
    baseline_end_ns_   = window_end_ns;

    // 先只根据时间窗口填充 duration，其他指标留待后续从 result 中提取。
    double duration_s = 0.0;
    if (baseline_end_ns_ > baseline_begin_ns_) {
        duration_s = static_cast<double>(baseline_end_ns_ - baseline_begin_ns_) * 1e-9;
    }

    baseline_metrics_snapshot_.baseline_duration_s = duration_s;
    baseline_metrics_snapshot_.baseline_path_length_m = 0.0;
    baseline_metrics_snapshot_.rms_xy_m = 0.0;
    baseline_metrics_snapshot_.rms_yaw_rad = 0.0;
    baseline_metrics_snapshot_.drift_rate_m_per_min = 0.0;
    baseline_metrics_snapshot_.rms_z_m = 0.0;
    baseline_metrics_snapshot_.rms_speed_mps = 0.0;

    // 当前版本暂时不使用 baseline 参与健康决策，因此 has_baseline_ 仍置为 false。
    // 等 GraphSmoother2DResult 结构确定后，可以在这里解析 result：
    //
    //   - baseline_metrics_snapshot_.baseline_path_length_m = result.path_length_xy_m;
    //   - baseline_metrics_snapshot_.rms_xy_m               = result.rms_xy_m;
    //   - baseline_metrics_snapshot_.rms_yaw_rad            = result.rms_yaw_rad;
    //   - baseline_metrics_snapshot_.drift_rate_m_per_min   = result.drift_rate_m_per_min;
    //   - has_baseline_ = (duration_s >= cfg_.min_baseline_duration_s
    //                      && result.sample_count >= cfg_.min_baseline_samples);
    //
    // 这里先统一标记 baseline 不参与决策，仅作为时间窗缓存。
    has_baseline_ = false;
    baseline_metrics_snapshot_.has_baseline = false;
}

// ============================ 主评估接口 ============================

NavHealthReport NavHealthMonitor::evaluate(MonoTimeNs now_mono_ns) const noexcept
{
    NavHealthReport rep{};
    NavHealthMetrics&  met = rep.metrics;
    NavHealthDecision& dec = rep.decision;

    met.eval_mono_ns = now_mono_ns;

    // -------- 1) 传感器心跳 & age 计算 --------

    auto compute_age_s = [](MonoTimeNs now, MonoTimeNs last) -> double {
        if (last == 0) {
            // 尚无有效观测
            return std::numeric_limits<double>::infinity();
        }
        if (now <= last) {
            return 0.0;
        }
        const std::int64_t dt_ns = static_cast<std::int64_t>(now - last);
        return static_cast<double>(dt_ns) * 1e-9;
    };

    const bool has_imu_ts = (last_imu_ns_ != 0);
    const bool has_dvl_ts = (last_dvl_ns_ != 0);

    const double imu_age_s = compute_age_s(now_mono_ns, last_imu_ns_);
    const double dvl_age_s = compute_age_s(now_mono_ns, last_dvl_ns_);

    met.imu_age_s = imu_age_s;
    met.dvl_age_s = dvl_age_s;

    bool imu_alive = false;
    bool dvl_alive = false;

    if (has_imu_ts) {
        const std::int64_t dt_ns =
            (now_mono_ns > last_imu_ns_) ?
            static_cast<std::int64_t>(now_mono_ns - last_imu_ns_) : 0;
        imu_alive = (dt_ns <= cfg_.imu_timeout_ns);
    }

    if (has_dvl_ts) {
        const std::int64_t dt_ns =
            (now_mono_ns > last_dvl_ns_) ?
            static_cast<std::int64_t>(now_mono_ns - last_dvl_ns_) : 0;
        dvl_alive = (dt_ns <= cfg_.dvl_timeout_ns);
    }

    met.imu_alive = imu_alive;
    met.dvl_alive = dvl_alive;

    // -------- 2) baseline 可用性与指标填充（当前版本仅缓存，不参与分档） --------

    bool baseline_valid = false;

    if (has_baseline_) {
        // 当前版本：只根据时间窗与配置判断“理论上是否可用”，
        // 但仍然不把 baseline 纳入健康决策（baseline_valid=false）。
        const double duration_s = baseline_metrics_snapshot_.baseline_duration_s;

        if (duration_s >= cfg_.min_baseline_duration_s) {
            baseline_valid = false;  // 预留，后续再真正启用
        }
    }

    if (baseline_valid) {
        met.has_baseline           = true;
        met.baseline_duration_s    = baseline_metrics_snapshot_.baseline_duration_s;
        met.baseline_path_length_m = baseline_metrics_snapshot_.baseline_path_length_m;
        met.rms_xy_m               = baseline_metrics_snapshot_.rms_xy_m;
        met.rms_yaw_rad            = baseline_metrics_snapshot_.rms_yaw_rad;
        met.drift_rate_m_per_min   = baseline_metrics_snapshot_.drift_rate_m_per_min;
        met.rms_z_m                = baseline_metrics_snapshot_.rms_z_m;
        met.rms_speed_mps          = baseline_metrics_snapshot_.rms_speed_mps;
    } else {
        met.has_baseline           = false;
        met.baseline_duration_s    = baseline_metrics_snapshot_.baseline_duration_s;
        met.baseline_path_length_m = baseline_metrics_snapshot_.baseline_path_length_m;
        // 误差类指标默认保持 0.0，表示“未评估”
        met.rms_xy_m               = 0.0;
        met.rms_yaw_rad            = 0.0;
        met.drift_rate_m_per_min   = 0.0;
        met.rms_z_m                = 0.0;
        met.rms_speed_mps          = 0.0;
    }

    // -------- 3) NavHealth 分档策略（当前版本：基于心跳 + 是否已启动） --------

    const bool system_uninitialized =
        (!has_imu_ts && !has_dvl_ts &&
         last_nav_state_.health == NavHealth::UNINITIALIZED &&
         !has_baseline_);

    NavHealth health = NavHealth::UNINITIALIZED;

    if (system_uninitialized) {
        // 尚未真正开始导航（没有 IMU/DVL 时间戳，也没有 baseline）
        health = NavHealth::UNINITIALIZED;
    } else {
        // 默认假定“正常”，再根据传感器状态降级
        health = NavHealth::OK;

        if (!imu_alive && !dvl_alive) {
            // 主传感器全部掉线，直接 INVALID
            health = NavHealth::INVALID;
        } else if (!imu_alive || !dvl_alive) {
            // 有一个掉线，则至少降级为 DEGRADED
            health = NavHealth::DEGRADED;
        }

        // 预留：未来在这里叠加 baseline 评估结果：
        //
        // if (baseline_valid) {
        //     const double rms_xy   = met.rms_xy_m;
        //     const double rms_yaw  = met.rms_yaw_rad;
        //     const double drift_rt = met.drift_rate_m_per_min;
        //
        //     const bool pos_bad =
        //         (rms_xy > cfg_.max_rms_xy_degraded_m);
        //     const bool yaw_bad =
        //         (rms_yaw > cfg_.max_rms_yaw_degraded_rad);
        //     const bool drift_bad =
        //         (drift_rt > cfg_.max_drift_rate_degraded_m_per_min);
        //
        //     const bool pos_mid =
        //         (rms_xy > cfg_.max_rms_xy_ok_m &&
        //          rms_xy <= cfg_.max_rms_xy_degraded_m);
        //     const bool yaw_mid =
        //         (rms_yaw > cfg_.max_rms_yaw_ok_rad &&
        //          rms_yaw <= cfg_.max_rms_yaw_degraded_rad);
        //     const bool drift_mid =
        //         (drift_rt > cfg_.max_drift_rate_ok_m_per_min &&
        //          drift_rt <= cfg_.max_drift_rate_degraded_m_per_min);
        //
        //     if (pos_bad || yaw_bad || drift_bad) {
        //         health = NavHealth::INVALID;
        //     } else if (pos_mid || yaw_mid || drift_mid) {
        //         if (health == NavHealth::OK) {
        //             health = NavHealth::DEGRADED;
        //         }
        //     }
        // }
    }

    dec.health = health;

    // -------- 4) status_flags 填充（NavStatusFlags） --------

    std::uint16_t flags = 0;

    if (imu_alive) {
        nav_flag_set(flags, NavStatusFlags::NAV_FLAG_IMU_OK);
    }
    if (dvl_alive) {
        nav_flag_set(flags, NavStatusFlags::NAV_FLAG_DVL_OK);
    }

    // ESKF_OK / ALIGN_DONE 等更细粒度标志，当前版本先由其他模块决定；
    // 这里不额外设置，避免和 ESKF / 对准流程的语义混淆。

    dec.status_flags = flags;

    // -------- 5) 工程建议决策 --------

    dec.recommend_stop_motion    = false;
    dec.recommend_reduce_speed   = false;
    dec.recommend_relocalize     = false;

    if (cfg_.enable_stop_suggestion &&
        health == NavHealth::INVALID) {
        dec.recommend_stop_motion = true;
    }

    if (cfg_.enable_speed_reduce_suggestion &&
        health == NavHealth::DEGRADED) {
        dec.recommend_reduce_speed = true;
    }

    // 重定位建议：当前版本由于 baseline 尚未参与决策，这里保守为 false。
    // 等 baseline 有效后，可以结合 drift_rate_m_per_min / rms_xy_m 再做判断：
    //
    // if (cfg_.enable_reloc_suggestion && baseline_valid) {
    //     if (met.drift_rate_m_per_min > cfg_.max_drift_rate_degraded_m_per_min ||
    //         met.rms_xy_m > cfg_.max_rms_xy_degraded_m) {
    //         dec.recommend_relocalize = true;
    //     }
    // }

    return rep;
}

} // namespace nav_core::estimator
