// nav_core/src/nav_core/estimator/nav_health_monitor.cpp

#include "nav_core/estimator/nav_health_monitor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav_core::estimator {

// ============================ 工具函数 ============================

namespace {

inline double ns_to_s(MonoTimeNs dt_ns) noexcept
{
    return static_cast<double>(dt_ns) * 1e-9;
}

inline double safe_ratio(std::size_t num, std::size_t den) noexcept
{
    if (den == 0u) {
        return 0.0;
    }
    return static_cast<double>(num) / static_cast<double>(den);
}

inline void worsen_health(shared::msg::NavHealth candidate,
                          shared::msg::NavHealth& current) noexcept
{
    if (static_cast<int>(candidate) > static_cast<int>(current)) {
        current = candidate;
    }
}

} // anonymous namespace

// ============================ NavHealthMonitor 实现 ============================

const char* nav_audit_root_cause_name(NavAuditRootCause cause) noexcept
{
    switch (cause) {
    case NavAuditRootCause::kHealthy:
        return "healthy";
    case NavAuditRootCause::kSensorInput:
        return "sensor_input";
    case NavAuditRootCause::kTransportTiming:
        return "transport_timing";
    case NavAuditRootCause::kEstimatorConsistency:
        return "estimator_consistency";
    case NavAuditRootCause::kEstimatorNumeric:
        return "estimator_numeric";
    case NavAuditRootCause::kUnknown:
    default:
        return "unknown";
    }
}

NavHealthMonitor::NavHealthMonitor(const NavHealthConfig& cfg)
    : cfg_(cfg)
{
    // stats_window_begin_ns_ 保持 0，待首次 notify_* / evaluate 时初始化
}

void NavHealthMonitor::setConfig(const NavHealthConfig& cfg) noexcept
{
    cfg_ = cfg;
}

// -------------------- 滤波器整体事件 --------------------

void NavHealthMonitor::notify_eskf_reset(MonoTimeNs now_mono_ns) noexcept
{
    eskf_initialized_   = true;
    eskf_last_reset_ns_ = now_mono_ns;

    // 重置统计窗口，从当前时刻重新累计 ESKF 相关指标
    reset_stats_window(now_mono_ns);
}

void NavHealthMonitor::notify_imu_propagate(MonoTimeNs now_mono_ns) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;
    ++imu_samples_accepted_;
}

void NavHealthMonitor::notify_imu_sample_issue(MonoTimeNs now_mono_ns,
                                               SensorAuditIssue issue) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;

    switch (issue) {
    case SensorAuditIssue::kAccepted:
        ++imu_samples_accepted_;
        break;
    case SensorAuditIssue::kPreprocessRejected:
        ++imu_preprocess_rejected_;
        break;
    case SensorAuditIssue::kStale:
        ++imu_stale_rejected_;
        break;
    case SensorAuditIssue::kOutOfOrder:
        ++imu_out_of_order_rejected_;
        break;
    case SensorAuditIssue::kGatedRejected:
    default:
        break;
    }
}

// -------------------- ESKF 观测更新事件 --------------------

void NavHealthMonitor::notify_dvl_xy_update(MonoTimeNs now_mono_ns,
                                            double nis,
                                            bool accepted) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;

    ++dvl_xy_updates_total_;
    if (accepted) {
        ++dvl_xy_updates_accepted_;
    }

    if (nis >= 0.0) {
        dvl_xy_nis_last_ = nis;
        dvl_xy_nis_sum_  += nis;
        if (nis > dvl_xy_nis_max_) {
            dvl_xy_nis_max_ = nis;
        }
    }
}

void NavHealthMonitor::notify_z_update(MonoTimeNs now_mono_ns,
                                       double nis,
                                       bool accepted) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;

    ++z_updates_total_;
    if (accepted) {
        ++z_updates_accepted_;
    }

    if (nis >= 0.0) {
        z_nis_last_ = nis;
        z_nis_sum_  += nis;
        if (nis > z_nis_max_) {
            z_nis_max_ = nis;
        }
    }
}

void NavHealthMonitor::notify_dvl_sample_issue(MonoTimeNs now_mono_ns,
                                               SensorAuditIssue issue) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;

    switch (issue) {
    case SensorAuditIssue::kAccepted:
        ++dvl_samples_accepted_;
        break;
    case SensorAuditIssue::kPreprocessRejected:
        ++dvl_preprocess_rejected_;
        break;
    case SensorAuditIssue::kStale:
        ++dvl_stale_rejected_;
        break;
    case SensorAuditIssue::kOutOfOrder:
        ++dvl_out_of_order_rejected_;
        break;
    case SensorAuditIssue::kGatedRejected:
        ++dvl_gated_rejected_;
        break;
    default:
        break;
    }
}

void NavHealthMonitor::notify_numeric_invalid(MonoTimeNs now_mono_ns) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;
    numeric_invalid_seen_ = true;
    numeric_invalid_last_ns_ = now_mono_ns;
}

// -------------------- 传感器心跳更新 --------------------

void NavHealthMonitor::update_sensor_heartbeat(MonoTimeNs now_mono_ns,
                                               MonoTimeNs last_imu_ns,
                                               MonoTimeNs last_dvl_ns) noexcept
{
    refresh_stats_window(now_mono_ns);
    last_eval_mono_ns_ = now_mono_ns;

    last_imu_ns_ = last_imu_ns;
    last_dvl_ns_ = last_dvl_ns;
}

// -------------------- 在线导航状态更新 --------------------

void NavHealthMonitor::update_online_state(const shared::msg::NavState& nav) noexcept
{
    last_nav_state_  = nav;
    has_nav_state_   = true;
}

// -------------------- 主评估接口 --------------------

NavHealthReport NavHealthMonitor::evaluate(MonoTimeNs now_mono_ns) const noexcept
{
    NavHealthReport rep{};

    // ---------- 填充基础指标 ----------
    NavHealthMetrics& m = rep.metrics;
    NavHealthDecision& d = rep.decision;

    m.eval_mono_ns     = now_mono_ns;
    m.eskf_initialized = eskf_initialized_;
    m.estimator_numeric_invalid = numeric_invalid_seen_;
    m.estimator_numeric_suspect = numeric_invalid_seen_;

    // 1) 传感器心跳
    if (last_imu_ns_ > 0) {
        m.imu_age_s = ns_to_s(now_mono_ns - last_imu_ns_);
        m.imu_alive = (m.imu_age_s * 1e9 <= static_cast<double>(cfg_.imu_timeout_ns));
    } else {
        m.imu_age_s = std::numeric_limits<double>::infinity();
        m.imu_alive = false;
    }

    if (last_dvl_ns_ > 0) {
        m.dvl_age_s = ns_to_s(now_mono_ns - last_dvl_ns_);
        m.dvl_alive = (m.dvl_age_s * 1e9 <= static_cast<double>(cfg_.dvl_timeout_ns));
    } else {
        m.dvl_age_s = std::numeric_limits<double>::infinity();
        m.dvl_alive = false;
    }

    m.imu_samples_accepted = imu_samples_accepted_;
    m.imu_preprocess_rejected = imu_preprocess_rejected_;
    m.imu_stale_rejected = imu_stale_rejected_;
    m.imu_out_of_order_rejected = imu_out_of_order_rejected_;
    m.dvl_samples_accepted = dvl_samples_accepted_;
    m.dvl_preprocess_rejected = dvl_preprocess_rejected_;
    m.dvl_gated_rejected = dvl_gated_rejected_;
    m.dvl_stale_rejected = dvl_stale_rejected_;
    m.dvl_out_of_order_rejected = dvl_out_of_order_rejected_;

    // 2) DVL XY 统计
    if (dvl_xy_updates_total_ > 0) {
        const double total   = static_cast<double>(dvl_xy_updates_total_);
        const double accepted = static_cast<double>(dvl_xy_updates_accepted_);
        const double denom   = std::max(1.0, total);

        m.dvl_xy_updates_total   = dvl_xy_updates_total_;
        m.dvl_xy_updates_accepted = dvl_xy_updates_accepted_;

        m.dvl_xy_accept_ratio    = accepted / denom;
        m.dvl_xy_nis_mean        = dvl_xy_nis_sum_ / denom;
        m.dvl_xy_nis_max         = dvl_xy_nis_max_;
        m.dvl_xy_nis_last        = dvl_xy_nis_last_;
    } else {
        // 保持默认 0 值
        m.dvl_xy_updates_total    = 0;
        m.dvl_xy_updates_accepted = 0;
        m.dvl_xy_accept_ratio     = 0.0;
        m.dvl_xy_nis_mean         = 0.0;
        m.dvl_xy_nis_max          = 0.0;
        m.dvl_xy_nis_last         = 0.0;
    }

    // 3) 垂向更新统计
    if (z_updates_total_ > 0) {
        const double total   = static_cast<double>(z_updates_total_);
        const double accepted = static_cast<double>(z_updates_accepted_);
        const double denom   = std::max(1.0, total);

        m.z_updates_total    = z_updates_total_;
        m.z_updates_accepted = z_updates_accepted_;

        m.z_accept_ratio     = accepted / denom;
        m.z_nis_mean         = z_nis_sum_ / denom;
        m.z_nis_max          = z_nis_max_;
        m.z_nis_last         = z_nis_last_;
    } else {
        m.z_updates_total    = 0;
        m.z_updates_accepted = 0;
        m.z_accept_ratio     = 0.0;
        m.z_nis_mean         = 0.0;
        m.z_nis_max          = 0.0;
        m.z_nis_last         = 0.0;
    }

    // 4) 最近一次 NavState 的 yaw / 速度。
    if (has_nav_state_) {
        m.last_yaw_rad = last_nav_state_.rpy[2];
        m.last_speed_xy_mps =
            std::hypot(last_nav_state_.vel[0], last_nav_state_.vel[1]);
    } else {
        m.last_yaw_rad = 0.0;
        m.last_speed_xy_mps = 0.0;
    }

    const std::size_t sensor_samples_total =
        m.imu_samples_accepted + m.imu_preprocess_rejected +
        m.imu_stale_rejected + m.imu_out_of_order_rejected +
        m.dvl_samples_accepted + m.dvl_preprocess_rejected +
        m.dvl_gated_rejected + m.dvl_stale_rejected +
        m.dvl_out_of_order_rejected;
    const std::size_t transport_issue_count =
        m.imu_stale_rejected + m.imu_out_of_order_rejected +
        m.dvl_stale_rejected + m.dvl_out_of_order_rejected;
    const std::size_t sensor_input_issue_count =
        m.imu_preprocess_rejected + m.dvl_preprocess_rejected + m.dvl_gated_rejected;
    const std::size_t estimator_consistency_issue_count =
        (m.dvl_xy_updates_total - m.dvl_xy_updates_accepted) +
        (m.z_updates_total - m.z_updates_accepted);

    const double transport_issue_ratio =
        safe_ratio(transport_issue_count, sensor_samples_total);
    const double sensor_input_issue_ratio =
        safe_ratio(sensor_input_issue_count, sensor_samples_total);

    const bool transport_invalid =
        !m.imu_alive ||
        (transport_issue_count >= 4u && transport_issue_ratio >= 0.45);
    const bool transport_degraded =
        !m.dvl_alive ||
        (transport_issue_count >= 2u && transport_issue_ratio >= 0.15);

    const bool sensor_input_invalid =
        sensor_input_issue_count >= 4u && sensor_input_issue_ratio >= 0.45;
    const bool sensor_input_degraded =
        sensor_input_issue_count >= 2u && sensor_input_issue_ratio >= 0.15;

    bool estimator_consistency_invalid = false;
    bool estimator_consistency_degraded = false;

    // ---------- 健康等级判定 ----------

    // 默认：若尚未初始化，则 health=UNINITIALIZED，否则从 OK 开始
    if (!eskf_initialized_) {
        d.health = shared::msg::NavHealth::UNINITIALIZED;
    } else {
        d.health = shared::msg::NavHealth::OK;
    }

    if (m.estimator_numeric_invalid) {
        d.health = shared::msg::NavHealth::INVALID;
    } else if (eskf_initialized_) {
        if (transport_invalid) {
            worsen_health(shared::msg::NavHealth::INVALID, d.health);
        } else if (transport_degraded) {
            worsen_health(shared::msg::NavHealth::DEGRADED, d.health);
        }

        if (sensor_input_invalid) {
            worsen_health(shared::msg::NavHealth::INVALID, d.health);
        } else if (sensor_input_degraded) {
            worsen_health(shared::msg::NavHealth::DEGRADED, d.health);
        }
    }

    // 2) 观测接受率与 NIS 统计（仅在 ESKF 已初始化时评估）
    if (eskf_initialized_) {
        // ---- DVL XY ----
        if (m.dvl_xy_updates_total > 0) {
            const double ratio = m.dvl_xy_accept_ratio;

            if (ratio < cfg_.dvl_accept_ratio_invalid) {
                estimator_consistency_invalid = true;
            } else if (ratio < cfg_.dvl_accept_ratio_degraded &&
                       d.health == shared::msg::NavHealth::OK) {
                estimator_consistency_degraded = true;
            }

            // 根据 NIS 均值判断
            if (m.dvl_xy_nis_mean > cfg_.dvl_nis_reject_max) {
                estimator_consistency_invalid = true;
            } else if (m.dvl_xy_nis_mean > cfg_.dvl_nis_ok_max &&
                       d.health == shared::msg::NavHealth::OK) {
                estimator_consistency_degraded = true;
            }
        }

        // ---- 垂向观测 Z ----
        if (m.z_updates_total > 0) {
            const double ratio = m.z_accept_ratio;

            if (ratio < cfg_.z_accept_ratio_invalid) {
                estimator_consistency_invalid = true;
            } else if (ratio < cfg_.z_accept_ratio_degraded &&
                       d.health == shared::msg::NavHealth::OK) {
                estimator_consistency_degraded = true;
            }

            if (m.z_nis_mean > cfg_.z_nis_reject_max) {
                estimator_consistency_invalid = true;
            } else if (m.z_nis_mean > cfg_.z_nis_ok_max &&
                       d.health == shared::msg::NavHealth::OK) {
                estimator_consistency_degraded = true;
            }
        }

        if (estimator_consistency_invalid) {
            worsen_health(shared::msg::NavHealth::INVALID, d.health);
        } else if (estimator_consistency_degraded) {
            worsen_health(shared::msg::NavHealth::DEGRADED, d.health);
        }
    }

    // 3) status_flags 填充（此处仅预留位，具体与 NavStatusFlags 的映射
    //    建议在后续根据 shared::msg::NavStatusFlags 的定义统一规划）
    d.status_flags = 0;
    // 例如将来可以做类似：
    // if (!m.imu_alive) d.status_flags |= NavStatusFlags::IMU_DEAD;
    // if (!m.dvl_alive) d.status_flags |= NavStatusFlags::DVL_DEAD;
    // if (d.health == shared::msg::NavHealth::DEGRADED) d.status_flags |= NavStatusFlags::NAV_DEGRADED;
    // if (d.health == shared::msg::NavHealth::INVALID)  d.status_flags |= NavStatusFlags::NAV_INVALID;

    // 4) 工程建议
    if (d.health == shared::msg::NavHealth::INVALID) {
        if (cfg_.enable_stop_suggestion) {
            d.recommend_stop_motion = true;
        }
        if (cfg_.enable_reloc_suggestion) {
            d.recommend_relocalize = true;
        }
    } else if (d.health == shared::msg::NavHealth::DEGRADED) {
        if (cfg_.enable_speed_reduce_suggestion) {
            d.recommend_reduce_speed = true;
        }
    }

    m.transport_timing_suspect = transport_degraded || transport_invalid;
    m.sensor_input_suspect = sensor_input_degraded || sensor_input_invalid;
    m.estimator_consistency_suspect =
        estimator_consistency_degraded || estimator_consistency_invalid;

    if (m.estimator_numeric_suspect) {
        rep.root_cause = NavAuditRootCause::kEstimatorNumeric;
    } else if (m.transport_timing_suspect &&
               (transport_issue_count >= sensor_input_issue_count ||
                !m.imu_alive || !m.dvl_alive)) {
        rep.root_cause = NavAuditRootCause::kTransportTiming;
    } else if (m.sensor_input_suspect) {
        rep.root_cause = NavAuditRootCause::kSensorInput;
    } else if (m.estimator_consistency_suspect ||
               (estimator_consistency_issue_count > 0u &&
                d.health != shared::msg::NavHealth::OK)) {
        rep.root_cause = NavAuditRootCause::kEstimatorConsistency;
    } else if (d.health == shared::msg::NavHealth::OK) {
        rep.root_cause = NavAuditRootCause::kHealthy;
    } else {
        rep.root_cause = NavAuditRootCause::kUnknown;
    }

    return rep;
}

void NavHealthMonitor::refresh_stats_window(MonoTimeNs now_mono_ns) noexcept
{
    if (stats_window_begin_ns_ == 0) {
        reset_stats_window(now_mono_ns);
        return;
    }

    const double window_s = cfg_.stats_window_duration_s;
    if (window_s <= 0.0) {
        return;
    }

    const double dt_s = ns_to_s(now_mono_ns - stats_window_begin_ns_);
    if (dt_s > window_s) {
        reset_stats_window(now_mono_ns);
    }
}

// -------------------- 辅助：重置统计窗口 --------------------

void NavHealthMonitor::reset_stats_window(MonoTimeNs new_begin_ns) noexcept
{
    stats_window_begin_ns_ = new_begin_ns;

    // DVL XY 统计
    dvl_xy_updates_total_    = 0;
    dvl_xy_updates_accepted_ = 0;
    dvl_xy_nis_sum_          = 0.0;
    dvl_xy_nis_max_          = 0.0;
    dvl_xy_nis_last_         = 0.0;

    // 垂向统计
    z_updates_total_         = 0;
    z_updates_accepted_      = 0;
    z_nis_sum_               = 0.0;
    z_nis_max_               = 0.0;
    z_nis_last_              = 0.0;

    // IMU / DVL 样本处理统计
    imu_samples_accepted_        = 0;
    imu_preprocess_rejected_     = 0;
    imu_stale_rejected_          = 0;
    imu_out_of_order_rejected_   = 0;
    dvl_samples_accepted_        = 0;
    dvl_preprocess_rejected_     = 0;
    dvl_gated_rejected_          = 0;
    dvl_stale_rejected_          = 0;
    dvl_out_of_order_rejected_   = 0;
    numeric_invalid_seen_        = false;
    numeric_invalid_last_ns_     = 0;
}

} // namespace nav_core::estimator
