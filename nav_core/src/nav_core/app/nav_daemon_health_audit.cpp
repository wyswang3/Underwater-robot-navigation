// nav_core/src/nav_core/app/nav_daemon_health_audit.cpp
//
// 作用：
//   - 把 NavHealthMonitor 的审查结果接回 nav_daemon 运行时状态；
//   - 统一输出健康等级变化、根因分类和操作员可见建议。
//
// 实现思路：
//   - 审查器只负责纯计算，这里负责把结果折叠进 NavState 语义与事件日志；
//   - 通过快照比较只在状态变化时记日志，减少运行时噪声。

#include "nav_core/app/nav_daemon_health_audit.hpp"

#include <cstdio>
#include <sstream>

namespace nav_core::app {
namespace {

using shared::msg::NavHealth;
using shared::msg::NavRunState;

void apply_health_report_to_nav(const estimator::NavHealthReport& report,
                                shared::msg::NavState&           nav) noexcept
{
    nav.status_flags = static_cast<std::uint16_t>(
        nav.status_flags | report.decision.status_flags);

    if (report.root_cause == estimator::NavAuditRootCause::kEstimatorConsistency ||
        report.root_cause == estimator::NavAuditRootCause::kEstimatorNumeric) {
        shared::msg::nav_flag_clear(nav.status_flags, shared::msg::NAV_FLAG_ESKF_OK);
    }

    if (nav.nav_state == NavRunState::kUninitialized ||
        nav.nav_state == NavRunState::kAligning ||
        report.decision.health == NavHealth::UNINITIALIZED ||
        report.decision.health == NavHealth::OK) {
        return;
    }

    if (report.decision.health == NavHealth::INVALID) {
        nav.valid = 0;
        nav.degraded = 0;
        nav.health = NavHealth::INVALID;
        nav.nav_state = NavRunState::kInvalid;
        if (report.root_cause == estimator::NavAuditRootCause::kEstimatorNumeric &&
            nav.fault_code == shared::msg::NavFaultCode::kNone) {
            nav.fault_code = shared::msg::NavFaultCode::kEstimatorNumericInvalid;
        }
        return;
    }

    if (nav.nav_state == NavRunState::kOk) {
        nav.valid = 1;
        nav.degraded = 1;
        nav.nav_state = NavRunState::kDegraded;
    }
    if (nav.health == NavHealth::OK) {
        nav.health = NavHealth::DEGRADED;
    }
}

std::string summarize_health_report(const estimator::NavHealthReport& report)
{
    std::ostringstream oss;
    const auto& m = report.metrics;

    switch (report.root_cause) {
    case estimator::NavAuditRootCause::kTransportTiming:
        oss << "imu_alive=" << static_cast<int>(m.imu_alive)
            << " dvl_alive=" << static_cast<int>(m.dvl_alive)
            << " imu_age_s=" << m.imu_age_s
            << " dvl_age_s=" << m.dvl_age_s
            << " imu_timing(stale=" << m.imu_stale_rejected
            << ",ooo=" << m.imu_out_of_order_rejected << ")"
            << " dvl_timing(stale=" << m.dvl_stale_rejected
            << ",ooo=" << m.dvl_out_of_order_rejected << ")";
        break;
    case estimator::NavAuditRootCause::kSensorInput:
        oss << "imu_pre_reject=" << m.imu_preprocess_rejected
            << " dvl_pre_reject=" << m.dvl_preprocess_rejected
            << " dvl_gate_reject=" << m.dvl_gated_rejected
            << " imu_ok=" << m.imu_samples_accepted
            << " dvl_ok=" << m.dvl_samples_accepted;
        break;
    case estimator::NavAuditRootCause::kEstimatorConsistency:
        oss << "dvl_xy_accept=" << m.dvl_xy_updates_accepted
            << "/" << m.dvl_xy_updates_total
            << " dvl_xy_ratio=" << m.dvl_xy_accept_ratio
            << " dvl_xy_nis_mean=" << m.dvl_xy_nis_mean
            << " z_accept=" << m.z_updates_accepted
            << "/" << m.z_updates_total
            << " z_ratio=" << m.z_accept_ratio
            << " z_nis_mean=" << m.z_nis_mean;
        break;
    case estimator::NavAuditRootCause::kEstimatorNumeric:
        oss << "non-finite ESKF state observed";
        break;
    case estimator::NavAuditRootCause::kHealthy:
        oss << "all monitored streams within thresholds";
        break;
    case estimator::NavAuditRootCause::kUnknown:
    default:
        oss << "health monitor has no dominant root cause";
        break;
    }

    if (report.decision.recommend_stop_motion) {
        oss << " action=stop";
    } else if (report.decision.recommend_reduce_speed) {
        oss << " action=reduce_speed";
    } else if (report.decision.recommend_relocalize) {
        oss << " action=relocalize";
    }

    return oss.str();
}

} // namespace

std::optional<estimator::NavHealthReport> evaluate_nav_health_audit(
    MonoTimeNs                   now_mono_ns,
    MonoTimeNs                   last_imu_ns,
    MonoTimeNs                   last_dvl_ns,
    estimator::NavHealthMonitor* health_monitor,
    bool                         health_monitor_enabled,
    shared::msg::NavState&       nav)
{
    if (!health_monitor_enabled || health_monitor == nullptr) {
        return std::nullopt;
    }

    if (nav.fault_code == shared::msg::NavFaultCode::kEstimatorNumericInvalid) {
        health_monitor->notify_numeric_invalid(now_mono_ns);
    }

    health_monitor->update_sensor_heartbeat(now_mono_ns, last_imu_ns, last_dvl_ns);
    health_monitor->update_online_state(nav);

    auto report = health_monitor->evaluate(now_mono_ns);
    apply_health_report_to_nav(report, nav);
    return report;
}

void publish_nav_health_audit(const NavDaemonConfig&                           cfg,
                              MonoTimeNs                                       now_mono_ns,
                              const std::optional<estimator::NavHealthReport>& report,
                              const shared::msg::NavState&                     nav,
                              NavLoopState&                                    loop_state,
                              NavEventCsvLogger*                               event_logger)
{
    if (!report.has_value()) {
        return;
    }

    const NavHealthAuditSnapshot snapshot{
        report->decision.health,
        report->root_cause,
        report->decision.recommend_stop_motion,
        report->decision.recommend_reduce_speed,
        report->decision.recommend_relocalize,
    };

    if (loop_state.have_last_health_audit_snapshot &&
        snapshot == loop_state.last_health_audit_snapshot) {
        return;
    }

    const char* root_cause = estimator::nav_audit_root_cause_name(report->root_cause);
    const std::string summary = summarize_health_report(*report);

    std::fprintf(stderr,
                 "[nav_daemon] health audit changed: health=%u cause=%s summary=%s\n",
                 static_cast<unsigned>(report->decision.health),
                 root_cause,
                 summary.c_str());

    if (event_logger != nullptr && cfg.logging.log_health_report) {
        event_logger->log_health_audit_changed(now_mono_ns, nav, root_cause, summary);
    }

    loop_state.last_health_audit_snapshot = snapshot;
    loop_state.have_last_health_audit_snapshot = true;
}

} // namespace nav_core::app
