#include "nav_core/app/nav_daemon_dvl_pipeline.hpp"

#include <cstdio>

#include "nav_core/io/log_packets.hpp"

namespace nav_core::app {
namespace {

using DvlSample = nav_core::DvlFrame;
namespace io = nav_core::io;

void dvl_rt_output_to_sample(const preprocess::DvlRtOutput& in, DvlSample& out)
{
    out.sensor_time_ns = in.sensor_time_ns;
    out.recv_mono_ns = in.recv_mono_ns;
    out.consume_mono_ns = in.consume_mono_ns;
    out.mono_ns = in.mono_ns;
    out.est_ns  = in.est_ns;

    if (in.has_bi) {
        out.vel_body_mps[0] = static_cast<float>(in.vel_bi_body.x);
        out.vel_body_mps[1] = static_cast<float>(in.vel_bi_body.y);
        out.vel_body_mps[2] = static_cast<float>(in.vel_bi_body.z);
        out.has_body_vel = true;
    } else {
        out.vel_body_mps[0] = 0.f;
        out.vel_body_mps[1] = 0.f;
        out.vel_body_mps[2] = 0.f;
        out.has_body_vel = false;
    }

    if (in.has_be) {
        out.vel_enu_mps[0] = static_cast<float>(in.vel_be_enu.x);
        out.vel_enu_mps[1] = static_cast<float>(in.vel_be_enu.y);
        out.vel_enu_mps[2] = static_cast<float>(in.vel_be_enu.z);
        out.has_enu_vel = true;
    } else {
        out.vel_enu_mps[0] = 0.f;
        out.vel_enu_mps[1] = 0.f;
        out.vel_enu_mps[2] = 0.f;
        out.has_enu_vel = false;
    }

    out.dist_enu_m[0] = 0.f;
    out.dist_enu_m[1] = 0.f;
    out.dist_enu_m[2] = 0.f;
    out.has_dist_enu = false;

    if (in.has_alt) {
        out.altitude_m = static_cast<float>(in.alt_bottom_m);
        out.has_altitude = true;
    } else {
        out.altitude_m = 0.f;
        out.has_altitude = false;
    }

    out.bottom_lock = in.bottom_lock;
    out.valid = in.gated_ok;
    out.track_type = nav_core::DvlTrackType::BottomTrack;
    out.fom = in.mean_corr;
    out.quality = 0;
}

inline SampleTiming sample_timing_from(const preprocess::DvlRawSample& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

inline SampleTiming sample_timing_from(const DvlSample& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

} // namespace

void process_dvl_pipeline(
    const NavDaemonConfig&         cfg,
    const SharedSensorSnapshot&    snapshot,
    MonoTimeNs                     now_mono_ns,
    preprocess::DvlRtPreprocessor& dvl_pp,
    estimator::EskfFilter&         eskf,
    nav_core::BinLogger*           sample_logger,
    nav_core::BinLogger*           timing_logger,
    NavEventCsvLogger*             event_logger,
    NavLoopState&                  loop_state,
    estimator::NavHealthMonitor*   health_monitor,
    bool                           health_monitor_enabled
)
{
    if (!snapshot.dvl_raw.has_value()) {
        return;
    }

    const auto& raw_snapshot = *snapshot.dvl_raw;
    double dvl_age_s = 0.0;
    if (raw_snapshot.mono_ns > 0 && now_mono_ns >= raw_snapshot.mono_ns) {
        dvl_age_s = static_cast<double>(now_mono_ns - raw_snapshot.mono_ns) * 1e-9;
    }

    if (is_sample_fresh(now_mono_ns, raw_snapshot.mono_ns, cfg.loop.max_dvl_age_s)) {
        preprocess::DvlRawSample raw_in = raw_snapshot;

        if (should_consume_sample(raw_in.mono_ns, loop_state.last_used_dvl_ns)) {
            raw_in.consume_mono_ns = now_mono_ns;
            preprocess::DvlRtOutput dvl_out{};
            const bool ok = dvl_pp.process(raw_in, dvl_out);

            if (ok && dvl_out.gated_ok) {
                DvlSample dvl_sample{};
                dvl_rt_output_to_sample(dvl_out, dvl_sample);

                loop_state.last_consumed_dvl_timing = sample_timing_from(dvl_sample);

                estimator::EskfUpdateDiagDvlXY diag_xy{};
                estimator::EskfUpdateDiagDvlZ  diag_z{};

                if (dvl_sample.has_enu_vel) {
                    eskf.update_dvl_xy(dvl_sample, &diag_xy);
                }

                if (eskf.config().enable_dvl_z_update &&
                    (dvl_sample.has_enu_vel || dvl_sample.has_altitude)) {
                    eskf.update_dvl_z(dvl_sample, &diag_z);
                }

                if (health_monitor_enabled && health_monitor != nullptr) {
                    health_monitor->notify_dvl_sample_issue(
                        dvl_sample.mono_ns,
                        estimator::SensorAuditIssue::kAccepted);
                    if (dvl_sample.has_enu_vel) {
                        health_monitor->notify_dvl_xy_update(
                            dvl_sample.mono_ns, diag_xy.nis, diag_xy.ok);
                    }
                    if (eskf.config().enable_dvl_z_update &&
                        (dvl_sample.has_enu_vel || dvl_sample.has_altitude)) {
                        health_monitor->notify_z_update(
                            dvl_sample.mono_ns, diag_z.nis, diag_z.ok);
                    }
                }

                if (sample_logger != nullptr) {
                    sample_logger->write(&dvl_sample, sizeof(dvl_sample));
                }

                write_timing_trace(timing_logger,
                                   io::TimingTraceKind::kDvlConsumed,
                                   loop_state.last_consumed_dvl_timing,
                                   0,
                                   compute_age_ms(now_mono_ns, dvl_sample.mono_ns),
                                   io::kTimingTraceFresh | io::kTimingTraceAccepted);
                loop_state.dvl_reject_tracker.clear();
            } else {
                if (health_monitor_enabled && health_monitor != nullptr) {
                    health_monitor->notify_dvl_sample_issue(
                        raw_in.mono_ns,
                        ok ? estimator::SensorAuditIssue::kGatedRejected
                           : estimator::SensorAuditIssue::kPreprocessRejected);
                }
                const auto sample_age_ms = compute_age_ms(now_mono_ns, raw_in.mono_ns);
                const auto reject_kind = ok
                    ? SensorRejectKind::kGatedRejected
                    : SensorRejectKind::kPreprocessRejected;
                write_timing_trace(timing_logger,
                                   io::TimingTraceKind::kDvlRejected,
                                   sample_timing_from(raw_in),
                                   0,
                                   sample_age_ms,
                                   io::kTimingTraceRejected);
                if (event_logger != nullptr &&
                    loop_state.dvl_reject_tracker.should_log(reject_kind)) {
                    event_logger->log_sensor_update_rejected(
                        now_mono_ns,
                        "dvl",
                        sensor_reject_name(reject_kind),
                        sample_age_ms,
                        static_cast<std::uint16_t>(
                            sensor_reject_fault_code("dvl", reject_kind)));
                }
            }

            loop_state.last_used_dvl_ns = raw_in.mono_ns;
            return;
        }

        if (raw_snapshot.mono_ns < loop_state.last_used_dvl_ns) {
            if (health_monitor_enabled && health_monitor != nullptr) {
                health_monitor->notify_dvl_sample_issue(
                    raw_snapshot.mono_ns,
                    estimator::SensorAuditIssue::kOutOfOrder);
            }
            const auto sample_age_ms = compute_age_ms(now_mono_ns, raw_snapshot.mono_ns);
            if (event_logger != nullptr &&
                loop_state.dvl_reject_tracker.should_log(SensorRejectKind::kOutOfOrder)) {
                event_logger->log_sensor_update_rejected(
                    now_mono_ns,
                    "dvl",
                    sensor_reject_name(SensorRejectKind::kOutOfOrder),
                    sample_age_ms,
                    static_cast<std::uint16_t>(
                        sensor_reject_fault_code("dvl", SensorRejectKind::kOutOfOrder)));
            }
            if ((loop_state.print_counter % 200) == 0) {
                write_timing_trace(timing_logger,
                                   io::TimingTraceKind::kDvlRejected,
                                   sample_timing_from(raw_snapshot),
                                   0,
                                   sample_age_ms,
                                   io::kTimingTraceRejected | io::kTimingTraceOutOfOrder);
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: DVL sample out-of-order "
                             "(sample=%lld last_used=%lld), drop\n",
                             static_cast<long long>(raw_snapshot.mono_ns),
                             static_cast<long long>(loop_state.last_used_dvl_ns));
            }
        }
        return;
    }

    if (health_monitor_enabled && health_monitor != nullptr) {
        health_monitor->notify_dvl_sample_issue(
            raw_snapshot.mono_ns,
            estimator::SensorAuditIssue::kStale);
    }
    const auto sample_age_ms = compute_age_ms(now_mono_ns, raw_snapshot.mono_ns);
    if (event_logger != nullptr &&
        loop_state.dvl_reject_tracker.should_log(SensorRejectKind::kStale)) {
        event_logger->log_sensor_update_rejected(
            now_mono_ns,
            "dvl",
            sensor_reject_name(SensorRejectKind::kStale),
            sample_age_ms,
            static_cast<std::uint16_t>(
                sensor_reject_fault_code("dvl", SensorRejectKind::kStale)));
    }
    if ((loop_state.print_counter % 200) == 0) {
        write_timing_trace(timing_logger,
                           io::TimingTraceKind::kDvlRejected,
                           sample_timing_from(raw_snapshot),
                           0,
                           sample_age_ms,
                           io::kTimingTraceRejected | io::kTimingTraceStale);
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: DVL age=%.3f > max_dvl_age_s=%.3f, "
                     "treat as offline\n",
                     dvl_age_s, cfg.loop.max_dvl_age_s);
    }
}

} // namespace nav_core::app
