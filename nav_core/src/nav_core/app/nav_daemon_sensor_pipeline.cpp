#include "nav_core/app/nav_daemon_sensor_pipeline.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

#include "nav_core/io/log_packets.hpp"

namespace nav_core::app {
namespace {

using DvlSample = nav_core::DvlFrame;
namespace io = nav_core::io;

void imu_frame_to_sample_raw(const ImuFrame& in, ImuSample& out)
{
    out.sensor_time_ns = in.sensor_time_ns;
    out.recv_mono_ns = in.recv_mono_ns;
    out.consume_mono_ns = in.consume_mono_ns;
    out.mono_ns = in.mono_ns;
    out.est_ns  = in.est_ns;

    for (int i = 0; i < 3; ++i) {
        out.ang_vel[i] = in.ang_vel[i];
        out.lin_acc[i] = in.lin_acc[i];
        out.euler[i] = in.euler[i];
    }

    out.temperature = in.temperature;
    out.valid = in.valid;
    out.status = in.status;
}

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

inline SampleTiming sample_timing_from(const ImuSample& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

inline SampleTiming sample_timing_from(const preprocess::DvlRawSample& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

inline SampleTiming sample_timing_from(const DvlSample& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

void process_imu_pipeline(const NavDaemonConfig&         cfg,
                          const SharedSensorSnapshot&    snapshot,
                          MonoTimeNs                     now_mono_ns,
                          preprocess::ImuRtPreprocessor& imu_pp,
                          estimator::EskfFilter&         eskf,
                          nav_core::BinLogger*           timing_logger,
                          NavEventCsvLogger*             event_logger,
                          NavLoopState&                  loop_state,
                          ProcessedLoopSamples&          out
#if NAV_CORE_ENABLE_GRAPH
                          , estimator::NavHealthMonitor* health_monitor,
                          bool                           health_monitor_enabled
#endif
)
{
    if (!snapshot.imu_frame.has_value()) {
        return;
    }

    const ImuFrame& frame_in = *snapshot.imu_frame;
    double imu_age_s = 0.0;
    if (frame_in.mono_ns > 0 && now_mono_ns >= frame_in.mono_ns) {
        imu_age_s = static_cast<double>(now_mono_ns - frame_in.mono_ns) * 1e-9;
    }

    if (is_sample_fresh(now_mono_ns, frame_in.mono_ns, cfg.loop.max_imu_age_s) &&
        should_consume_sample(frame_in.mono_ns, loop_state.last_used_imu_ns)) {
        ImuSample samp_in{};
        imu_frame_to_sample_raw(frame_in, samp_in);
        samp_in.consume_mono_ns = now_mono_ns;
        samp_in.est_ns = samp_in.mono_ns;

        ImuSample samp_out{};
        const bool ok = imu_pp.process(samp_in, samp_out);
        if (!ok) {
            const auto sample_age_ms = compute_age_ms(now_mono_ns, samp_in.mono_ns);
            write_timing_trace(timing_logger,
                               io::TimingTraceKind::kImuRejected,
                               sample_timing_from(samp_in),
                               0,
                               sample_age_ms,
                               io::kTimingTraceRejected);
            if (event_logger != nullptr &&
                loop_state.imu_reject_tracker.should_log(SensorRejectKind::kPreprocessRejected)) {
                event_logger->log_sensor_update_rejected(
                    now_mono_ns,
                    "imu",
                    sensor_reject_name(SensorRejectKind::kPreprocessRejected),
                    sample_age_ms,
                    static_cast<std::uint16_t>(
                        sensor_reject_fault_code("imu", SensorRejectKind::kPreprocessRejected)));
            }
            if ((loop_state.print_counter % 50) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: ImuRtPreprocessor rejected a sample "
                             "(可能是 NaN / 无效欧拉角 / 静止窗尚未建立 bias)\n");
            }
        } else {
            samp_out.consume_mono_ns = samp_in.consume_mono_ns;
            samp_out.est_ns = samp_out.mono_ns;
            out.latest_nav_imu = samp_out;
            loop_state.last_consumed_nav_imu = samp_out;
            loop_state.last_consumed_imu_timing = sample_timing_from(samp_out);
            loop_state.last_used_imu_ns = samp_out.mono_ns;
            const bool imu_used = eskf.propagate_imu(samp_out);
#if NAV_CORE_ENABLE_GRAPH
            if (health_monitor_enabled && health_monitor != nullptr && imu_used) {
                health_monitor->notify_imu_propagate(samp_out.mono_ns);
            }
#else
            (void)imu_used;
#endif
            write_timing_trace(timing_logger,
                               io::TimingTraceKind::kImuConsumed,
                               loop_state.last_consumed_imu_timing,
                               0,
                               compute_age_ms(now_mono_ns, samp_out.mono_ns),
                               io::kTimingTraceFresh | io::kTimingTraceAccepted);
            loop_state.imu_reject_tracker.clear();
        }
        loop_state.last_used_imu_ns = frame_in.mono_ns;
        return;
    }

    if (!is_sample_fresh(now_mono_ns, frame_in.mono_ns, cfg.loop.max_imu_age_s)) {
        const auto sample_age_ms = compute_age_ms(now_mono_ns, frame_in.mono_ns);
        if (event_logger != nullptr &&
            loop_state.imu_reject_tracker.should_log(SensorRejectKind::kStale)) {
            event_logger->log_sensor_update_rejected(
                now_mono_ns,
                "imu",
                sensor_reject_name(SensorRejectKind::kStale),
                sample_age_ms,
                static_cast<std::uint16_t>(
                    sensor_reject_fault_code("imu", SensorRejectKind::kStale)));
        }
        if ((loop_state.print_counter % 200) == 0) {
            write_timing_trace(timing_logger,
                               io::TimingTraceKind::kImuRejected,
                               sample_timing_from(frame_in),
                               0,
                               sample_age_ms,
                               io::kTimingTraceRejected | io::kTimingTraceStale);
            std::fprintf(stderr,
                         "[nav_daemon] WARNING: IMU sample age=%.3f > max_imu_age_s=%.3f, "
                         "skip stale sample\n",
                         imu_age_s, cfg.loop.max_imu_age_s);
        }
        return;
    }

    if (!should_consume_sample(frame_in.mono_ns, loop_state.last_used_imu_ns) &&
        frame_in.mono_ns < loop_state.last_used_imu_ns) {
        const auto sample_age_ms = compute_age_ms(now_mono_ns, frame_in.mono_ns);
        if (event_logger != nullptr &&
            loop_state.imu_reject_tracker.should_log(SensorRejectKind::kOutOfOrder)) {
            event_logger->log_sensor_update_rejected(
                now_mono_ns,
                "imu",
                sensor_reject_name(SensorRejectKind::kOutOfOrder),
                sample_age_ms,
                static_cast<std::uint16_t>(
                    sensor_reject_fault_code("imu", SensorRejectKind::kOutOfOrder)));
        }
        if ((loop_state.print_counter % 200) == 0) {
            write_timing_trace(timing_logger,
                               io::TimingTraceKind::kImuRejected,
                               sample_timing_from(frame_in),
                               0,
                               sample_age_ms,
                               io::kTimingTraceRejected | io::kTimingTraceOutOfOrder);
            std::fprintf(stderr,
                         "[nav_daemon] WARNING: IMU sample out-of-order "
                         "(sample=%lld last_used=%lld), drop\n",
                         static_cast<long long>(frame_in.mono_ns),
                         static_cast<long long>(loop_state.last_used_imu_ns));
        }
    }
}

void process_dvl_pipeline(const NavDaemonConfig&         cfg,
                          const SharedSensorSnapshot&    snapshot,
                          MonoTimeNs                     now_mono_ns,
                          preprocess::DvlRtPreprocessor& dvl_pp,
                          estimator::EskfFilter&         eskf,
                          nav_core::BinLogger*           sample_logger,
                          nav_core::BinLogger*           timing_logger,
                          NavEventCsvLogger*             event_logger,
                          NavLoopState&                  loop_state
#if NAV_CORE_ENABLE_GRAPH
                          , estimator::NavHealthMonitor* health_monitor,
                          bool                           health_monitor_enabled
#endif
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

#if NAV_CORE_ENABLE_GRAPH
                if (health_monitor_enabled && health_monitor != nullptr) {
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
#endif

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

        if (raw_in.mono_ns < loop_state.last_used_dvl_ns) {
            const auto sample_age_ms = compute_age_ms(now_mono_ns, raw_in.mono_ns);
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
                                   sample_timing_from(raw_in),
                                   0,
                                   sample_age_ms,
                                   io::kTimingTraceRejected | io::kTimingTraceOutOfOrder);
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: DVL sample out-of-order "
                             "(sample=%lld last_used=%lld), drop\n",
                             static_cast<long long>(raw_in.mono_ns),
                             static_cast<long long>(loop_state.last_used_dvl_ns));
            }
        }
        return;
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

} // namespace

ProcessedLoopSamples process_sensor_pipelines(
    const NavDaemonConfig&         cfg,
    const SharedSensorSnapshot&    snapshot,
    MonoTimeNs                     now_mono_ns,
    preprocess::ImuRtPreprocessor& imu_pp,
    preprocess::DvlRtPreprocessor& dvl_pp,
    estimator::EskfFilter&         eskf,
    nav_core::BinLogger*           sample_logger,
    nav_core::BinLogger*           timing_logger,
    NavEventCsvLogger*             event_logger,
    NavLoopState&                  loop_state
#if NAV_CORE_ENABLE_GRAPH
    , estimator::NavHealthMonitor* health_monitor,
    bool                           health_monitor_enabled
#endif
)
{
    ProcessedLoopSamples out{};
    out.last_imu_ns = snapshot.last_imu_ns;
    out.last_dvl_ns = snapshot.last_dvl_ns;

    process_imu_pipeline(cfg,
                         snapshot,
                         now_mono_ns,
                         imu_pp,
                         eskf,
                         timing_logger,
                         event_logger,
                         loop_state,
                         out
#if NAV_CORE_ENABLE_GRAPH
                         , health_monitor,
                         health_monitor_enabled
#endif
    );

    if (!out.latest_nav_imu.has_value() &&
        loop_state.last_consumed_nav_imu.has_value() &&
        is_sample_fresh(now_mono_ns, loop_state.last_consumed_imu_timing, cfg.loop.max_imu_age_s)) {
        out.latest_nav_imu = loop_state.last_consumed_nav_imu;
    }

    process_dvl_pipeline(cfg,
                         snapshot,
                         now_mono_ns,
                         dvl_pp,
                         eskf,
                         sample_logger,
                         timing_logger,
                         event_logger,
                         loop_state
#if NAV_CORE_ENABLE_GRAPH
                         , health_monitor,
                         health_monitor_enabled
#endif
    );

    return out;
}

} // namespace nav_core::app
