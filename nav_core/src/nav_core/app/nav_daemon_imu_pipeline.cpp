#include "nav_core/app/nav_daemon_imu_pipeline.hpp"

#include <cstdio>

#include "nav_core/io/log_packets.hpp"

namespace nav_core::app {
namespace {

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

inline SampleTiming sample_timing_from(const ImuSample& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

inline SampleTiming sample_timing_from_frame(const ImuFrame& sample) noexcept
{
    return SampleTiming{sample.mono_ns, sample.recv_mono_ns, sample.consume_mono_ns};
}

} // namespace

std::optional<ImuSample> process_imu_pipeline(
    const NavDaemonConfig&         cfg,
    const SharedSensorSnapshot&    snapshot,
    MonoTimeNs                     now_mono_ns,
    preprocess::ImuRtPreprocessor& imu_pp,
    estimator::EskfFilter&         eskf,
    nav_core::BinLogger*           timing_logger,
    NavEventCsvLogger*             event_logger,
    NavLoopState&                  loop_state
#if NAV_CORE_ENABLE_GRAPH
    , estimator::NavHealthMonitor* health_monitor,
    bool                           health_monitor_enabled
#endif
)
{
    std::optional<ImuSample> latest_nav_imu;

    if (snapshot.imu_frame.has_value()) {
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
                    loop_state.imu_reject_tracker.should_log(
                        SensorRejectKind::kPreprocessRejected)) {
                    event_logger->log_sensor_update_rejected(
                        now_mono_ns,
                        "imu",
                        sensor_reject_name(SensorRejectKind::kPreprocessRejected),
                        sample_age_ms,
                        static_cast<std::uint16_t>(sensor_reject_fault_code(
                            "imu", SensorRejectKind::kPreprocessRejected)));
                }
                if ((loop_state.print_counter % 50) == 0) {
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: ImuRtPreprocessor rejected a sample "
                                 "(可能是 NaN / 无效欧拉角 / 静止窗尚未建立 bias)\n");
                }
            } else {
                samp_out.consume_mono_ns = samp_in.consume_mono_ns;
                samp_out.est_ns = samp_out.mono_ns;
                latest_nav_imu = samp_out;
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
        } else if (!is_sample_fresh(now_mono_ns, frame_in.mono_ns, cfg.loop.max_imu_age_s)) {
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
                                   sample_timing_from_frame(frame_in),
                                   0,
                                   sample_age_ms,
                                   io::kTimingTraceRejected | io::kTimingTraceStale);
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: IMU sample age=%.3f > max_imu_age_s=%.3f, "
                             "skip stale sample\n",
                             imu_age_s, cfg.loop.max_imu_age_s);
            }
        } else if (!should_consume_sample(frame_in.mono_ns, loop_state.last_used_imu_ns) &&
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
                                   sample_timing_from_frame(frame_in),
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

    if (!latest_nav_imu.has_value() &&
        loop_state.last_consumed_nav_imu.has_value() &&
        is_sample_fresh(now_mono_ns, loop_state.last_consumed_imu_timing, cfg.loop.max_imu_age_s)) {
        latest_nav_imu = loop_state.last_consumed_nav_imu;
    }

    return latest_nav_imu;
}

} // namespace nav_core::app
