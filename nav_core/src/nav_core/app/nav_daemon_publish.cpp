#include "nav_core/app/nav_daemon_publish.hpp"

#include <cstdio>

#include "nav_core/io/log_packets.hpp"

namespace nav_core::app {
namespace {

namespace io = nav_core::io;

} // namespace

shared::msg::NavState build_nav_state_output(
    const NavDaemonConfig&         cfg,
    const std::optional<ImuSample>& latest_nav_imu,
    MonoTimeNs                     last_imu_ns,
    MonoTimeNs                     last_dvl_ns,
    MonoTimeNs                     now_mono_ns,
    const NavLoopState&            loop_state,
    preprocess::ImuRtPreprocessor& imu_pp,
    estimator::EskfFilter&         eskf
#if NAV_CORE_ENABLE_GRAPH
    , estimator::NavHealthMonitor* health_monitor,
    bool                           health_monitor_enabled
#endif
)
{
    shared::msg::NavState nav{};
    const MonoTimeNs state_stamp_ns = eskf.last_propagate_time();
    fill_nav_state_kinematics(
        eskf,
        nav,
        state_stamp_ns,
        latest_nav_imu.has_value() ? &(*latest_nav_imu) : nullptr);

    NavPublishContext publish_ctx{};
    publish_ctx.publish_mono_ns = now_mono_ns;
    publish_ctx.state_stamp_ns = state_stamp_ns;
    publish_ctx.imu_timing = loop_state.last_consumed_imu_timing;
    publish_ctx.dvl_timing = loop_state.last_consumed_dvl_timing;
    publish_ctx.max_imu_age_s = cfg.loop.max_imu_age_s;
    publish_ctx.max_dvl_age_s = cfg.loop.max_dvl_age_s;
    publish_ctx.imu_enabled = cfg.imu.enable;
    publish_ctx.dvl_enabled = cfg.dvl.enable;
    publish_ctx.imu_bias_ready = imu_pp.biasReady();
    publish_ctx.imu_device_state = cfg.imu.enable
        ? loop_state.imu_runtime.binder.status().state
        : DeviceConnectionState::DISCONNECTED;
    publish_ctx.dvl_device_state = cfg.dvl.enable
        ? loop_state.dvl_runtime.binder.status().state
        : DeviceConnectionState::DISCONNECTED;
    apply_nav_publish_semantics(publish_ctx, eskf_state_is_finite(eskf), nav);

#if NAV_CORE_ENABLE_GRAPH
    if (health_monitor_enabled && health_monitor != nullptr) {
        health_monitor->update_sensor_heartbeat(now_mono_ns, last_imu_ns, last_dvl_ns);
        health_monitor->update_online_state(nav);
    }
#else
    (void)last_imu_ns;
    (void)last_dvl_ns;
#endif

    return nav;
}

void publish_nav_state_output(const shared::msg::NavState& nav,
                              MonoTimeNs now_mono_ns,
                              NavLoopState& loop_state,
                              io::NavStatePublisher& nav_pub,
                              nav_core::BinLogger* nav_state_logger,
                              nav_core::BinLogger* timing_logger,
                              NavEventCsvLogger* event_logger)
{
    if (nav_pub.ok()) {
        if (!nav_pub.publish(nav) && (loop_state.print_counter % 100) == 0) {
            std::fprintf(stderr,
                         "[nav_daemon] ERROR: NavStatePublisher::publish failed "
                         "(请检查共享内存是否被消费端正确读取)\n");
        }
    }

    if (nav_state_logger != nullptr) {
        nav_state_logger->writePod(nav);
    }

    write_timing_trace(
        timing_logger,
        io::TimingTraceKind::kNavPublished,
        SampleTiming{static_cast<MonoTimeNs>(nav.t_ns),
                     loop_state.last_consumed_imu_timing.recv_mono_ns,
                     now_mono_ns},
        now_mono_ns,
        nav.age_ms,
        static_cast<std::uint16_t>(
            (nav.valid ? io::kTimingTraceValid : io::kTimingTraceNone) |
            (nav.stale ? io::kTimingTraceStale : io::kTimingTraceNone) |
            (nav.degraded ? io::kTimingTraceDegraded : io::kTimingTraceNone)),
        nav.fault_code);

    const NavPublishSnapshot nav_snapshot{
        nav.valid != 0,
        nav.stale != 0,
        nav.degraded != 0,
        static_cast<std::uint16_t>(nav.fault_code),
        nav.status_flags,
    };
    if (event_logger != nullptr &&
        (!loop_state.have_last_nav_publish_snapshot ||
         !(nav_snapshot == loop_state.last_nav_publish_snapshot))) {
        event_logger->log_nav_publish_state_changed(now_mono_ns, nav);
        loop_state.last_nav_publish_snapshot = nav_snapshot;
        loop_state.have_last_nav_publish_snapshot = true;
    }
}

void maybe_print_nav_summary(const shared::msg::NavState& nav,
                             MonoTimeNs now_mono_ns,
                             NavLoopState& loop_state,
                             double print_interval_s)
{
    const double since_last_print_s =
        (loop_state.last_print_ns > 0)
            ? (now_mono_ns - loop_state.last_print_ns) * 1e-9
            : (print_interval_s + 1.0);

    if (since_last_print_s < print_interval_s) {
        return;
    }

    loop_state.last_print_ns = now_mono_ns;
    const double t_s = static_cast<double>(now_mono_ns) * 1e-9;

    std::fprintf(stderr,
                 "[nav_daemon] NAV t=%.1fs "
                 "E=%.3f N=%.3f U=%.3f depth=%.2f "
                 "vE=%.3f vN=%.3f vU=%.3f "
                 "yaw=%.3f valid=%u stale=%u degraded=%u "
                 "state=%u health=%u fault=%u age_ms=%u "
                 "sensor_mask=0x%04X flags=0x%04X\n",
                 t_s,
                 nav.pos[0], nav.pos[1], nav.pos[2],
                 nav.depth,
                 nav.vel[0], nav.vel[1], nav.vel[2],
                 nav.rpy[2],
                 static_cast<unsigned>(nav.valid),
                 static_cast<unsigned>(nav.stale),
                 static_cast<unsigned>(nav.degraded),
                 static_cast<unsigned>(nav.nav_state),
                 static_cast<unsigned>(nav.health),
                 static_cast<unsigned>(nav.fault_code),
                 static_cast<unsigned>(nav.age_ms),
                 static_cast<unsigned>(nav.sensor_mask),
                 static_cast<unsigned>(nav.status_flags));
}

} // namespace nav_core::app
