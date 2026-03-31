// src/nav_core/app/nav_daemon_runner.cpp
//
// 角色：实现 nav_daemon 的完整业务管线：
//   - 共享传感器状态 SharedSensorState
//   - IMU / DVL 驱动初始化
//   - ImuRtPreprocessor / DvlRtPreprocessor / EskfFilter / NavStatePublisher / BinLogger 初始化
//   - 主循环 run_main_loop(..., stop_flag)
//   - 对外统一入口 app::run_nav_daemon(...)
//
// main() 不再出现在本文件中。

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_devices.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/imu_port_selector.hpp"
#include "nav_core/app/nav_runtime_status.hpp"
#include "nav_core/app/sample_timing.hpp"

#include "nav_core/core/types.hpp"
#include "nav_core/core/timebase.hpp"
#include "nav_core/core/status.hpp"

#include "nav_core/drivers/imu_driver_wit.hpp"
#include "nav_core/drivers/dvl_driver.hpp"

#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"
#include "nav_core/preprocess/imu_rt_preprocessor.hpp"

#include "nav_core/estimator/eskf.hpp"

#include "nav_core/io/nav_state_publisher.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/io/log_packets.hpp"

#include "shared/msg/nav_state.hpp"

namespace {

using nav_core::MonoTimeNs;
using nav_core::ImuFrame;
using nav_core::ImuSample;
using nav_core::DvlFrame;          // ESKF 那边用的 DvlSample = DvlFrame

namespace app        = nav_core::app;
namespace drivers    = nav_core::drivers;
namespace preprocess = nav_core::preprocess;
namespace estimator  = nav_core::estimator;
namespace io         = nav_core::io;
namespace smsg       = shared::msg;

using DvlSample      = nav_core::DvlFrame;    // 方便后面写 update_dvl_xy
using ProtoTrackType = nav_core::dvl_protocol::TrackType;               // = dvl_protocol::TrackType
using drivers::DvlCoordFrame;                // = dvl_protocol::CoordFrame
using app::SampleTiming;
using app::ManagedDeviceRuntime;
using app::NavEventCsvLogger;
using app::NavPublishSnapshot;
using app::SharedSensorState;
using app::init_named_bin_logger;
using app::init_nav_state_publisher;
using app::service_dvl_device;
using app::service_imu_device;
using app::write_timing_trace;


// ========== 简单时间工具 ==========

inline MonoTimeNs monotonic_now_ns()
{
    using clock = std::chrono::steady_clock;
    auto now    = clock::now().time_since_epoch();
    return static_cast<MonoTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()
    );
}

enum class SensorRejectKind : std::uint8_t {
    kNone = 0,
    kPreprocessRejected,
    kStale,
    kOutOfOrder,
    kGatedRejected,
};

const char* sensor_reject_name(SensorRejectKind kind) noexcept
{
    switch (kind) {
    case SensorRejectKind::kPreprocessRejected:
        return "preprocess_rejected";
    case SensorRejectKind::kStale:
        return "stale_sample";
    case SensorRejectKind::kOutOfOrder:
        return "out_of_order";
    case SensorRejectKind::kGatedRejected:
        return "gated_rejected";
    case SensorRejectKind::kNone:
    default:
        return "none";
    }
}

smsg::NavFaultCode sensor_reject_fault_code(const char* sensor_id, SensorRejectKind kind) noexcept
{
    const bool imu = (sensor_id != nullptr && std::strcmp(sensor_id, "imu") == 0);
    switch (kind) {
    case SensorRejectKind::kStale:
    case SensorRejectKind::kOutOfOrder:
        return imu ? smsg::NavFaultCode::kImuStale : smsg::NavFaultCode::kNoData;
    case SensorRejectKind::kPreprocessRejected:
    case SensorRejectKind::kGatedRejected:
    case SensorRejectKind::kNone:
    default:
        return smsg::NavFaultCode::kNone;
    }
}

struct RejectEventTracker {
    SensorRejectKind last_kind{SensorRejectKind::kNone};

    bool should_log(SensorRejectKind kind)
    {
        if (kind == SensorRejectKind::kNone) {
            last_kind = SensorRejectKind::kNone;
            return false;
        }
        if (kind == last_kind) {
            return false;
        }
        last_kind = kind;
        return true;
    }

    void clear() noexcept { last_kind = SensorRejectKind::kNone; }
};

// ========== IMU 帧 → 样本 转换工具 ==========

inline void imu_frame_to_sample_raw(const ImuFrame& in, ImuSample& out)
{
    // 时间戳直接拷贝：主线程只允许在消费路径补 consume_mono_ns。
    out.sensor_time_ns = in.sensor_time_ns;
    out.recv_mono_ns = in.recv_mono_ns;
    out.consume_mono_ns = in.consume_mono_ns;
    out.mono_ns = in.mono_ns;
    out.est_ns  = in.est_ns;

    // 角速度 / 加速度 / 欧拉角
    for (int i = 0; i < 3; ++i) {
        out.ang_vel[i] = in.ang_vel[i];
        out.lin_acc[i] = in.lin_acc[i];
        out.euler[i]   = in.euler[i];
    }

    // 温度 / 有效性 / 状态
    out.temperature = in.temperature;
    out.valid       = in.valid;
    out.status      = in.status;
}

void reset_eskf_to_config_defaults(estimator::EskfFilter& eskf)
{
    const auto cfg = eskf.config();
    estimator::EskfNominalState x0{};
    x0.p_enu = nav_core::Vec3d{cfg.init_E_m, cfg.init_N_m, cfg.init_U_m};
    x0.v_enu = nav_core::Vec3d{cfg.init_vE_mps, cfg.init_vN_mps, cfg.init_vU_mps};
    x0.rpy_nb_rad = nav_core::Vec3d{0.0, 0.0, cfg.init_yaw_rad};
    x0.ba_mps2 = nav_core::Vec3d{0.0, 0.0, 0.0};
    x0.bg_rad_s = nav_core::Vec3d{0.0, 0.0, cfg.init_bgz_rad_s};
    eskf.reset(x0);
}

// 将实时预处理结果 DvlRtOutput 映射为 ESKF 所需的 DvlSample（= DvlFrame）
inline void dvl_rt_output_to_sample(const preprocess::DvlRtOutput& in,
                                    DvlSample&                    out)
{
    out.sensor_time_ns = in.sensor_time_ns;
    out.recv_mono_ns = in.recv_mono_ns;
    out.consume_mono_ns = in.consume_mono_ns;
    out.mono_ns = in.mono_ns;
    out.est_ns  = in.est_ns;

    // ---- 体坐标系速度（BI → body=FRD）----
    if (in.has_bi) {
        out.vel_body_mps[0] = static_cast<float>(in.vel_bi_body.x);
        out.vel_body_mps[1] = static_cast<float>(in.vel_bi_body.y);
        out.vel_body_mps[2] = static_cast<float>(in.vel_bi_body.z);
        out.has_body_vel    = true;
    } else {
        out.vel_body_mps[0] = 0.f;
        out.vel_body_mps[1] = 0.f;
        out.vel_body_mps[2] = 0.f;
        out.has_body_vel    = false;
    }

    // ---- ENU 速度（BE → ENU）----
    if (in.has_be) {
        out.vel_enu_mps[0] = static_cast<float>(in.vel_be_enu.x);
        out.vel_enu_mps[1] = static_cast<float>(in.vel_be_enu.y);
        out.vel_enu_mps[2] = static_cast<float>(in.vel_be_enu.z);
        out.has_enu_vel    = true;
    } else {
        out.vel_enu_mps[0] = 0.f;
        out.vel_enu_mps[1] = 0.f;
        out.vel_enu_mps[2] = 0.f;
        out.has_enu_vel    = false;
    }

    // ---- ENU 积分位移：当前我们不用，保持默认 ----
    out.dist_enu_m[0] = 0.f;
    out.dist_enu_m[1] = 0.f;
    out.dist_enu_m[2] = 0.f;
    out.has_dist_enu  = false;

    // ---- 高度 ----
    if (in.has_alt) {
        out.altitude_m   = static_cast<float>(in.alt_bottom_m);
        out.has_altitude = true;
    } else {
        out.altitude_m   = 0.f;
        out.has_altitude = false;
    }

    // ---- 质量 / 标志 ----
    out.bottom_lock = in.bottom_lock;
    out.valid       = in.gated_ok;

    out.track_type = nav_core::DvlTrackType::BottomTrack;   // 目前统一先当作底跟踪

    out.fom     = in.mean_corr;
    out.quality = 0;
}

inline SampleTiming sample_timing_from(const ImuFrame& sample) noexcept
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

// ========== 主循环：所有细节通过已有模块完成 ==========

int run_main_loop(const app::NavDaemonConfig&    cfg,
                  SharedSensorState&             shared_state,
                  drivers::ImuDriverWit&         imu_driver,
                  drivers::DvlDriver&            dvl_driver,
                  preprocess::ImuRtPreprocessor& imu_pp,
                  preprocess::DvlRtPreprocessor& dvl_pp,
                  estimator::EskfFilter&         eskf,
                  io::NavStatePublisher&         nav_pub,
                  nav_core::BinLogger*           sample_logger,
                  nav_core::BinLogger*           nav_state_logger,
                  nav_core::BinLogger*           timing_logger,
                  NavEventCsvLogger*             event_logger,
                  std::atomic<bool>&             stop_flag)
{
    const double loop_hz   = (cfg.loop.nav_loop_hz > 0.0) ? cfg.loop.nav_loop_hz : 20.0;
    const double loop_dt_s = 1.0 / loop_hz;

    const double max_imu_age_s = cfg.loop.max_imu_age_s;
    const double max_dvl_age_s = cfg.loop.max_dvl_age_s;

    std::fprintf(stderr, "[nav_daemon] main loop started (%.2f Hz)\n", loop_hz);

    std::fprintf(stderr,
                 "[nav_daemon] READY: online navigation is running "
                 "(IMU=%s, DVL=%s, logging=%s)\n",
                 cfg.imu.enable ? "ENABLED" : "DISABLED",
                 cfg.dvl.enable ? "ENABLED" : "DISABLED",
                 (sample_logger || timing_logger) ? "ON" : "OFF");

    auto          next_wakeup    = std::chrono::steady_clock::now();
    std::uint64_t print_counter  = 0;
    const double  print_interval_s = 10.0;
    MonoTimeNs    last_print_ns    = 0;
    MonoTimeNs    last_used_imu_ns     = 0;
    MonoTimeNs    last_used_dvl_ns     = 0;
    std::optional<ImuSample> last_consumed_nav_imu;
    SampleTiming            last_consumed_imu_timing{};
    SampleTiming            last_consumed_dvl_timing{};
    ManagedDeviceRuntime    imu_runtime{app::DeviceBinder("imu",
                                                       cfg.imu.driver.port,
                                                       cfg.imu.driver.binding,
                                                       {},
                                                       app::make_imu_port_selector(
                                                           cfg.imu.driver))};
    ManagedDeviceRuntime    dvl_runtime{
        app::DeviceBinder("dvl", cfg.dvl.driver.port, cfg.dvl.driver.binding)};
    RejectEventTracker      imu_reject_tracker{};
    RejectEventTracker      dvl_reject_tracker{};
    NavPublishSnapshot      last_nav_publish_snapshot{};
    bool                    have_last_nav_publish_snapshot = false;

#if NAV_CORE_ENABLE_GRAPH
    estimator::NavHealthMonitor health_monitor(cfg.estimator.health);
    const bool health_monitor_enabled = cfg.estimator.enable_health;
#endif

    while (!stop_flag.load(std::memory_order_relaxed)) {
        next_wakeup += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(loop_dt_s));

        const MonoTimeNs now_mono_ns = monotonic_now_ns();

        if (cfg.imu.enable &&
            service_imu_device(cfg, shared_state, imu_driver, imu_runtime, timing_logger,
                               event_logger, now_mono_ns)) {
            imu_pp.reset();
            reset_eskf_to_config_defaults(eskf);
            last_consumed_nav_imu.reset();
            last_consumed_imu_timing = SampleTiming{};
            last_used_imu_ns = 0;
            last_consumed_dvl_timing = SampleTiming{};
            last_used_dvl_ns = 0;
            std::fprintf(stderr,
                         "[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline\n");
        }

        if (cfg.dvl.enable &&
            service_dvl_device(cfg, shared_state, dvl_driver, dvl_runtime, timing_logger,
                               event_logger, now_mono_ns)) {
            dvl_pp.reset();
            last_consumed_dvl_timing = SampleTiming{};
            last_used_dvl_ns = 0;
            std::fprintf(stderr,
                         "[nav_daemon] DVL connectivity changed; clear DVL freshness cache\n");
        }

        std::optional<ImuFrame>                 imu_frame;
        std::optional<preprocess::DvlRawSample> dvl_raw;
        std::optional<ImuSample>                latest_nav_imu;
        MonoTimeNs last_imu_ns = 0;
        MonoTimeNs last_dvl_ns = 0;

        {
            std::lock_guard<std::mutex> lock(shared_state.m);
            imu_frame   = shared_state.last_imu;
            dvl_raw     = shared_state.last_dvl_raw;
            last_imu_ns = shared_state.last_imu_mono_ns;
            last_dvl_ns = shared_state.last_dvl_mono_ns;
            ++shared_state.loop_counter;
        }

#if !NAV_CORE_ENABLE_GRAPH
        (void)last_imu_ns;
        (void)last_dvl_ns;
#endif

        double imu_age_s = 0.0;
        double dvl_age_s = 0.0;

        // ---------- IMU 管线：只消费“新且 fresh”的样本，禁止重复传播旧快照 ----------
        if (imu_frame.has_value()) {
            const ImuFrame& frame_in = *imu_frame;
            if (frame_in.mono_ns > 0 && now_mono_ns >= frame_in.mono_ns) {
                imu_age_s = static_cast<double>(now_mono_ns - frame_in.mono_ns) * 1e-9;
            }

            if (app::is_sample_fresh(now_mono_ns, frame_in.mono_ns, max_imu_age_s) &&
                app::should_consume_sample(frame_in.mono_ns, last_used_imu_ns)) {
                ImuSample samp_in{};
                imu_frame_to_sample_raw(frame_in, samp_in);
                samp_in.consume_mono_ns = now_mono_ns;
                samp_in.est_ns = samp_in.mono_ns;

                ImuSample samp_out{};
                const bool ok = imu_pp.process(samp_in, samp_out);
                if (!ok) {
                    const auto sample_age_ms = app::compute_age_ms(now_mono_ns, samp_in.mono_ns);
                    write_timing_trace(
                        timing_logger,
                        io::TimingTraceKind::kImuRejected,
                        sample_timing_from(samp_in),
                        0,
                        sample_age_ms,
                        io::kTimingTraceRejected);
                    if (event_logger != nullptr &&
                        imu_reject_tracker.should_log(SensorRejectKind::kPreprocessRejected)) {
                        // 这里只在拒绝类别变化时落一条事件，避免主循环因为同一类坏样本连续刷文本。
                        event_logger->log_sensor_update_rejected(
                            now_mono_ns,
                            "imu",
                            sensor_reject_name(SensorRejectKind::kPreprocessRejected),
                            sample_age_ms,
                            static_cast<std::uint16_t>(
                                sensor_reject_fault_code("imu", SensorRejectKind::kPreprocessRejected)));
                    }
                    if ((print_counter % 50) == 0) {
                        std::fprintf(stderr,
                                     "[nav_daemon] WARNING: ImuRtPreprocessor rejected a sample "
                                     "(可能是 NaN / 无效欧拉角 / 静止窗尚未建立 bias)\n");
                    }
                } else {
                    samp_out.consume_mono_ns = samp_in.consume_mono_ns;
                    samp_out.est_ns = samp_out.mono_ns;
                    latest_nav_imu = samp_out;
                    last_consumed_nav_imu = samp_out;
                    last_consumed_imu_timing = sample_timing_from(samp_out);
                    last_used_imu_ns = samp_out.mono_ns;
                    const bool imu_used = eskf.propagate_imu(samp_out);
#if !NAV_CORE_ENABLE_GRAPH
                    (void)imu_used;
#endif
#if NAV_CORE_ENABLE_GRAPH
                    if (health_monitor_enabled && imu_used) {
                        health_monitor.notify_imu_propagate(samp_out.mono_ns);
                    }
#endif
                    write_timing_trace(
                        timing_logger,
                        io::TimingTraceKind::kImuConsumed,
                        last_consumed_imu_timing,
                        0,
                        app::compute_age_ms(now_mono_ns, samp_out.mono_ns),
                        io::kTimingTraceFresh | io::kTimingTraceAccepted);
                    imu_reject_tracker.clear();
                }
                last_used_imu_ns = frame_in.mono_ns;
            } else if (!app::is_sample_fresh(now_mono_ns, frame_in.mono_ns, max_imu_age_s)) {
                const auto sample_age_ms = app::compute_age_ms(now_mono_ns, frame_in.mono_ns);
                if (event_logger != nullptr &&
                    imu_reject_tracker.should_log(SensorRejectKind::kStale)) {
                    event_logger->log_sensor_update_rejected(
                        now_mono_ns,
                        "imu",
                        sensor_reject_name(SensorRejectKind::kStale),
                        sample_age_ms,
                        static_cast<std::uint16_t>(
                            sensor_reject_fault_code("imu", SensorRejectKind::kStale)));
                }
                if ((print_counter % 200) == 0) {
                    write_timing_trace(
                        timing_logger,
                        io::TimingTraceKind::kImuRejected,
                        sample_timing_from(frame_in),
                        0,
                        sample_age_ms,
                        io::kTimingTraceRejected | io::kTimingTraceStale);
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: IMU sample age=%.3f > max_imu_age_s=%.3f, "
                                 "skip stale sample\n",
                                 imu_age_s, max_imu_age_s);
                }
            } else if (!app::should_consume_sample(frame_in.mono_ns, last_used_imu_ns) &&
                       frame_in.mono_ns < last_used_imu_ns) {
                const auto sample_age_ms = app::compute_age_ms(now_mono_ns, frame_in.mono_ns);
                if (event_logger != nullptr &&
                    imu_reject_tracker.should_log(SensorRejectKind::kOutOfOrder)) {
                    event_logger->log_sensor_update_rejected(
                        now_mono_ns,
                        "imu",
                        sensor_reject_name(SensorRejectKind::kOutOfOrder),
                        sample_age_ms,
                        static_cast<std::uint16_t>(
                            sensor_reject_fault_code("imu", SensorRejectKind::kOutOfOrder)));
                }
                if ((print_counter % 200) == 0) {
                    write_timing_trace(
                        timing_logger,
                        io::TimingTraceKind::kImuRejected,
                        sample_timing_from(frame_in),
                        0,
                        sample_age_ms,
                        io::kTimingTraceRejected | io::kTimingTraceOutOfOrder);
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: IMU sample out-of-order "
                                 "(sample=%lld last_used=%lld), drop\n",
                                 static_cast<long long>(frame_in.mono_ns),
                                 static_cast<long long>(last_used_imu_ns));
                }
            }
        }

        if (!latest_nav_imu.has_value() &&
            last_consumed_nav_imu.has_value() &&
            app::is_sample_fresh(now_mono_ns, last_consumed_imu_timing, max_imu_age_s)) {
            latest_nav_imu = last_consumed_nav_imu;
        }

        // ---------- DVL 管线：只消费“新且 fresh”的样本，禁止乱序重复更新 ----------
        if (dvl_raw.has_value()) {
            if (dvl_raw->mono_ns > 0 && now_mono_ns >= dvl_raw->mono_ns) {
                dvl_age_s = static_cast<double>(now_mono_ns - dvl_raw->mono_ns) * 1e-9;
            }

            if (app::is_sample_fresh(now_mono_ns, dvl_raw->mono_ns, max_dvl_age_s)) {
                preprocess::DvlRawSample raw_in = *dvl_raw;

                if (app::should_consume_sample(raw_in.mono_ns, last_used_dvl_ns)) {
                    raw_in.consume_mono_ns = now_mono_ns;
                    preprocess::DvlRtOutput dvl_out{};
                    const bool ok = dvl_pp.process(raw_in, dvl_out);

                    if (ok && dvl_out.gated_ok) {
                        DvlSample dvl_sample{};
                        dvl_rt_output_to_sample(dvl_out, dvl_sample);

                        last_consumed_dvl_timing = sample_timing_from(dvl_sample);

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
                        if (health_monitor_enabled) {
                            if (dvl_sample.has_enu_vel) {
                                health_monitor.notify_dvl_xy_update(
                                    dvl_sample.mono_ns, diag_xy.nis, diag_xy.ok);
                            }
                            if (eskf.config().enable_dvl_z_update &&
                                (dvl_sample.has_enu_vel || dvl_sample.has_altitude)) {
                                health_monitor.notify_z_update(
                                    dvl_sample.mono_ns, diag_z.nis, diag_z.ok);
                            }
                        }
#endif

                        if (sample_logger) {
                            sample_logger->write(&dvl_sample, sizeof(dvl_sample));
                        }

                        write_timing_trace(
                            timing_logger,
                            io::TimingTraceKind::kDvlConsumed,
                            last_consumed_dvl_timing,
                            0,
                            app::compute_age_ms(now_mono_ns, dvl_sample.mono_ns),
                            io::kTimingTraceFresh | io::kTimingTraceAccepted);
                        dvl_reject_tracker.clear();
                    } else {
                        const auto sample_age_ms = app::compute_age_ms(now_mono_ns, raw_in.mono_ns);
                        const auto reject_kind = ok
                            ? SensorRejectKind::kGatedRejected
                            : SensorRejectKind::kPreprocessRejected;
                        write_timing_trace(
                            timing_logger,
                            io::TimingTraceKind::kDvlRejected,
                            sample_timing_from(raw_in),
                            0,
                            sample_age_ms,
                            io::kTimingTraceRejected);
                        if (event_logger != nullptr && dvl_reject_tracker.should_log(reject_kind)) {
                            event_logger->log_sensor_update_rejected(
                                now_mono_ns,
                                "dvl",
                                sensor_reject_name(reject_kind),
                                sample_age_ms,
                                static_cast<std::uint16_t>(
                                    sensor_reject_fault_code("dvl", reject_kind)));
                        }
                    }

                    last_used_dvl_ns = raw_in.mono_ns;
                } else if (raw_in.mono_ns < last_used_dvl_ns) {
                    const auto sample_age_ms = app::compute_age_ms(now_mono_ns, raw_in.mono_ns);
                    if (event_logger != nullptr &&
                        dvl_reject_tracker.should_log(SensorRejectKind::kOutOfOrder)) {
                        event_logger->log_sensor_update_rejected(
                            now_mono_ns,
                            "dvl",
                            sensor_reject_name(SensorRejectKind::kOutOfOrder),
                            sample_age_ms,
                            static_cast<std::uint16_t>(
                                sensor_reject_fault_code("dvl", SensorRejectKind::kOutOfOrder)));
                    }
                    if ((print_counter % 200) == 0) {
                        write_timing_trace(
                            timing_logger,
                            io::TimingTraceKind::kDvlRejected,
                            sample_timing_from(raw_in),
                            0,
                            sample_age_ms,
                            io::kTimingTraceRejected | io::kTimingTraceOutOfOrder);
                        std::fprintf(stderr,
                                     "[nav_daemon] WARNING: DVL sample out-of-order "
                                     "(sample=%lld last_used=%lld), drop\n",
                                     static_cast<long long>(raw_in.mono_ns),
                                     static_cast<long long>(last_used_dvl_ns));
                    }
                }
            } else {
                const auto sample_age_ms = app::compute_age_ms(now_mono_ns, dvl_raw->mono_ns);
                if (event_logger != nullptr &&
                    dvl_reject_tracker.should_log(SensorRejectKind::kStale)) {
                    event_logger->log_sensor_update_rejected(
                        now_mono_ns,
                        "dvl",
                        sensor_reject_name(SensorRejectKind::kStale),
                        sample_age_ms,
                        static_cast<std::uint16_t>(
                            sensor_reject_fault_code("dvl", SensorRejectKind::kStale)));
                }
                if ((print_counter % 200) == 0) {
                    write_timing_trace(
                        timing_logger,
                        io::TimingTraceKind::kDvlRejected,
                        sample_timing_from(*dvl_raw),
                        0,
                        sample_age_ms,
                        io::kTimingTraceRejected | io::kTimingTraceStale);
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: DVL age=%.3f > max_dvl_age_s=%.3f, "
                                 "treat as offline\n",
                                 dvl_age_s, max_dvl_age_s);
                }
            }
        }

        // ---------- ESKF → NavState（唯一出口） ----------
        smsg::NavState nav{};
        const MonoTimeNs state_stamp_ns = eskf.last_propagate_time();
        app::fill_nav_state_kinematics(
            eskf,
            nav,
            state_stamp_ns,
            latest_nav_imu.has_value() ? &(*latest_nav_imu) : nullptr);

        app::NavPublishContext publish_ctx{};
        publish_ctx.publish_mono_ns = now_mono_ns;
        publish_ctx.state_stamp_ns = state_stamp_ns;
        publish_ctx.imu_timing = last_consumed_imu_timing;
        publish_ctx.dvl_timing = last_consumed_dvl_timing;
        publish_ctx.max_imu_age_s = max_imu_age_s;
        publish_ctx.max_dvl_age_s = max_dvl_age_s;
        publish_ctx.imu_enabled = cfg.imu.enable;
        publish_ctx.dvl_enabled = cfg.dvl.enable;
        publish_ctx.imu_bias_ready = imu_pp.biasReady();
        publish_ctx.imu_device_state = cfg.imu.enable
            ? imu_runtime.binder.status().state
            : app::DeviceConnectionState::DISCONNECTED;
        publish_ctx.dvl_device_state = cfg.dvl.enable
            ? dvl_runtime.binder.status().state
            : app::DeviceConnectionState::DISCONNECTED;
        app::apply_nav_publish_semantics(
            publish_ctx, app::eskf_state_is_finite(eskf), nav);

#if NAV_CORE_ENABLE_GRAPH
        if (health_monitor_enabled) {
            health_monitor.update_sensor_heartbeat(now_mono_ns, last_imu_ns, last_dvl_ns);
            health_monitor.update_online_state(nav);
        }
#endif

        // ---------- 发布 NavState ----------
        if (nav_pub.ok()) {
            if (!nav_pub.publish(nav) && (print_counter % 100) == 0) {
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
            SampleTiming{state_stamp_ns, last_consumed_imu_timing.recv_mono_ns, now_mono_ns},
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
            (!have_last_nav_publish_snapshot || !(nav_snapshot == last_nav_publish_snapshot))) {
            // 这里只在 valid/stale/degraded/fault/status_flags 组合变化时写事件，
            // 让现场先看到“状态为什么切了”，而不是每个 publish 周期都重复一遍。
            event_logger->log_nav_publish_state_changed(now_mono_ns, nav);
            last_nav_publish_snapshot = nav_snapshot;
            have_last_nav_publish_snapshot = true;
        }

        // ---------- 每 10 秒输出一次当前导航结果 ----------
        {
            const double since_last_print_s =
                (last_print_ns > 0)
                    ? (now_mono_ns - last_print_ns) * 1e-9
                    : (print_interval_s + 1.0);  // 第一次循环强制打印

            if (since_last_print_s >= print_interval_s) {
                last_print_ns = now_mono_ns;

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
        }

        ++print_counter;
        std::this_thread::sleep_until(next_wakeup);
    }

    std::fprintf(stderr, "[nav_daemon] main loop exiting (stop requested)\n");

    imu_driver.stop();
    dvl_driver.stop();

    return 0;
}

} // anonymous namespace

// ========== 对外统一入口：给 main 调用 ==========

namespace nav_core::app {

int run_nav_daemon(const NavDaemonConfig&       cfg,
                   const estimator::EskfConfig& eskf_cfg,
                   std::atomic<bool>&           stop_flag)
{
    SharedSensorState shared_state;

    drivers::ImuDriverWit imu_driver;
    drivers::DvlDriver    dvl_driver;

    // ==== 实时预处理器 + ESKF ====
    preprocess::ImuRtPreprocessor imu_pp(cfg.imu.rt_preproc);
    preprocess::DvlRtPreprocessor dvl_pp(cfg.dvl.rt_preproc);
    estimator::EskfFilter         eskf(eskf_cfg);

    // NavStatePublisher
    io::NavStatePublisher nav_pub;
    bool pub_ok = init_nav_state_publisher(cfg, nav_pub);
    if (!pub_ok) {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: NavStatePublisher init failed or disabled "
                     "(控制侧将无法从共享内存读取导航状态)\n");
    }

    // BinLogger
    nav_core::BinLogger  sample_logger;
    nav_core::BinLogger* sample_logger_ptr = nullptr;
    if (init_named_bin_logger(cfg, "nav.bin", sample_logger)) {
        sample_logger_ptr = &sample_logger;
    } else {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: sample BinLogger init failed or disabled "
                     "(不会写处理后的 DVL 样本日志，但不影响导航本身)\n");
    }

    nav_core::BinLogger  timing_logger;
    nav_core::BinLogger* timing_logger_ptr = nullptr;
    if (init_named_bin_logger(cfg, "nav_timing.bin", timing_logger)) {
        timing_logger_ptr = &timing_logger;
    } else {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: timing BinLogger init failed or disabled "
                     "(不会写时间语义追踪日志，但不影响导航本身)\n");
    }

    nav_core::BinLogger  nav_state_logger;
    nav_core::BinLogger* nav_state_logger_ptr = nullptr;
    if (init_named_bin_logger(cfg, "nav_state.bin", nav_state_logger)) {
        nav_state_logger_ptr = &nav_state_logger;
    } else {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: nav_state BinLogger init failed or disabled "
                     "(不会写导航状态日志，但不影响导航本身)\n");
    }

    NavEventCsvLogger  nav_event_logger;
    NavEventCsvLogger* nav_event_logger_ptr = nullptr;
    if (nav_event_logger.init(cfg)) {
        nav_event_logger_ptr = &nav_event_logger;
    } else {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: nav_events.csv init failed or disabled "
                     "(结构化低频事件日志不可用，但不影响导航主链)\n");
    }

    std::fprintf(stderr,
                 "[nav_daemon] READY: device binders armed, ESKF online, loop=%.2f Hz\n",
                 cfg.loop.nav_loop_hz > 0.0 ? cfg.loop.nav_loop_hz : 20.0);

    // 主循环
    return run_main_loop(cfg, shared_state,
                         imu_driver, dvl_driver,
                         imu_pp, dvl_pp,
                         eskf, nav_pub,
                         sample_logger_ptr,
                         nav_state_logger_ptr,
                         timing_logger_ptr,
                         nav_event_logger_ptr,
                         stop_flag);
}

} // namespace nav_core::app
