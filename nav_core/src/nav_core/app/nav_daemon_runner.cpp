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
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <ctime>
#include <iostream>

#include "nav_core/app/nav_daemon_config.hpp"
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
using nav_core::SysTimeNs;
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


// ========== 简单时间工具 ==========

inline MonoTimeNs monotonic_now_ns()
{
    using clock = std::chrono::steady_clock;
    auto now    = clock::now().time_since_epoch();
    return static_cast<MonoTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()
    );
}

inline SysTimeNs system_now_ns()
{
    using clock = std::chrono::system_clock;
    auto now    = clock::now().time_since_epoch();
    return static_cast<SysTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count()
    );
}

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

// ========== 共享传感器状态（回调 → 主循环） ==========

struct SharedSensorState {
    std::mutex m;

    std::optional<ImuFrame>               last_imu;
    MonoTimeNs                            last_imu_mono_ns{0};

    // DVL：保存“原始采样”（DvlRawSample），让主循环去调用 DvlRtPreprocessor
    std::optional<preprocess::DvlRawSample> last_dvl_raw;
    MonoTimeNs                              last_dvl_mono_ns{0};

    std::uint64_t loop_counter{0};
};

// ========== 驱动初始化 ==========

bool init_imu(const app::NavDaemonConfig& cfg,
              SharedSensorState&          shared_state,
              drivers::ImuDriverWit&      imu_driver)
{
    if (!cfg.imu.enable) {
        std::fprintf(stderr, "[nav_daemon] IMU disabled by config\n");
        return true;
    }

    drivers::ImuConfig dcfg = cfg.imu.driver;

    auto on_frame = [&shared_state](const ImuFrame& frame) {
        MonoTimeNs mono_ns = frame.mono_ns;
        if (mono_ns <= 0) {
            mono_ns = monotonic_now_ns();
        }
        std::lock_guard<std::mutex> lock(shared_state.m);
        shared_state.last_imu         = frame;
        shared_state.last_imu_mono_ns = mono_ns;
    };

    drivers::ImuDriverWit::RawCallback on_raw;  // 不用，留空即可

    if (!imu_driver.init(dcfg, on_frame, on_raw)) {
        std::fprintf(stderr,
                     "[nav_daemon] ERROR: ImuDriverWit::init failed (port=%s, baud=%d)\n",
                     dcfg.port.c_str(), dcfg.baud);
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查 IMU 串口号 / 波特率 / 设备权限 / 设备是否插好\n");
        return false;
    }
    if (!imu_driver.start()) {
        std::fprintf(stderr, "[nav_daemon] ERROR: ImuDriverWit::start failed\n");
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查串口是否被其它进程占用，或驱动内部异常\n");
        return false;
    }

    std::fprintf(stderr,
                 "[nav_daemon] IMU driver started on %s @ %d\n",
                 dcfg.port.c_str(), dcfg.baud);
    return true;
}

bool init_dvl(const app::NavDaemonConfig& cfg,
              SharedSensorState&          shared,
              drivers::DvlDriver&         dvl)
{
    if (!cfg.dvl.enable) {
        std::fprintf(stderr, "[nav_daemon] DVL disabled by config\n");
        return true;
    }

    using drivers::DvlRawData;
    using preprocess::DvlRawSample;

    drivers::DvlConfig dcfg = cfg.dvl.driver;

    // ★ RawCallback：DvlRawData → DvlRawSample → SharedSensorState
    auto on_raw = [&shared](const DvlRawData& raw) {
        DvlRawSample sample{};

        sample.sensor_time_ns = raw.sensor_time_ns;
        sample.recv_mono_ns = raw.recv_mono_ns;
        sample.consume_mono_ns = 0;
        sample.mono_ns = raw.mono_ns;
        sample.est_ns  = raw.est_ns;

        const auto& p = raw.parsed;

        // ---- kind: 根据帧类型字符串做一个简单映射 ----
        std::uint8_t kind = 0;
        if (p.src == "BI") {
        kind = 0;
        } else if (p.src == "BE") {
        kind = 1;
        } else if (p.src == "BS") {
        kind = 2;
        } else if (p.src == "BD") {
        kind = 3;
        } else {
            // 将来要支持 WI/WE/WD 时再细化
            kind = 0;
        }
        sample.kind = kind;

        // ---- bottom_lock / 质量状态 ----
        sample.bottom_lock = p.bottom_lock;

        // 【如果 DvlRawSample 还有 range_m/corr/fom/status 等字段，这里可以继续补映射】

        // ---- 速度：根据 CoordFrame 决定填 Earth 还是 Instrument ----
        // 先清零 Vec3f（三个分量）
        sample.vel_inst_mps.x  = 0.f;
        sample.vel_inst_mps.y  = 0.f;
        sample.vel_inst_mps.z  = 0.f;
        sample.vel_earth_mps.x = 0.f;
        sample.vel_earth_mps.y = 0.f;
        sample.vel_earth_mps.z = 0.f;

        if (p.coord_frame == DvlCoordFrame::Earth) {
            // ENU 速度直接填 vel_earth_mps
            sample.vel_earth_mps.x = static_cast<float>(p.ve_mps());
            sample.vel_earth_mps.y = static_cast<float>(p.vn_mps());
            sample.vel_earth_mps.z = static_cast<float>(p.vu_mps());
        } else {
            // Instrument / Ship 帧走 vel_inst_mps，后面再用安装角/姿态旋转
            sample.vel_inst_mps.x = static_cast<float>(p.ve_mps());
            sample.vel_inst_mps.y = static_cast<float>(p.vn_mps());
            sample.vel_inst_mps.z = static_cast<float>(p.vu_mps());
        }

        // ---- 写入共享状态（供主循环使用）----
        {
            std::lock_guard<std::mutex> lock(shared.m);
            shared.last_dvl_raw     = sample;
            shared.last_dvl_mono_ns = sample.mono_ns;
        }
    };

    if (!dvl.init(dcfg, on_raw)) {
        std::fprintf(stderr,
                     "[nav_daemon] ERROR: DvlDriver::init failed (port=%s, baud=%d)\n",
                     dcfg.port.c_str(), dcfg.baud);
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查 DVL 串口 / 波特率 / 供电 / 厂家协议配置\n");
        return false;
    }

    if (!dvl.start()) {
        std::fprintf(stderr, "[nav_daemon] ERROR: DvlDriver::start failed\n");
        return false;
    }

    std::fprintf(stderr,
                 "[nav_daemon] DVL driver started on %s @ %d\n",
                 dcfg.port.c_str(), dcfg.baud);
    return true;
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

inline void write_timing_trace(nav_core::BinLogger*        logger,
                               io::TimingTraceKind         kind,
                               const SampleTiming&         timing,
                               MonoTimeNs                  publish_mono_ns,
                               std::uint32_t               age_ms,
                               std::uint16_t               flags,
                               shared::msg::NavFaultCode   fault_code =
                                   shared::msg::NavFaultCode::kNone)
{
    if (logger == nullptr) {
        return;
    }

    io::TimingTracePacketV1 pkt{};
    pkt.kind = static_cast<std::uint16_t>(kind);
    pkt.flags = flags;
    pkt.sensor_time_ns = timing.sensor_time_ns;
    pkt.recv_mono_ns = timing.recv_mono_ns;
    pkt.consume_mono_ns = timing.consume_mono_ns;
    pkt.publish_mono_ns = publish_mono_ns;
    pkt.age_ms = age_ms;
    pkt.fault_code = static_cast<std::uint32_t>(fault_code);
    logger->writePod(pkt);
}

// ========== NavStatePublisher 初始化 ==========

bool init_nav_state_publisher(const app::NavDaemonConfig& cfg,
                              io::NavStatePublisher&      pub)
{
    io::NavStatePublisherConfig shm_cfg = cfg.publisher.shm;
    if (!shm_cfg.enable) {
        std::fprintf(stderr, "[nav_daemon] NavStatePublisher disabled by config\n");
        return false;
    }
    if (!pub.init(shm_cfg)) {
        std::fprintf(stderr,
                     "[nav_daemon] ERROR: NavStatePublisher::init failed (shm=%s)\n",
                     shm_cfg.shm_name.c_str());
        std::fprintf(stderr,
                     "[nav_daemon] HINT: 请检查 shm_name 是否与控制侧一致、系统共享内存权限是否足够\n");
        return false;
    }
    std::fprintf(stderr,
                 "[nav_daemon] NavStatePublisher initialized (shm=%s)\n",
                 shm_cfg.shm_name.c_str());
    return true;
}

// ========== BinLogger 初始化（使用 nav_core::BinLogger 简单版本） ==========

bool init_named_bin_logger(const app::NavDaemonConfig& cfg,
                           const char*                 filename,
                           nav_core::BinLogger&        logger)
{
    const auto& lcfg = cfg.logging;
    if (!lcfg.enable || filename == nullptr || filename[0] == '\0') {
        std::fprintf(stderr,
                     "[nav_daemon] bin_logger disabled by config or invalid filename\n");
        return false;
    }

    namespace fs = std::filesystem;

    fs::path base(lcfg.base_dir);  // 例如 "/home/wys/data/nav"
    if (lcfg.split_by_date) {
        auto     now = std::chrono::system_clock::now();
        std::time_t tt = std::chrono::system_clock::to_time_t(now);
        std::tm  tm{};
    #if defined(_WIN32)
        localtime_s(&tm, &tt);
    #else
        localtime_r(&tt, &tm);
    #endif
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%04d-%02d-%02d",
                      tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
        base /= buf;
    }
    base /= "nav";

    fs::path file = base / filename;

    // BinLogger::open 自己会递归创建目录（ensureDirForFile），这里不强求 create_directories。
    if (!logger.open(file.string(), /*append=*/true)) {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: BinLogger::open('%s') failed, disable logging\n",
                     file.string().c_str());
        return false;
    }

    std::fprintf(stderr,
                 "[nav_daemon] bin_logger logging to %s\n",
                 file.string().c_str());
    return true;
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
                  nav_core::BinLogger*           timing_logger,
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

#if NAV_CORE_ENABLE_GRAPH
    estimator::NavHealthMonitor health_monitor(cfg.estimator.health);
    const bool health_monitor_enabled = cfg.estimator.enable_health;
#endif

    while (!stop_flag.load(std::memory_order_relaxed)) {
        next_wakeup += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(loop_dt_s));

        const MonoTimeNs now_mono_ns = monotonic_now_ns();

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
                }
                last_used_imu_ns = frame_in.mono_ns;
            } else if (!app::is_sample_fresh(now_mono_ns, frame_in.mono_ns, max_imu_age_s) &&
                       (print_counter % 200) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: IMU sample age=%.3f > max_imu_age_s=%.3f, "
                             "skip stale sample\n",
                             imu_age_s, max_imu_age_s);
            } else if (!app::should_consume_sample(frame_in.mono_ns, last_used_imu_ns) &&
                       frame_in.mono_ns < last_used_imu_ns &&
                       (print_counter % 200) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: IMU sample out-of-order "
                             "(sample=%lld last_used=%lld), drop\n",
                             static_cast<long long>(frame_in.mono_ns),
                             static_cast<long long>(last_used_imu_ns));
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
                    }

                    last_used_dvl_ns = raw_in.mono_ns;
                } else if (raw_in.mono_ns < last_used_dvl_ns && (print_counter % 200) == 0) {
                    std::fprintf(stderr,
                                 "[nav_daemon] WARNING: DVL sample out-of-order "
                                 "(sample=%lld last_used=%lld), drop\n",
                                 static_cast<long long>(raw_in.mono_ns),
                                 static_cast<long long>(last_used_dvl_ns));
                }
            } else if ((print_counter % 200) == 0) {
                std::fprintf(stderr,
                             "[nav_daemon] WARNING: DVL age=%.3f > max_dvl_age_s=%.3f, "
                             "treat as offline\n",
                             dvl_age_s, max_dvl_age_s);
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

    if (!init_imu(cfg, shared_state, imu_driver)) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: IMU init failed "
                     "(检查 nav_daemon.yaml.imu.driver.* 配置与硬件连接)\n");
        return 1;
    }
    if (!init_dvl(cfg, shared_state, dvl_driver)) {
        std::fprintf(stderr,
                     "[nav_daemon] FATAL: DVL init failed "
                     "(检查 nav_daemon.yaml.dvl.driver.* 配置与硬件连接)\n");
        imu_driver.stop();
        return 1;
    }

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

    std::fprintf(stderr,
                 "[nav_daemon] READY: IMU+DVL drivers started, ESKF online, loop=%.2f Hz\n",
                 cfg.loop.nav_loop_hz > 0.0 ? cfg.loop.nav_loop_hz : 20.0);

    // 主循环
    return run_main_loop(cfg, shared_state,
                         imu_driver, dvl_driver,
                         imu_pp, dvl_pp,
                         eskf, nav_pub,
                         sample_logger_ptr,
                         timing_logger_ptr,
                         stop_flag);
}

} // namespace nav_core::app
