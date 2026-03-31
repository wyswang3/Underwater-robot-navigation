// src/nav_core/app/nav_daemon_runner.cpp
//
// 角色：
//   - 作为 nav_daemon 的流程编排入口；
//   - 负责初始化运行时对象，并驱动主循环按固定顺序调用各个子模块；
//   - 尽量只保留“调度骨架”，不再内联 IMU/DVL 处理、发布和日志细节。

#include <atomic>
#include <chrono>
#include <cstdio>
#include <thread>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_devices.hpp"
#include "nav_core/app/nav_daemon_dvl_pipeline.hpp"
#include "nav_core/app/nav_daemon_health_audit.hpp"
#include "nav_core/app/nav_daemon_imu_pipeline.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/nav_daemon_loop_state.hpp"
#include "nav_core/app/nav_daemon_publish.hpp"

#include "nav_core/drivers/dvl_driver.hpp"
#include "nav_core/drivers/imu_driver_wit.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/io/nav_state_publisher.hpp"
#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"
#include "nav_core/preprocess/imu_rt_preprocessor.hpp"
#include "nav_core/estimator/nav_health_monitor.hpp"

namespace {

namespace app = nav_core::app;
namespace drivers = nav_core::drivers;
namespace estimator = nav_core::estimator;
namespace io = nav_core::io;
namespace preprocess = nav_core::preprocess;

int run_main_loop(const app::NavDaemonConfig&    cfg,
                  app::SharedSensorState&        shared_state,
                  drivers::ImuDriverWit&         imu_driver,
                  drivers::DvlDriver&            dvl_driver,
                  preprocess::ImuRtPreprocessor& imu_pp,
                  preprocess::DvlRtPreprocessor& dvl_pp,
                  estimator::EskfFilter&         eskf,
                  io::NavStatePublisher&         nav_pub,
                  nav_core::BinLogger*           sample_logger,
                  nav_core::BinLogger*           nav_state_logger,
                  nav_core::BinLogger*           timing_logger,
                  app::NavEventCsvLogger*        event_logger,
                  std::atomic<bool>&             stop_flag)
{
    const double loop_hz = (cfg.loop.nav_loop_hz > 0.0) ? cfg.loop.nav_loop_hz : 20.0;
    const double loop_dt_s = 1.0 / loop_hz;
    const double print_interval_s = 10.0;

    std::fprintf(stderr, "[nav_daemon] main loop started (%.2f Hz)\n", loop_hz);
    std::fprintf(stderr,
                 "[nav_daemon] READY: online navigation is running "
                 "(IMU=%s, DVL=%s, logging=%s)\n",
                 cfg.imu.enable ? "ENABLED" : "DISABLED",
                 cfg.dvl.enable ? "ENABLED" : "DISABLED",
                 (sample_logger || timing_logger) ? "ON" : "OFF");

    auto next_wakeup = std::chrono::steady_clock::now();
    app::NavLoopState loop_state = app::make_nav_loop_state(cfg);

    estimator::NavHealthMonitor health_monitor(cfg.estimator.health);
    const bool health_monitor_enabled = cfg.estimator.enable_health;
    if (health_monitor_enabled) {
        health_monitor.notify_eskf_reset(app::loop_monotonic_now_ns());
    }

    while (!stop_flag.load(std::memory_order_relaxed)) {
        next_wakeup += std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(loop_dt_s));

        const nav_core::MonoTimeNs now_mono_ns = app::loop_monotonic_now_ns();

        if (cfg.imu.enable &&
            app::service_imu_device(cfg,
                                    shared_state,
                                    imu_driver,
                                    loop_state.imu_runtime,
                                    timing_logger,
                                    event_logger,
                                    now_mono_ns)) {
            app::handle_imu_connectivity_change(loop_state, imu_pp, eskf);
            if (health_monitor_enabled) {
                health_monitor.notify_eskf_reset(now_mono_ns);
            }
        }

        if (cfg.dvl.enable &&
            app::service_dvl_device(cfg,
                                    shared_state,
                                    dvl_driver,
                                    loop_state.dvl_runtime,
                                    timing_logger,
                                    event_logger,
                                    now_mono_ns)) {
            app::handle_dvl_connectivity_change(loop_state, dvl_pp);
        }

        const app::SharedSensorSnapshot snapshot =
            app::snapshot_shared_sensor_state(shared_state);
        const auto latest_nav_imu =
            app::process_imu_pipeline(
                cfg,
                snapshot,
                now_mono_ns,
                imu_pp,
                eskf,
                timing_logger,
                event_logger,
                loop_state,
                &health_monitor,
                health_monitor_enabled
            );

        app::process_dvl_pipeline(
            cfg,
            snapshot,
            now_mono_ns,
            dvl_pp,
            eskf,
            sample_logger,
            timing_logger,
            event_logger,
            loop_state,
            &health_monitor,
            health_monitor_enabled
        );

        auto nav = app::build_nav_state_output(
            cfg,
            latest_nav_imu,
            now_mono_ns,
            loop_state,
            imu_pp,
            eskf
        );
        const auto health_report = app::evaluate_nav_health_audit(
            now_mono_ns,
            snapshot.last_imu_ns,
            snapshot.last_dvl_ns,
            &health_monitor,
            health_monitor_enabled,
            nav);

        app::publish_nav_state_output(
            nav,
            now_mono_ns,
            loop_state,
            nav_pub,
            nav_state_logger,
            timing_logger,
            event_logger);
        app::publish_nav_health_audit(
            cfg,
            now_mono_ns,
            health_report,
            nav,
            loop_state,
            event_logger);

        app::maybe_print_nav_summary(nav, now_mono_ns, loop_state, print_interval_s);

        ++loop_state.print_counter;
        std::this_thread::sleep_until(next_wakeup);
    }

    std::fprintf(stderr, "[nav_daemon] main loop exiting (stop requested)\n");
    imu_driver.stop();
    dvl_driver.stop();
    return 0;
}

} // namespace

namespace nav_core::app {

int run_nav_daemon(const NavDaemonConfig&       cfg,
                   const estimator::EskfConfig& eskf_cfg,
                   std::atomic<bool>&           stop_flag)
{
    SharedSensorState shared_state;

    drivers::ImuDriverWit imu_driver;
    drivers::DvlDriver    dvl_driver;

    preprocess::ImuRtPreprocessor imu_pp(cfg.imu.rt_preproc);
    preprocess::DvlRtPreprocessor dvl_pp(cfg.dvl.rt_preproc);
    estimator::EskfFilter         eskf(eskf_cfg);

    io::NavStatePublisher nav_pub;
    if (!init_nav_state_publisher(cfg, nav_pub)) {
        std::fprintf(stderr,
                     "[nav_daemon] WARNING: NavStatePublisher init failed or disabled "
                     "(控制侧将无法从共享内存读取导航状态)\n");
    }

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

    return run_main_loop(cfg,
                         shared_state,
                         imu_driver,
                         dvl_driver,
                         imu_pp,
                         dvl_pp,
                         eskf,
                         nav_pub,
                         sample_logger_ptr,
                         nav_state_logger_ptr,
                         timing_logger_ptr,
                         nav_event_logger_ptr,
                         stop_flag);
}

} // namespace nav_core::app
