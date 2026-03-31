// nav_core/include/nav_core/app/nav_daemon_publish.hpp
//
// @file  nav_daemon_publish.hpp
// @brief nav_daemon 的 NavState 构建、发布与周期性摘要打印接口。
//
// 角色：
//   - 根据 ESKF 状态、设备状态和最近样本 timing 构建最终 NavState；
//   - 负责共享内存发布、binlog 落盘、状态变化事件记录；
//   - 统一周期性导航摘要输出，避免这些逻辑散落在主循环中。
//
// 边界：
//   - 本模块不做传感器预处理和设备重连；
//   - 假定传感器消费结果已经由 nav_daemon_sensor_pipeline 给出。
//
#pragma once

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/nav_daemon_loop_state.hpp"
#include "nav_core/app/nav_daemon_sensor_pipeline.hpp"
#include "nav_core/app/nav_runtime_status.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/io/nav_state_publisher.hpp"
#include "nav_core/preprocess/imu_rt_preprocessor.hpp"

#if NAV_CORE_ENABLE_GRAPH
#include "nav_core/estimator/nav_health_monitor.hpp"
#endif

namespace nav_core::app {

shared::msg::NavState build_nav_state_output(
    const NavDaemonConfig&         cfg,
    const ProcessedLoopSamples&    processed_samples,
    MonoTimeNs                     now_mono_ns,
    const NavLoopState&            loop_state,
    preprocess::ImuRtPreprocessor& imu_pp,
    estimator::EskfFilter&         eskf
#if NAV_CORE_ENABLE_GRAPH
    , estimator::NavHealthMonitor* health_monitor = nullptr,
    bool                           health_monitor_enabled = false
#endif
);

void publish_nav_state_output(const shared::msg::NavState& nav,
                              MonoTimeNs now_mono_ns,
                              NavLoopState& loop_state,
                              io::NavStatePublisher& nav_pub,
                              nav_core::BinLogger* nav_state_logger,
                              nav_core::BinLogger* timing_logger,
                              NavEventCsvLogger* event_logger);

void maybe_print_nav_summary(const shared::msg::NavState& nav,
                             MonoTimeNs now_mono_ns,
                             NavLoopState& loop_state,
                             double print_interval_s);

} // namespace nav_core::app
