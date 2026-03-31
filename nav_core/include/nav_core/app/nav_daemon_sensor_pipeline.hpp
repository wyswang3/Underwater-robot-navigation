// nav_core/include/nav_core/app/nav_daemon_sensor_pipeline.hpp
//
// @file  nav_daemon_sensor_pipeline.hpp
// @brief nav_daemon 的 IMU/DVL 消费、预处理与 ESKF 更新管线。
//
// 角色：
//   - 消费 SharedSensorSnapshot 中的最新 IMU / DVL 数据；
//   - 执行 freshness / out-of-order 判定、实时预处理和 ESKF 更新；
//   - 统一写 timing trace、坏样本事件日志和 DVL 样本日志。
//
// 边界：
//   - 本模块只负责“样本如何进入 ESKF”；
//   - 不负责设备发现与掉线重连；
//   - 不负责 NavState 语义构建与发布。
//
#pragma once

#include <optional>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/nav_daemon_loop_state.hpp"
#include "nav_core/core/types.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"
#include "nav_core/preprocess/imu_rt_preprocessor.hpp"

#if NAV_CORE_ENABLE_GRAPH
#include "nav_core/estimator/nav_health_monitor.hpp"
#endif

namespace nav_core::app {

/**
 * @brief 一次主循环处理完传感器样本后的产物。
 *
 * 说明：
 *  - `latest_nav_imu` 是本周期可用于 NavState 输出的最新 IMU 样本；
 *  - `last_imu_ns/last_dvl_ns` 主要供健康监测和发布语义使用。
 */
struct ProcessedLoopSamples {
    std::optional<ImuSample> latest_nav_imu;
    MonoTimeNs               last_imu_ns{0};
    MonoTimeNs               last_dvl_ns{0};
};

ProcessedLoopSamples process_sensor_pipelines(
    const NavDaemonConfig&            cfg,
    const SharedSensorSnapshot&       snapshot,
    MonoTimeNs                        now_mono_ns,
    preprocess::ImuRtPreprocessor&    imu_pp,
    preprocess::DvlRtPreprocessor&    dvl_pp,
    estimator::EskfFilter&            eskf,
    nav_core::BinLogger*              sample_logger,
    nav_core::BinLogger*              timing_logger,
    NavEventCsvLogger*                event_logger,
    NavLoopState&                     loop_state
#if NAV_CORE_ENABLE_GRAPH
    , estimator::NavHealthMonitor*    health_monitor = nullptr,
    bool                              health_monitor_enabled = false
#endif
);

} // namespace nav_core::app
