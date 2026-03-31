// nav_core/include/nav_core/app/nav_daemon_dvl_pipeline.hpp
//
// @file  nav_daemon_dvl_pipeline.hpp
// @brief nav_daemon 的 DVL 独立处理模块。
//
// 角色：
//   - 从共享快照中提取 DVL 原始样本；
//   - 执行 freshness / out-of-order 判定和实时预处理；
//   - 把合法 DVL 观测用于 ESKF 的 XY/Z 更新；
//   - 维护 DVL 相关 timing trace、拒绝事件日志和样本日志。
//
// 边界：
//   - 本模块不处理 IMU；
//   - 不负责设备重连；
//   - 不直接发布 NavState，只更新 ESKF 与循环状态。
//
#pragma once

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/nav_daemon_loop_state.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"

#if NAV_CORE_ENABLE_GRAPH
#include "nav_core/estimator/nav_health_monitor.hpp"
#endif

namespace nav_core::app {

void process_dvl_pipeline(
    const NavDaemonConfig&         cfg,
    const SharedSensorSnapshot&    snapshot,
    MonoTimeNs                     now_mono_ns,
    preprocess::DvlRtPreprocessor& dvl_pp,
    estimator::EskfFilter&         eskf,
    nav_core::BinLogger*           sample_logger,
    nav_core::BinLogger*           timing_logger,
    NavEventCsvLogger*             event_logger,
    NavLoopState&                  loop_state
#if NAV_CORE_ENABLE_GRAPH
    , estimator::NavHealthMonitor* health_monitor = nullptr,
    bool                           health_monitor_enabled = false
#endif
);

} // namespace nav_core::app
