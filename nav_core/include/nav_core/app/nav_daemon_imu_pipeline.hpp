// nav_core/include/nav_core/app/nav_daemon_imu_pipeline.hpp
//
// @file  nav_daemon_imu_pipeline.hpp
// @brief nav_daemon 的 IMU 独立处理模块。
//
// 角色：
//   - 从共享快照中提取 IMU 帧；
//   - 执行 freshness / out-of-order 判定和实时预处理；
//   - 把合法 IMU 样本推进 ESKF 传播；
//   - 维护 IMU 相关 timing trace 与拒绝事件日志。
//
// 边界：
//   - 本模块不处理 DVL；
//   - 不负责设备重连；
//   - 只输出“当前可用于导航发布的 IMU 样本”。
//
#pragma once

#include <optional>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/nav_daemon_loop_state.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "nav_core/estimator/nav_health_monitor.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/preprocess/imu_rt_preprocessor.hpp"

namespace nav_core::app {

std::optional<ImuSample> process_imu_pipeline(
    const NavDaemonConfig&         cfg,
    const SharedSensorSnapshot&    snapshot,
    MonoTimeNs                     now_mono_ns,
    preprocess::ImuRtPreprocessor& imu_pp,
    estimator::EskfFilter&         eskf,
    nav_core::BinLogger*           timing_logger,
    NavEventCsvLogger*             event_logger,
    NavLoopState&                  loop_state,
    estimator::NavHealthMonitor*   health_monitor = nullptr,
    bool                           health_monitor_enabled = false
);

} // namespace nav_core::app
