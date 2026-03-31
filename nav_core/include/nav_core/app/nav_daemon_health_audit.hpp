// nav_core/include/nav_core/app/nav_daemon_health_audit.hpp
//
// @file  nav_daemon_health_audit.hpp
// @brief nav_daemon 中 ESKF 健康审查桥接模块。
//
// 角色：
//   - 承接 estimator::NavHealthMonitor 的纯计算输出；
//   - 把根因分类与健康等级合并回最终 NavState 语义；
//   - 向操作员输出稳定、低频、可读的告警摘要，而不是让这些逻辑散落在 runner/publish 中。
//
// 边界：
//   - 本模块不读取串口、不处理 IMU/DVL 原始样本；
//   - 不直接修改 ESKF 状态，只做评估结果桥接与反馈；
//   - shared NavState ABI 保持不变，更细的根因信息仅走 stderr / nav_events.csv。
//
#pragma once

#include <optional>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/nav_daemon_loop_state.hpp"
#include "nav_core/core/types.hpp"
#include "nav_core/estimator/nav_health_monitor.hpp"

namespace nav_core::app {

std::optional<estimator::NavHealthReport> evaluate_nav_health_audit(
    MonoTimeNs                   now_mono_ns,
    MonoTimeNs                   last_imu_ns,
    MonoTimeNs                   last_dvl_ns,
    estimator::NavHealthMonitor* health_monitor,
    bool                         health_monitor_enabled,
    shared::msg::NavState&       nav);

void publish_nav_health_audit(
    const NavDaemonConfig&                             cfg,
    MonoTimeNs                                         now_mono_ns,
    const std::optional<estimator::NavHealthReport>&   report,
    const shared::msg::NavState&                       nav,
    NavLoopState&                                      loop_state,
    NavEventCsvLogger*                                 event_logger);

} // namespace nav_core::app
