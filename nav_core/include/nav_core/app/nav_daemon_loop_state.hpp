// nav_core/include/nav_core/app/nav_daemon_loop_state.hpp
//
// @file  nav_daemon_loop_state.hpp
// @brief nav_daemon 主循环的持久状态与共享快照接口。
//
// 角色：
//   - 保存主循环跨 tick 复用的状态，例如最后消费时间、设备运行时状态、事件去重信息；
//   - 提供共享传感器缓存到主线程快照的统一入口；
//   - 把“主循环自身的状态”从 runner 中拆出来，避免主文件同时承载状态定义和流程编排。
//
// 边界：
//   - 本模块不做 IMU/DVL 预处理与 ESKF 更新；
//   - 不负责 NavState 发布，只提供主循环运行所需的状态容器和基础工具。
//
#pragma once

#include <cstdint>
#include <optional>

#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_devices.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/app/sample_timing.hpp"
#include "nav_core/core/types.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"
#include "nav_core/preprocess/imu_rt_preprocessor.hpp"

namespace nav_core::app {

enum class SensorRejectKind : std::uint8_t {
    kNone = 0,
    kPreprocessRejected,
    kStale,
    kOutOfOrder,
    kGatedRejected,
};

const char* sensor_reject_name(SensorRejectKind kind) noexcept;
shared::msg::NavFaultCode sensor_reject_fault_code(const char* sensor_id,
                                                   SensorRejectKind kind) noexcept;

struct RejectEventTracker {
    SensorRejectKind last_kind{SensorRejectKind::kNone};

    bool should_log(SensorRejectKind kind);
    void clear() noexcept;
};

/**
 * @brief 主循环从 SharedSensorState 抽取出的一次只读快照。
 *
 * 约定：
 *  - 驱动线程持续写 SharedSensorState；
 *  - 主线程每个循环只拿一份快照，之后的处理都基于这份快照，避免持锁过久。
 */
struct SharedSensorSnapshot {
    std::optional<ImuFrame>                 imu_frame;
    std::optional<preprocess::DvlRawSample> dvl_raw;
    MonoTimeNs                              last_imu_ns{0};
    MonoTimeNs                              last_dvl_ns{0};
};

/**
 * @brief nav_daemon 主循环跨周期持有的运行时状态。
 *
 * 包含：
 *  - 设备绑定状态与最近一次打印/事件快照；
 *  - 样本 freshness / out-of-order 判定所需的最后消费时间；
 *  - 上一帧可复用 IMU 样本及其 timing 元数据。
 */
struct NavLoopState {
    ManagedDeviceRuntime         imu_runtime;
    ManagedDeviceRuntime         dvl_runtime;
    std::uint64_t                print_counter{0};
    MonoTimeNs                   last_print_ns{0};
    MonoTimeNs                   last_used_imu_ns{0};
    MonoTimeNs                   last_used_dvl_ns{0};
    std::optional<ImuSample>     last_consumed_nav_imu;
    SampleTiming                 last_consumed_imu_timing{};
    SampleTiming                 last_consumed_dvl_timing{};
    RejectEventTracker           imu_reject_tracker{};
    RejectEventTracker           dvl_reject_tracker{};
    NavPublishSnapshot           last_nav_publish_snapshot{};
    bool                         have_last_nav_publish_snapshot{false};

    NavLoopState(ManagedDeviceRuntime imu_runtime_in,
                 ManagedDeviceRuntime dvl_runtime_in);
};

MonoTimeNs loop_monotonic_now_ns();

NavLoopState make_nav_loop_state(const NavDaemonConfig& cfg);

void handle_imu_connectivity_change(NavLoopState& loop_state,
                                    preprocess::ImuRtPreprocessor& imu_pp,
                                    estimator::EskfFilter& eskf);

void handle_dvl_connectivity_change(NavLoopState& loop_state,
                                    preprocess::DvlRtPreprocessor& dvl_pp);

SharedSensorSnapshot snapshot_shared_sensor_state(SharedSensorState& shared_state);

} // namespace nav_core::app
