// nav_core/src/nav_core/app/nav_daemon_loop_state.cpp
//
// 作用：
//   - 维护 nav_daemon 主循环跨 tick 复用的运行时状态；
//   - 集中处理设备切换后的 reset、共享快照读取和事件去重辅助逻辑。
//
// 实现思路：
//   - 把“遇到设备重连时哪些缓存要清空、哪些状态要复位”固定在这里；
//   - 主循环只调用统一 helper，避免 reset 语义散落在多个分支里。

#include "nav_core/app/nav_daemon_loop_state.hpp"

#include <chrono>
#include <cstdio>
#include <cstring>
#include <utility>

#include "nav_core/app/imu_port_selector.hpp"

namespace nav_core::app {
namespace {

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

} // namespace

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

shared::msg::NavFaultCode sensor_reject_fault_code(const char* sensor_id,
                                                   SensorRejectKind kind) noexcept
{
    const bool imu = (sensor_id != nullptr && std::strcmp(sensor_id, "imu") == 0);
    switch (kind) {
    case SensorRejectKind::kStale:
    case SensorRejectKind::kOutOfOrder:
        return imu ? shared::msg::NavFaultCode::kImuStale
                   : shared::msg::NavFaultCode::kNoData;
    case SensorRejectKind::kPreprocessRejected:
    case SensorRejectKind::kGatedRejected:
    case SensorRejectKind::kNone:
    default:
        return shared::msg::NavFaultCode::kNone;
    }
}

bool RejectEventTracker::should_log(SensorRejectKind kind)
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

void RejectEventTracker::clear() noexcept
{
    last_kind = SensorRejectKind::kNone;
}

bool NavHealthAuditSnapshot::operator==(const NavHealthAuditSnapshot& rhs) const noexcept
{
    return health == rhs.health &&
           root_cause == rhs.root_cause &&
           recommend_stop_motion == rhs.recommend_stop_motion &&
           recommend_reduce_speed == rhs.recommend_reduce_speed &&
           recommend_relocalize == rhs.recommend_relocalize;
}

NavLoopState::NavLoopState(ManagedDeviceRuntime imu_runtime_in,
                           ManagedDeviceRuntime dvl_runtime_in)
    : imu_runtime(std::move(imu_runtime_in)),
      dvl_runtime(std::move(dvl_runtime_in))
{
}

MonoTimeNs loop_monotonic_now_ns()
{
    using clock = std::chrono::steady_clock;
    const auto now = clock::now().time_since_epoch();
    return static_cast<MonoTimeNs>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
}

NavLoopState make_nav_loop_state(const NavDaemonConfig& cfg)
{
    return NavLoopState(
        ManagedDeviceRuntime{DeviceBinder("imu",
                                          cfg.imu.driver.port,
                                          cfg.imu.driver.binding,
                                          {},
                                          make_imu_port_selector(cfg.imu.driver))},
        ManagedDeviceRuntime{DeviceBinder("dvl", cfg.dvl.driver.port, cfg.dvl.driver.binding)});
}

void handle_imu_connectivity_change(NavLoopState& loop_state,
                                    preprocess::ImuRtPreprocessor& imu_pp,
                                    estimator::EskfFilter& eskf)
{
    imu_pp.reset();
    reset_eskf_to_config_defaults(eskf);
    loop_state.last_consumed_nav_imu.reset();
    loop_state.last_consumed_imu_timing = SampleTiming{};
    loop_state.last_used_imu_ns = 0;
    loop_state.last_consumed_dvl_timing = SampleTiming{};
    loop_state.last_used_dvl_ns = 0;
    std::fprintf(stderr,
                 "[nav_daemon] IMU connectivity changed; reset aligner and ESKF baseline\n");
}

void handle_dvl_connectivity_change(NavLoopState& loop_state,
                                    preprocess::DvlRtPreprocessor& dvl_pp)
{
    dvl_pp.reset();
    loop_state.last_consumed_dvl_timing = SampleTiming{};
    loop_state.last_used_dvl_ns = 0;
    std::fprintf(stderr,
                 "[nav_daemon] DVL connectivity changed; clear DVL freshness cache\n");
}

SharedSensorSnapshot snapshot_shared_sensor_state(SharedSensorState& shared_state)
{
    SharedSensorSnapshot snapshot{};
    {
        std::lock_guard<std::mutex> lock(shared_state.m);
        snapshot.imu_frame = shared_state.last_imu;
        snapshot.dvl_raw = shared_state.last_dvl_raw;
        snapshot.last_imu_ns = shared_state.last_imu_mono_ns;
        snapshot.last_dvl_ns = shared_state.last_dvl_mono_ns;
        ++shared_state.loop_counter;
    }
    return snapshot;
}

} // namespace nav_core::app
