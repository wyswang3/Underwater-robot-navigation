#include "nav_core/app/nav_runtime_status.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace nav_core::app {

namespace {

using shared::msg::NavFaultCode;
using shared::msg::NavHealth;
using shared::msg::NavRunState;

constexpr std::uint32_t kAgeUnknownMs = 0xFFFFFFFFu;

inline bool is_finite3(const nav_core::Vec3d& v) noexcept
{
    return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

inline bool age_within_limit(MonoTimeNs now_ns,
                             MonoTimeNs sample_ns,
                             double     max_age_s) noexcept
{
    if (sample_ns <= 0) {
        return false;
    }
    if (max_age_s <= 0.0) {
        return true;
    }
    if (now_ns < sample_ns) {
        return false;
    }
    const double age_s = static_cast<double>(now_ns - sample_ns) * 1e-9;
    return age_s <= max_age_s;
}

inline std::uint32_t compute_age_ms(MonoTimeNs now_ns, MonoTimeNs stamp_ns) noexcept
{
    if (stamp_ns <= 0 || now_ns < stamp_ns) {
        return kAgeUnknownMs;
    }
    const std::uint64_t age_ms64 =
        static_cast<std::uint64_t>(now_ns - stamp_ns) / 1000000ull;
    return (age_ms64 > std::numeric_limits<std::uint32_t>::max())
        ? std::numeric_limits<std::uint32_t>::max()
        : static_cast<std::uint32_t>(age_ms64);
}

} // namespace

bool eskf_state_is_finite(const estimator::EskfFilter& eskf) noexcept
{
    const auto st = eskf.state();
    return is_finite3(st.p_enu) &&
           is_finite3(st.v_enu) &&
           is_finite3(st.rpy_nb_rad) &&
           is_finite3(st.ba_mps2) &&
           is_finite3(st.bg_rad_s);
}

void fill_nav_state_kinematics(const estimator::EskfFilter& eskf,
                               shared::msg::NavState&       nav,
                               MonoTimeNs                   state_stamp_ns,
                               const ImuSample*             latest_imu_sample) noexcept
{
    const auto st = eskf.state();

    nav.t_ns = static_cast<std::uint64_t>(state_stamp_ns);

    nav.pos[0] = st.p_enu.x;
    nav.pos[1] = st.p_enu.y;
    nav.pos[2] = st.p_enu.z;

    nav.vel[0] = st.v_enu.x;
    nav.vel[1] = st.v_enu.y;
    nav.vel[2] = st.v_enu.z;

    nav.rpy[0] = st.rpy_nb_rad.x;
    nav.rpy[1] = st.rpy_nb_rad.y;
    nav.rpy[2] = st.rpy_nb_rad.z;

    nav.depth = -st.p_enu.z;

    // Body kinematics are control-facing measured quantities, not ESKF bias states.
    if (latest_imu_sample != nullptr) {
        nav.omega_b[0] = static_cast<double>(latest_imu_sample->ang_vel[0]);
        nav.omega_b[1] = static_cast<double>(latest_imu_sample->ang_vel[1]);
        nav.omega_b[2] = static_cast<double>(latest_imu_sample->ang_vel[2]);

        nav.acc_b[0] = static_cast<double>(latest_imu_sample->lin_acc[0]);
        nav.acc_b[1] = static_cast<double>(latest_imu_sample->lin_acc[1]);
        nav.acc_b[2] = static_cast<double>(latest_imu_sample->lin_acc[2]);
    } else {
        nav.omega_b[0] = 0.0;
        nav.omega_b[1] = 0.0;
        nav.omega_b[2] = 0.0;

        nav.acc_b[0] = 0.0;
        nav.acc_b[1] = 0.0;
        nav.acc_b[2] = 0.0;
    }
}

void apply_nav_publish_semantics(const NavPublishContext& ctx,
                                 bool                     state_finite,
                                 shared::msg::NavState&   nav) noexcept
{
    nav.age_ms = compute_age_ms(ctx.publish_mono_ns, static_cast<MonoTimeNs>(nav.t_ns));
    nav.valid = 0;
    nav.stale = 0;
    nav.degraded = 0;
    nav.nav_state = NavRunState::kUninitialized;
    nav.health = NavHealth::UNINITIALIZED;
    nav.fault_code = NavFaultCode::kEstimatorUninitialized;
    nav.sensor_mask = shared::msg::NAV_SENSOR_NONE;
    nav.status_flags = shared::msg::NAV_FLAG_NONE;

    const bool imu_seen = ctx.last_imu_mono_ns > 0;
    const bool have_state = ctx.state_stamp_ns > 0;
    const bool imu_fresh = ctx.imu_enabled &&
                           age_within_limit(ctx.publish_mono_ns,
                                            ctx.last_imu_mono_ns,
                                            ctx.max_imu_age_s);
    const bool dvl_fresh = ctx.dvl_enabled &&
                           age_within_limit(ctx.publish_mono_ns,
                                            ctx.last_dvl_mono_ns,
                                            ctx.max_dvl_age_s);

    if (imu_fresh) {
        shared::msg::nav_sensor_set(nav.sensor_mask, shared::msg::NAV_SENSOR_IMU);
        shared::msg::nav_flag_set(nav.status_flags, shared::msg::NAV_FLAG_IMU_OK);
    }
    if (dvl_fresh) {
        shared::msg::nav_sensor_set(nav.sensor_mask, shared::msg::NAV_SENSOR_DVL);
        shared::msg::nav_flag_set(nav.status_flags, shared::msg::NAV_FLAG_DVL_OK);
    }
    if (ctx.imu_bias_ready) {
        shared::msg::nav_flag_set(nav.status_flags, shared::msg::NAV_FLAG_ALIGN_DONE);
    }

    if (!ctx.imu_enabled || !imu_seen) {
        nav.nav_state = NavRunState::kUninitialized;
        nav.health = NavHealth::UNINITIALIZED;
        nav.fault_code = NavFaultCode::kImuNoData;
        return;
    }

    if (!ctx.imu_bias_ready || !have_state) {
        nav.nav_state = NavRunState::kAligning;
        nav.health = NavHealth::UNINITIALIZED;
        nav.fault_code = ctx.imu_bias_ready
            ? NavFaultCode::kEstimatorUninitialized
            : NavFaultCode::kAlignmentPending;
        return;
    }

    if (!state_finite) {
        nav.nav_state = NavRunState::kInvalid;
        nav.health = NavHealth::INVALID;
        nav.fault_code = NavFaultCode::kEstimatorNumericInvalid;
        return;
    }

    if (!imu_fresh) {
        nav.nav_state = NavRunState::kInvalid;
        nav.health = NavHealth::INVALID;
        nav.fault_code = NavFaultCode::kImuStale;
        nav.stale = 1;
        return;
    }

    shared::msg::nav_flag_set(nav.status_flags, shared::msg::NAV_FLAG_ESKF_OK);

    if (ctx.dvl_enabled && !dvl_fresh) {
        nav.valid = 1;
        nav.degraded = 1;
        nav.nav_state = NavRunState::kDegraded;
        nav.health = NavHealth::DEGRADED;
        nav.fault_code = NavFaultCode::kNone;
        return;
    }

    nav.valid = 1;
    nav.nav_state = NavRunState::kOk;
    nav.health = NavHealth::OK;
    nav.fault_code = NavFaultCode::kNone;
}

} // namespace nav_core::app
