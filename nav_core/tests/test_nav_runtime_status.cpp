#include <cmath>
#include <cstdint>
#include <iostream>

#include "nav_core/app/nav_runtime_status.hpp"
#include "nav_core/estimator/eskf.hpp"

namespace {

#define TEST_CHECK(cond)                                                                          \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " CHECK(" #cond ") failed\n";                                            \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_EQ(a, b)                                                                             \
    do {                                                                                          \
        auto _va = (a);                                                                           \
        auto _vb = (b);                                                                           \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed. got=" << static_cast<int>(_va)              \
                      << " expect=" << static_cast<int>(_vb) << "\n";                             \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

#define TEST_NEAR(a, b, eps)                                                                      \
    do {                                                                                          \
        const auto _va = static_cast<double>(a);                                                  \
        const auto _vb = static_cast<double>(b);                                                  \
        const auto _ve = static_cast<double>(eps);                                                \
        if (std::fabs(_va - _vb) > _ve) {                                                         \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " NEAR(" #a ", " #b ", " #eps ") failed. got=" << _va                    \
                      << " expect=" << _vb << " eps=" << _ve << "\n";                             \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

shared::msg::NavState make_nav_snapshot(nav_core::estimator::EskfFilter& eskf,
                                        nav_core::MonoTimeNs             stamp_ns,
                                        const nav_core::ImuSample*       imu_sample = nullptr)
{
    shared::msg::NavState nav{};
    nav_core::app::fill_nav_state_kinematics(eskf, nav, stamp_ns, imu_sample);
    return nav;
}

int test_nav_body_kinematics_use_latest_imu_measurement()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    nav_core::ImuSample imu{};
    imu.ang_vel[0] = 0.1f;
    imu.ang_vel[1] = -0.2f;
    imu.ang_vel[2] = 0.3f;
    imu.lin_acc[0] = 1.0f;
    imu.lin_acc[1] = 2.0f;
    imu.lin_acc[2] = -3.0f;

    const auto nav = make_nav_snapshot(eskf, 42, &imu);
    TEST_NEAR(nav.omega_b[0], 0.1, 1e-6);
    TEST_NEAR(nav.omega_b[1], -0.2, 1e-6);
    TEST_NEAR(nav.omega_b[2], 0.3, 1e-6);
    TEST_NEAR(nav.acc_b[0], 1.0, 1e-6);
    TEST_NEAR(nav.acc_b[1], 2.0, 1e-6);
    TEST_NEAR(nav.acc_b[2], -3.0, 1e-6);
    return 0;
}

int test_nav_body_kinematics_zero_when_no_imu_measurement()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    const auto nav = make_nav_snapshot(eskf, 42, nullptr);
    TEST_NEAR(nav.omega_b[0], 0.0, 1e-9);
    TEST_NEAR(nav.omega_b[1], 0.0, 1e-9);
    TEST_NEAR(nav.omega_b[2], 0.0, 1e-9);
    TEST_NEAR(nav.acc_b[0], 0.0, 1e-9);
    TEST_NEAR(nav.acc_b[1], 0.0, 1e-9);
    TEST_NEAR(nav.acc_b[2], 0.0, 1e-9);
    return 0;
}

int test_uninitialized_nav_is_not_publishable()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    auto nav = make_nav_snapshot(eskf, 0);
    nav_core::app::NavPublishContext ctx{};
    ctx.publish_mono_ns = 1'000'000'000ll;
    ctx.max_imu_age_s = 0.2;
    ctx.max_dvl_age_s = 3.0;
    ctx.imu_enabled = true;
    ctx.dvl_enabled = true;
    ctx.imu_bias_ready = false;

    nav_core::app::apply_nav_publish_semantics(ctx, true, nav);

    TEST_EQ(nav.nav_state, shared::msg::NavRunState::kUninitialized);
    TEST_EQ(nav.health, shared::msg::NavHealth::UNINITIALIZED);
    TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuDeviceNotFound);
    TEST_CHECK(nav.valid == 0);
    TEST_CHECK(nav.stale == 0);
    TEST_EQ(nav.status_flags, shared::msg::NAV_FLAG_NONE);
    return 0;
}

int test_aligning_nav_stays_invalid()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    auto nav = make_nav_snapshot(eskf, 0);
    nav_core::app::NavPublishContext ctx{};
    ctx.publish_mono_ns = 2'000'000'000ll;
    ctx.imu_timing.sensor_time_ns = 1'950'000'000ll;
    ctx.imu_timing.recv_mono_ns = 1'951'000'000ll;
    ctx.max_imu_age_s = 0.2;
    ctx.max_dvl_age_s = 3.0;
    ctx.imu_enabled = true;
    ctx.dvl_enabled = true;
    ctx.imu_bias_ready = false;

    nav_core::app::apply_nav_publish_semantics(ctx, true, nav);

    TEST_EQ(nav.nav_state, shared::msg::NavRunState::kAligning);
    TEST_EQ(nav.health, shared::msg::NavHealth::UNINITIALIZED);
    TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kAlignmentPending);
    TEST_CHECK(nav.valid == 0);
    TEST_CHECK(nav.stale == 0);
    return 0;
}

int test_device_mismatch_sets_explicit_fault_and_flag()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    auto nav = make_nav_snapshot(eskf, 0);
    nav_core::app::NavPublishContext ctx{};
    ctx.publish_mono_ns = 2'000'000'000ll;
    ctx.max_imu_age_s = 0.2;
    ctx.max_dvl_age_s = 3.0;
    ctx.imu_enabled = true;
    ctx.dvl_enabled = true;
    ctx.imu_bias_ready = false;
    ctx.imu_device_state = nav_core::app::DeviceConnectionState::MISMATCH;
    ctx.dvl_device_state = nav_core::app::DeviceConnectionState::RECONNECTING;

    nav_core::app::apply_nav_publish_semantics(ctx, true, nav);

    TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuDeviceMismatch);
    TEST_CHECK(shared::msg::nav_flag_has(nav.status_flags,
                                         shared::msg::NAV_FLAG_IMU_BIND_MISMATCH));
    TEST_CHECK(shared::msg::nav_flag_has(nav.status_flags,
                                         shared::msg::NAV_FLAG_DVL_RECONNECTING));
    return 0;
}

int test_imu_stale_nav_becomes_invalid()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    constexpr nav_core::MonoTimeNs kStampNs = 1'000'000'000ll;
    auto nav = make_nav_snapshot(eskf, kStampNs);

    nav_core::app::NavPublishContext ctx{};
    ctx.publish_mono_ns = 1'500'000'000ll;
    ctx.state_stamp_ns = kStampNs;
    ctx.imu_timing.sensor_time_ns = 1'000'000'000ll;
    ctx.imu_timing.recv_mono_ns = 1'010'000'000ll;
    ctx.imu_timing.consume_mono_ns = 1'020'000'000ll;
    ctx.dvl_timing.sensor_time_ns = 1'400'000'000ll;
    ctx.dvl_timing.recv_mono_ns = 1'410'000'000ll;
    ctx.dvl_timing.consume_mono_ns = 1'420'000'000ll;
    ctx.max_imu_age_s = 0.2;
    ctx.max_dvl_age_s = 3.0;
    ctx.imu_enabled = true;
    ctx.dvl_enabled = true;
    ctx.imu_bias_ready = true;

    nav_core::app::apply_nav_publish_semantics(ctx, true, nav);

    TEST_EQ(nav.nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(nav.health, shared::msg::NavHealth::INVALID);
    TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuStale);
    TEST_CHECK(nav.valid == 0);
    TEST_CHECK(nav.stale == 1);
    TEST_EQ(nav.age_ms, 500u);
    return 0;
}

int test_dvl_loss_is_explicitly_degraded()
{
    nav_core::estimator::EskfConfig cfg{};
    nav_core::estimator::EskfFilter eskf(cfg);

    constexpr nav_core::MonoTimeNs kStampNs = 3'000'000'000ll;
    auto nav = make_nav_snapshot(eskf, kStampNs);

    nav_core::app::NavPublishContext ctx{};
    ctx.publish_mono_ns = 3'050'000'000ll;
    ctx.state_stamp_ns = kStampNs;
    ctx.imu_timing.sensor_time_ns = 3'040'000'000ll;
    ctx.imu_timing.recv_mono_ns = 3'041'000'000ll;
    ctx.imu_timing.consume_mono_ns = 3'042'000'000ll;
    ctx.max_imu_age_s = 0.2;
    ctx.max_dvl_age_s = 3.0;
    ctx.imu_enabled = true;
    ctx.dvl_enabled = true;
    ctx.imu_bias_ready = true;

    nav_core::app::apply_nav_publish_semantics(ctx, true, nav);

    TEST_EQ(nav.nav_state, shared::msg::NavRunState::kDegraded);
    TEST_EQ(nav.health, shared::msg::NavHealth::DEGRADED);
    TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kNone);
    TEST_CHECK(nav.valid == 1);
    TEST_CHECK(nav.degraded == 1);
    TEST_CHECK(shared::msg::nav_sensor_has(nav.sensor_mask, shared::msg::NAV_SENSOR_IMU));
    TEST_CHECK(!shared::msg::nav_sensor_has(nav.sensor_mask, shared::msg::NAV_SENSOR_DVL));
    TEST_CHECK(shared::msg::nav_flag_has(nav.status_flags, shared::msg::NAV_FLAG_IMU_OK));
    TEST_CHECK(shared::msg::nav_flag_has(nav.status_flags, shared::msg::NAV_FLAG_ALIGN_DONE));
    TEST_CHECK(shared::msg::nav_flag_has(nav.status_flags, shared::msg::NAV_FLAG_ESKF_OK));
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_uninitialized_nav_is_not_publishable();
    if (rc != 0) return rc;
    rc = test_aligning_nav_stays_invalid();
    if (rc != 0) return rc;
    rc = test_device_mismatch_sets_explicit_fault_and_flag();
    if (rc != 0) return rc;
    rc = test_imu_stale_nav_becomes_invalid();
    if (rc != 0) return rc;
    rc = test_dvl_loss_is_explicitly_degraded();
    if (rc != 0) return rc;
    rc = test_nav_body_kinematics_use_latest_imu_measurement();
    if (rc != 0) return rc;
    rc = test_nav_body_kinematics_zero_when_no_imu_measurement();
    if (rc != 0) return rc;

    std::cout << "[test_nav_runtime_status] all tests passed.\n";
    return 0;
}
