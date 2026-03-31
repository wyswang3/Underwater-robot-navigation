#include <iostream>

#include "nav_core/estimator/nav_health_monitor.hpp"

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
        const auto _va = (a);                                                                     \
        const auto _vb = (b);                                                                     \
        if (!((_va) == (_vb))) {                                                                  \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " EQ(" #a ", " #b ") failed\n";                                          \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

int test_transport_timing_root_cause_when_imu_is_dead()
{
    nav_core::estimator::NavHealthConfig cfg{};
    cfg.imu_timeout_ns = 100'000'000;
    cfg.dvl_timeout_ns = 500'000'000;

    nav_core::estimator::NavHealthMonitor monitor(cfg);
    monitor.notify_eskf_reset(1'000'000'000ll);
    monitor.update_sensor_heartbeat(1'500'000'000ll, 1'200'000'000ll, 1'450'000'000ll);

    const auto report = monitor.evaluate(1'500'000'000ll);
    TEST_EQ(report.decision.health, shared::msg::NavHealth::INVALID);
    TEST_EQ(report.root_cause, nav_core::estimator::NavAuditRootCause::kTransportTiming);
    TEST_CHECK(report.decision.recommend_stop_motion);
    return 0;
}

int test_sensor_input_root_cause_when_preprocess_rejects_dominate()
{
    nav_core::estimator::NavHealthConfig cfg{};
    nav_core::estimator::NavHealthMonitor monitor(cfg);
    monitor.notify_eskf_reset(1'000'000'000ll);

    for (int i = 0; i < 6; ++i) {
        monitor.notify_imu_sample_issue(
            1'000'000'000ll + i,
            nav_core::estimator::SensorAuditIssue::kPreprocessRejected);
    }
    for (int i = 0; i < 2; ++i) {
        monitor.notify_imu_propagate(1'100'000'000ll + i);
    }
    monitor.update_sensor_heartbeat(1'200'000'000ll, 1'190'000'000ll, 1'190'000'000ll);

    const auto report = monitor.evaluate(1'200'000'000ll);
    TEST_EQ(report.decision.health, shared::msg::NavHealth::INVALID);
    TEST_EQ(report.root_cause, nav_core::estimator::NavAuditRootCause::kSensorInput);
    TEST_CHECK(report.metrics.sensor_input_suspect);
    return 0;
}

int test_estimator_consistency_root_cause_when_nis_is_high()
{
    nav_core::estimator::NavHealthConfig cfg{};
    nav_core::estimator::NavHealthMonitor monitor(cfg);
    monitor.notify_eskf_reset(1'000'000'000ll);
    monitor.update_sensor_heartbeat(1'100'000'000ll, 1'090'000'000ll, 1'090'000'000ll);

    for (int i = 0; i < 5; ++i) {
        monitor.notify_dvl_sample_issue(
            1'100'000'000ll + i,
            nav_core::estimator::SensorAuditIssue::kAccepted);
        monitor.notify_dvl_xy_update(1'100'000'000ll + i, 30.0, false);
    }

    const auto report = monitor.evaluate(1'200'000'000ll);
    TEST_EQ(report.decision.health, shared::msg::NavHealth::INVALID);
    TEST_EQ(report.root_cause,
            nav_core::estimator::NavAuditRootCause::kEstimatorConsistency);
    TEST_CHECK(report.metrics.estimator_consistency_suspect);
    return 0;
}

int test_numeric_invalid_root_cause_when_numeric_fault_seen()
{
    nav_core::estimator::NavHealthConfig cfg{};
    nav_core::estimator::NavHealthMonitor monitor(cfg);
    monitor.notify_eskf_reset(1'000'000'000ll);
    monitor.update_sensor_heartbeat(1'100'000'000ll, 1'090'000'000ll, 1'090'000'000ll);
    monitor.notify_numeric_invalid(1'150'000'000ll);

    const auto report = monitor.evaluate(1'200'000'000ll);
    TEST_EQ(report.decision.health, shared::msg::NavHealth::INVALID);
    TEST_EQ(report.root_cause, nav_core::estimator::NavAuditRootCause::kEstimatorNumeric);
    TEST_CHECK(report.metrics.estimator_numeric_invalid);
    return 0;
}

} // namespace

int main()
{
    int rc = test_transport_timing_root_cause_when_imu_is_dead();
    if (rc != 0) return rc;
    rc = test_sensor_input_root_cause_when_preprocess_rejects_dominate();
    if (rc != 0) return rc;
    rc = test_estimator_consistency_root_cause_when_nis_is_high();
    if (rc != 0) return rc;
    rc = test_numeric_invalid_root_cause_when_numeric_fault_seen();
    if (rc != 0) return rc;

    std::cout << "[test_nav_health_monitor] all tests passed.\n";
    return 0;
}
