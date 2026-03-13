#include <cstdint>
#include <iostream>

#include "nav_core/app/nav_runtime_status.hpp"
#include "nav_core/app/sample_timing.hpp"

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
                      << " EQ(" #a ", " #b ") failed. got=" << static_cast<long long>(_va)       \
                      << " expect=" << static_cast<long long>(_vb) << "\n";                       \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

int test_delayed_sample_becomes_stale_from_sample_time()
{
    nav_core::app::SampleTiming timing{};
    timing.sensor_time_ns = 1'000'000'000ll;
    timing.recv_mono_ns = 1'180'000'000ll;
    timing.consume_mono_ns = 1'220'000'000ll;

    TEST_CHECK(!nav_core::app::is_sample_fresh(1'250'000'000ll, timing, 0.2));
    TEST_EQ(nav_core::app::compute_age_ms(1'250'000'000ll, timing.sample_mono_ns()), 250u);
    return 0;
}

int test_out_of_order_and_duplicate_samples_are_rejected()
{
    TEST_CHECK(nav_core::app::should_consume_sample(100, 0));
    TEST_CHECK(nav_core::app::should_consume_sample(200, 100));
    TEST_CHECK(!nav_core::app::should_consume_sample(200, 200));
    TEST_CHECK(!nav_core::app::should_consume_sample(150, 200));
    return 0;
}

int test_nav_publish_semantics_use_sample_age_not_consume_age()
{
    shared::msg::NavState nav{};
    nav.t_ns = 1'000'000'000ull;

    nav_core::app::NavPublishContext ctx{};
    ctx.publish_mono_ns = 1'250'000'000ll;
    ctx.state_stamp_ns = 1'000'000'000ll;
    ctx.imu_timing.sensor_time_ns = 1'000'000'000ll;
    ctx.imu_timing.recv_mono_ns = 1'180'000'000ll;
    ctx.imu_timing.consume_mono_ns = 1'220'000'000ll;
    ctx.max_imu_age_s = 0.2;
    ctx.imu_enabled = true;
    ctx.imu_bias_ready = true;

    nav_core::app::apply_nav_publish_semantics(ctx, true, nav);

    TEST_CHECK(nav.valid == 0);
    TEST_CHECK(nav.stale == 1);
    TEST_EQ(nav.nav_state, shared::msg::NavRunState::kInvalid);
    TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuStale);
    TEST_EQ(nav.age_ms, 250u);
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_delayed_sample_becomes_stale_from_sample_time();
    if (rc != 0) return rc;
    rc = test_out_of_order_and_duplicate_samples_are_rejected();
    if (rc != 0) return rc;
    rc = test_nav_publish_semantics_use_sample_age_not_consume_age();
    if (rc != 0) return rc;

    std::cout << "[test_sample_timing] all tests passed.\n";
    return 0;
}
