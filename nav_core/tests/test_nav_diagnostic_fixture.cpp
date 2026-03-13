#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "nav_core/io/log_packets.hpp"
#include "shared/msg/nav_state.hpp"

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

template <class Pod>
bool write_pod(std::ofstream& ofs, const Pod& pod)
{
    ofs.write(reinterpret_cast<const char*>(&pod), sizeof(Pod));
    return static_cast<bool>(ofs);
}

int test_nav_fixture_writes_timing_and_state_logs()
{
    const auto base_ns = static_cast<std::int64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count());
    const char* keep_root_env = std::getenv("UWSYS_KEEP_NAV_FIXTURE_ROOT");
    const auto root = keep_root_env != nullptr
        ? std::filesystem::path(keep_root_env)
        : (std::filesystem::temp_directory_path() /
           ("nav_diag_fixture_" + std::to_string(static_cast<long long>(::getpid()))));
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    std::ofstream timing(root / "nav_timing.bin", std::ios::binary | std::ios::trunc);
    std::ofstream nav_state(root / "nav_state.bin", std::ios::binary | std::ios::trunc);
    TEST_CHECK(timing.is_open());
    TEST_CHECK(nav_state.is_open());

    nav_core::io::TimingTracePacketV1 pkt{};
    pkt.version = 1;
    pkt.kind = static_cast<std::uint16_t>(nav_core::io::TimingTraceKind::kImuDeviceState);
    pkt.flags = nav_core::io::kTimingTraceDeviceReconnecting;
    pkt.publish_mono_ns = base_ns + 1'000'000'000ll;
    pkt.age_ms = 0xFFFFFFFFu;
    pkt.fault_code = 6; // DeviceConnectionState::RECONNECTING
    TEST_CHECK(write_pod(timing, pkt));

    pkt = {};
    pkt.version = 1;
    pkt.kind = static_cast<std::uint16_t>(nav_core::io::TimingTraceKind::kNavPublished);
    pkt.flags = nav_core::io::kTimingTraceFresh |
                nav_core::io::kTimingTraceAccepted |
                nav_core::io::kTimingTraceValid;
    pkt.sensor_time_ns = base_ns + 1'010'000'000ll;
    pkt.recv_mono_ns = base_ns + 1'011'000'000ll;
    pkt.consume_mono_ns = base_ns + 1'012'000'000ll;
    pkt.publish_mono_ns = base_ns + 1'020'000'000ll;
    pkt.age_ms = 12;
    pkt.fault_code = 0;
    TEST_CHECK(write_pod(timing, pkt));

    pkt = {};
    pkt.version = 1;
    pkt.kind = static_cast<std::uint16_t>(nav_core::io::TimingTraceKind::kNavPublished);
    pkt.flags = nav_core::io::kTimingTraceStale |
                nav_core::io::kTimingTraceDegraded;
    pkt.sensor_time_ns = base_ns + 1'100'000'000ll;
    pkt.recv_mono_ns = base_ns + 1'101'000'000ll;
    pkt.consume_mono_ns = base_ns + 1'102'000'000ll;
    pkt.publish_mono_ns = base_ns + 1'220'000'000ll;
    pkt.age_ms = 220;
    pkt.fault_code = static_cast<std::uint32_t>(shared::msg::NavFaultCode::kImuDisconnected);
    TEST_CHECK(write_pod(timing, pkt));

    shared::msg::NavState state{};
    state.t_ns = static_cast<std::uint64_t>(base_ns + 1'020'000'000ll);
    state.age_ms = 12;
    state.valid = 1;
    state.stale = 0;
    state.degraded = 0;
    state.nav_state = shared::msg::NavRunState::kOk;
    state.health = shared::msg::NavHealth::OK;
    state.fault_code = shared::msg::NavFaultCode::kNone;
    state.sensor_mask = shared::msg::NAV_SENSOR_IMU;
    state.status_flags = shared::msg::NAV_FLAG_IMU_OK |
                         shared::msg::NAV_FLAG_IMU_DEVICE_ONLINE |
                         shared::msg::NAV_FLAG_ALIGN_DONE |
                         shared::msg::NAV_FLAG_ESKF_OK;
    TEST_CHECK(write_pod(nav_state, state));

    state = {};
    state.t_ns = static_cast<std::uint64_t>(base_ns + 1'220'000'000ll);
    state.age_ms = 220;
    state.valid = 0;
    state.stale = 1;
    state.degraded = 1;
    state.nav_state = shared::msg::NavRunState::kInvalid;
    state.health = shared::msg::NavHealth::INVALID;
    state.fault_code = shared::msg::NavFaultCode::kImuDisconnected;
    state.status_flags = shared::msg::NAV_FLAG_IMU_RECONNECTING;
    TEST_CHECK(write_pod(nav_state, state));

    timing.close();
    nav_state.close();

    TEST_EQ(std::filesystem::file_size(root / "nav_timing.bin"),
            3u * sizeof(nav_core::io::TimingTracePacketV1));
    TEST_EQ(std::filesystem::file_size(root / "nav_state.bin"),
            2u * sizeof(shared::msg::NavState));

    if (keep_root_env == nullptr) {
        std::filesystem::remove_all(root);
    } else {
        std::cout << "[test_nav_diagnostic_fixture] kept logs at " << root << "\n";
    }
    return 0;
}

} // namespace

int main()
{
    const int rc = test_nav_fixture_writes_timing_and_state_logs();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_nav_diagnostic_fixture] all tests passed.\n";
    return 0;
}
