#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

#include "nav_core/app/nav_state_replay.hpp"

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

namespace fs = std::filesystem;

shared::msg::NavState make_state(std::uint64_t t_ns,
                                 std::uint8_t  valid,
                                 std::uint16_t fault_code,
                                 std::uint16_t status_flags)
{
    shared::msg::NavState state{};
    state.t_ns = t_ns;
    state.valid = valid;
    state.stale = 0;
    state.degraded = 0;
    state.nav_state = valid ? shared::msg::NavRunState::kOk : shared::msg::NavRunState::kInvalid;
    state.health = valid ? shared::msg::NavHealth::OK : shared::msg::NavHealth::INVALID;
    state.age_ms = 20;
    state.fault_code = static_cast<shared::msg::NavFaultCode>(fault_code);
    state.status_flags = status_flags;
    return state;
}

int test_load_replay_file_preserves_records()
{
    const fs::path path = fs::temp_directory_path() / "nav_state_replay_load.bin";
    const std::vector<shared::msg::NavState> expected{
        make_state(1'000'000'000ull, 1, 0, shared::msg::NAV_FLAG_IMU_OK),
        make_state(1'040'000'000ull, 0, 12, shared::msg::NAV_FLAG_IMU_RECONNECTING),
    };

    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(expected.data()),
                  static_cast<std::streamsize>(expected.size() * sizeof(shared::msg::NavState)));
    }

    const auto loaded = nav_core::app::load_nav_state_replay_file(path);
    TEST_EQ(loaded.size(), expected.size());
    TEST_EQ(loaded[1].t_ns, expected[1].t_ns);
    TEST_EQ(static_cast<unsigned>(loaded[1].fault_code), static_cast<unsigned>(expected[1].fault_code));
    TEST_EQ(loaded[1].status_flags, expected[1].status_flags);
    std::error_code ec;
    fs::remove(path, ec);
    return 0;
}

int test_replay_records_calls_publish_in_order()
{
    const std::vector<shared::msg::NavState> records{
        make_state(1'000'000'000ull, 1, 0, shared::msg::NAV_FLAG_IMU_OK),
        make_state(1'050'000'000ull, 0, 12, shared::msg::NAV_FLAG_IMU_RECONNECTING),
        make_state(1'090'000'000ull, 0, 14, shared::msg::NAV_FLAG_DVL_BIND_MISMATCH),
    };

    std::vector<shared::msg::NavState> published;
    nav_core::app::NavStateReplayOptions options{};
    options.speed = 1'000'000.0;
    options.max_frames = 2;
    options.print_every = 0;

    const int rc = nav_core::app::replay_nav_state_records(
        records,
        options,
        [&](const shared::msg::NavState& state) {
            published.push_back(state);
            return true;
        },
        std::cerr);

    TEST_EQ(rc, 0);
    TEST_EQ(published.size(), 2u);
    TEST_EQ(published[0].t_ns, records[0].t_ns);
    TEST_EQ(static_cast<unsigned>(published[1].fault_code), 12u);
    TEST_CHECK(shared::msg::nav_flag_has(
        published[1].status_flags, shared::msg::NavStatusFlags::NAV_FLAG_IMU_RECONNECTING));
    return 0;
}

} // namespace

int main()
{
    int rc = test_load_replay_file_preserves_records();
    if (rc != 0) return rc;
    rc = test_replay_records_calls_publish_in_order();
    if (rc != 0) return rc;

    std::cout << "[test_nav_state_replay] all tests passed.\n";
    return 0;
}
