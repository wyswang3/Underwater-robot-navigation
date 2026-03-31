#include "nav_core/app/nav_state_replay.hpp"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <stdexcept>
#include <string>
#include <thread>

namespace nav_core::app {

namespace {

bool should_stop(const std::atomic_bool* stop_flag) noexcept
{
    return stop_flag != nullptr && stop_flag->load();
}

void maybe_sleep(std::uint64_t delta_ns, double speed)
{
    if (delta_ns == 0 || speed <= 0.0) {
        return;
    }

    const auto wait_ns = static_cast<std::uint64_t>(
        static_cast<long double>(delta_ns) / static_cast<long double>(speed));
    if (wait_ns == 0) {
        return;
    }

    std::this_thread::sleep_for(std::chrono::nanoseconds(wait_ns));
}

} // namespace

std::vector<shared::msg::NavState> load_nav_state_replay_file(const std::filesystem::path& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        throw std::runtime_error("cannot open replay file: " + path.string());
    }

    in.seekg(0, std::ios::end);
    const auto size = static_cast<std::size_t>(in.tellg());
    in.seekg(0, std::ios::beg);

    if ((size % sizeof(shared::msg::NavState)) != 0) {
        throw std::runtime_error("replay file size is not a multiple of NavState");
    }

    std::vector<shared::msg::NavState> records(size / sizeof(shared::msg::NavState));
    if (!records.empty() &&
        !in.read(reinterpret_cast<char*>(records.data()), static_cast<std::streamsize>(size))) {
        throw std::runtime_error("failed to read replay file: " + path.string());
    }
    return records;
}

int replay_nav_state_records(const std::vector<shared::msg::NavState>& records,
                             const NavStateReplayOptions&              options,
                             NavStatePublishFn                         publish,
                             std::ostream&                             log,
                             const std::atomic_bool*                   stop_flag)
{
    if (!publish) {
        log << "[nav_replay][ERR] publish callback is empty\n";
        return 2;
    }
    if (options.speed <= 0.0) {
        log << "[nav_replay][ERR] speed must be > 0\n";
        return 3;
    }
    if (records.empty()) {
        log << "[nav_replay][ERR] no NavState records to replay\n";
        return 4;
    }

    const std::size_t frame_limit =
        (options.max_frames == 0 || options.max_frames > records.size())
            ? records.size()
            : options.max_frames;

    if (options.start_delay_ms > 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(options.start_delay_ms));
    }

    std::size_t published_total = 0;
    do {
        for (std::size_t idx = 0; idx < frame_limit; ++idx) {
            if (should_stop(stop_flag)) {
                return 0;
            }

            if (idx > 0) {
                const auto prev_t_ns = records[idx - 1].t_ns;
                const auto curr_t_ns = records[idx].t_ns;
                const std::uint64_t delta_ns = (curr_t_ns > prev_t_ns) ? (curr_t_ns - prev_t_ns) : 0;
                maybe_sleep(delta_ns, options.speed);
            }

            if (!publish(records[idx])) {
                log << "[nav_replay][ERR] publish failed at frame=" << idx << "\n";
                return 5;
            }

            ++published_total;
            if (options.print_every > 0 &&
                (published_total == 1 ||
                 (published_total % options.print_every) == 0 ||
                 idx + 1 == frame_limit)) {
                log << "[nav_replay] frame=" << published_total
                    << " t_ns=" << records[idx].t_ns
                    << " valid=" << static_cast<unsigned>(records[idx].valid)
                    << " stale=" << static_cast<unsigned>(records[idx].stale)
                    << " fault=" << static_cast<unsigned>(records[idx].fault_code)
                    << " status_flags=0x" << std::hex
                    << static_cast<unsigned>(records[idx].status_flags)
                    << std::dec << "\n";
            }
        }
    } while (options.loop && !should_stop(stop_flag));

    return 0;
}

} // namespace nav_core::app
