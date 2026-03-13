#pragma once

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <functional>
#include <ostream>
#include <vector>

#include "shared/msg/nav_state.hpp"

namespace nav_core::app {

/**
 * @brief Minimal NavState replay options for incident-window validation.
 *
 * Semantics:
 *  - Replay cadence is derived from adjacent `NavState::t_ns` deltas.
 *  - The recorded `NavState::age_ms` is preserved; the replay hop only rebuilds
 *    SHM header publish times so downstream can re-evaluate stale/no-data policy.
 *  - `max_frames == 0` means replay the entire file once.
 */
struct NavStateReplayOptions {
    double      speed = 1.0;
    bool        loop = false;
    std::size_t max_frames = 0;
    std::size_t print_every = 50;
    std::uint32_t start_delay_ms = 0;
};

using NavStatePublishFn = std::function<bool(const shared::msg::NavState&)>;

std::vector<shared::msg::NavState> load_nav_state_replay_file(const std::filesystem::path& path);

int replay_nav_state_records(const std::vector<shared::msg::NavState>& records,
                             const NavStateReplayOptions&              options,
                             NavStatePublishFn                         publish,
                             std::ostream&                             log,
                             const std::atomic_bool*                   stop_flag = nullptr);

} // namespace nav_core::app
