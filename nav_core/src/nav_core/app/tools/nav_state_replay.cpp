#include <atomic>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>

#include "nav_core/app/nav_state_replay.hpp"
#include "nav_core/io/nav_state_publisher.hpp"

namespace {

std::atomic_bool g_stop{false};

void on_signal(int) { g_stop.store(true); }

struct Args {
    std::filesystem::path nav_state_bin{};
    std::filesystem::path incident_bundle{};
    std::string nav_state_shm{"/rov_nav_state_v1"};
    double speed{1.0};
    bool loop{false};
    std::size_t max_frames{0};
    std::size_t print_every{50};
    std::uint32_t start_delay_ms{0};
};

void usage(const char* prog)
{
    std::cerr
        << "Usage: " << prog << " [options]\n"
        << "Options:\n"
        << "  --nav-state-bin <path>      replay a raw nav_state.bin slice\n"
        << "  --incident-bundle <dir>     replay <dir>/nav_state_window.bin\n"
        << "  --nav-state-shm <name>      default: /rov_nav_state_v1\n"
        << "  --speed <factor>            replay speedup, default: 1.0\n"
        << "  --loop 0|1                  default: 0\n"
        << "  --max-frames <n>            0 means replay all frames\n"
        << "  --print-every <n>           default: 50\n"
        << "  --start-delay-ms <ms>       default: 0\n";
}

bool parse_bool(const std::string& raw, bool& out)
{
    if (raw == "1" || raw == "true") {
        out = true;
        return true;
    }
    if (raw == "0" || raw == "false") {
        out = false;
        return true;
    }
    return false;
}

bool parse_args(int argc, char** argv, Args& args)
{
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto need_value = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[nav_replay][ERR] missing value for " << opt << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (key == "--help" || key == "-h") {
            usage(argv[0]);
            return false;
        } else if (key == "--nav-state-bin") {
            const char* value = need_value("--nav-state-bin");
            if (!value) return false;
            args.nav_state_bin = value;
        } else if (key == "--incident-bundle") {
            const char* value = need_value("--incident-bundle");
            if (!value) return false;
            args.incident_bundle = value;
        } else if (key == "--nav-state-shm") {
            const char* value = need_value("--nav-state-shm");
            if (!value) return false;
            args.nav_state_shm = value;
        } else if (key == "--speed") {
            const char* value = need_value("--speed");
            if (!value) return false;
            args.speed = std::stod(value);
        } else if (key == "--loop") {
            const char* value = need_value("--loop");
            if (!value) return false;
            bool loop = false;
            if (!parse_bool(value, loop)) {
                std::cerr << "[nav_replay][ERR] invalid bool for --loop: " << value << "\n";
                return false;
            }
            args.loop = loop;
        } else if (key == "--max-frames") {
            const char* value = need_value("--max-frames");
            if (!value) return false;
            args.max_frames = static_cast<std::size_t>(std::stoull(value));
        } else if (key == "--print-every") {
            const char* value = need_value("--print-every");
            if (!value) return false;
            args.print_every = static_cast<std::size_t>(std::stoull(value));
        } else if (key == "--start-delay-ms") {
            const char* value = need_value("--start-delay-ms");
            if (!value) return false;
            args.start_delay_ms = static_cast<std::uint32_t>(std::stoul(value));
        } else {
            std::cerr << "[nav_replay][ERR] unknown option: " << key << "\n";
            usage(argv[0]);
            return false;
        }
    }

    if (!args.incident_bundle.empty()) {
        args.nav_state_bin = args.incident_bundle / "nav_state_window.bin";
    }
    if (args.nav_state_bin.empty()) {
        std::cerr << "[nav_replay][ERR] specify --nav-state-bin or --incident-bundle\n";
        return false;
    }
    if (args.nav_state_shm.empty() || args.nav_state_shm.front() != '/') {
        std::cerr << "[nav_replay][ERR] nav_state_shm must start with '/'\n";
        return false;
    }
    if (args.speed <= 0.0) {
        std::cerr << "[nav_replay][ERR] speed must be > 0\n";
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    Args args{};
    if (!parse_args(argc, argv, args)) {
        return 2;
    }

    std::vector<shared::msg::NavState> records;
    try {
        records = nav_core::app::load_nav_state_replay_file(args.nav_state_bin);
    } catch (const std::exception& e) {
        std::cerr << "[nav_replay][ERR] " << e.what() << "\n";
        return 3;
    }

    nav_core::io::NavStatePublisher publisher;
    nav_core::io::NavStatePublisherConfig cfg{};
    cfg.enable = true;
    cfg.shm_name = args.nav_state_shm;
    if (!publisher.init(cfg)) {
        std::cerr << "[nav_replay][ERR] NavStatePublisher init failed for "
                  << args.nav_state_shm << "\n";
        return 4;
    }

    nav_core::app::NavStateReplayOptions options{};
    options.speed = args.speed;
    options.loop = args.loop;
    options.max_frames = args.max_frames;
    options.print_every = args.print_every;
    options.start_delay_ms = args.start_delay_ms;

    std::cerr << "[nav_replay] input=" << args.nav_state_bin
              << " frames=" << records.size()
              << " speed=" << args.speed
              << " nav_state_shm=" << args.nav_state_shm << "\n";

    const int rc = nav_core::app::replay_nav_state_records(
        records,
        options,
        [&](const shared::msg::NavState& state) { return publisher.publish(state); },
        std::cerr,
        &g_stop);
    publisher.shutdown();
    return rc;
}
