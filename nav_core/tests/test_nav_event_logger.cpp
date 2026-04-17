#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "nav_core/app/nav_daemon_logging.hpp"

namespace {

#define TEST_CHECK(cond)                                                                          \
    do {                                                                                          \
        if (!(cond)) {                                                                            \
            std::cerr << "[FAIL] " << __FILE__ << ":" << __LINE__                                 \
                      << " CHECK(" #cond ") failed\n";                                            \
            return 1;                                                                             \
        }                                                                                         \
    } while (0)

int test_operator_policy_event_is_written_to_nav_events_csv()
{
    const auto root =
        std::filesystem::temp_directory_path() /
        ("nav_event_logger_" + std::to_string(static_cast<long long>(::getpid())));
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root);

    nav_core::app::NavDaemonConfig cfg{};
    cfg.logging.enable = true;
    cfg.logging.base_dir = root.string();
    cfg.logging.split_by_date = false;

    nav_core::app::NavEventCsvLogger logger;
    TEST_CHECK(logger.init(cfg));

    logger.log_operator_policy_applied(123456789ll, true, "nav_lane_manager");

    std::filesystem::path path;
    for (const auto& entry : std::filesystem::recursive_directory_iterator(root)) {
        if (entry.is_regular_file() && entry.path().filename() == "nav_events.csv") {
            path = entry.path();
            break;
        }
    }
    TEST_CHECK(!path.empty());

    std::ifstream ifs(path);
    TEST_CHECK(ifs.is_open());
    const std::string contents((std::istreambuf_iterator<char>(ifs)),
                               std::istreambuf_iterator<char>());

    TEST_CHECK(contents.find("operator_policy_applied") != std::string::npos);
    TEST_CHECK(contents.find("nav_lane_manager") != std::string::npos);
    TEST_CHECK(contents.find("dvl_enabled=enabled") != std::string::npos);
    TEST_CHECK(contents.find("\"dvl\"") != std::string::npos);
    TEST_CHECK(contents.find("\"operator_policy\"") != std::string::npos);

    std::filesystem::remove_all(root);
    return 0;
}

} // namespace

int main()
{
    const int rc = test_operator_policy_event_is_written_to_nav_events_csv();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_nav_event_logger] all tests passed.\n";
    return 0;
}
