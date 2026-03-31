#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include "nav_core/app/nav_daemon_config.hpp"

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

int test_hex_slave_addr_is_loaded()
{
    const auto tmp_dir = std::filesystem::temp_directory_path() / "nav_daemon_config_tests";
    std::filesystem::create_directories(tmp_dir);
    const auto yaml_path = tmp_dir / "nav_daemon_hex_addr.yaml";

    {
        std::ofstream ofs(yaml_path);
        TEST_CHECK(ofs.is_open());
        ofs << "imu:\n";
        ofs << "  driver:\n";
        ofs << "    slave_addr: 0x50\n";
    }

    nav_core::app::NavDaemonConfig cfg{};
    std::string err;
    TEST_CHECK(nav_core::app::load_nav_daemon_config_from_yaml(yaml_path.string(), cfg, &err));
    TEST_EQ(static_cast<unsigned int>(cfg.imu.driver.slave_addr), 0x50u);

    std::filesystem::remove(yaml_path);
    return 0;
}

} // namespace

int main()
{
    const int rc = test_hex_slave_addr_is_loaded();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_nav_daemon_config] all tests passed.\n";
    return 0;
}
