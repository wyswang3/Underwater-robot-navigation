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

int test_health_config_fields_are_loaded()
{
    const auto tmp_dir = std::filesystem::temp_directory_path() / "nav_daemon_config_tests";
    std::filesystem::create_directories(tmp_dir);
    const auto yaml_path = tmp_dir / "nav_daemon_health.yaml";

    {
        std::ofstream ofs(yaml_path);
        TEST_CHECK(ofs.is_open());
        ofs << "estimator:\n";
        ofs << "  enable_health: true\n";
        ofs << "  health:\n";
        ofs << "    imu_timeout_ns: 123000000\n";
        ofs << "    dvl_timeout_ns: 456000000\n";
        ofs << "    dvl_nis_ok_max: 7.5\n";
        ofs << "    dvl_nis_reject_max: 12.5\n";
        ofs << "    z_nis_ok_max: 4.5\n";
        ofs << "    z_nis_reject_max: 8.5\n";
        ofs << "    dvl_accept_ratio_degraded: 0.65\n";
        ofs << "    dvl_accept_ratio_invalid: 0.35\n";
        ofs << "    z_accept_ratio_degraded: 0.55\n";
        ofs << "    z_accept_ratio_invalid: 0.25\n";
        ofs << "    stats_window_duration_s: 15.0\n";
        ofs << "    enable_stop_suggestion: false\n";
        ofs << "    enable_reloc_suggestion: false\n";
        ofs << "    enable_speed_reduce_suggestion: true\n";
    }

    nav_core::app::NavDaemonConfig cfg{};
    std::string err;
    TEST_CHECK(nav_core::app::load_nav_daemon_config_from_yaml(yaml_path.string(), cfg, &err));
    TEST_EQ(cfg.estimator.health.imu_timeout_ns, 123000000ll);
    TEST_EQ(cfg.estimator.health.dvl_timeout_ns, 456000000ll);
    TEST_CHECK(cfg.estimator.health.dvl_nis_ok_max == 7.5);
    TEST_CHECK(cfg.estimator.health.dvl_nis_reject_max == 12.5);
    TEST_CHECK(cfg.estimator.health.z_nis_ok_max == 4.5);
    TEST_CHECK(cfg.estimator.health.z_nis_reject_max == 8.5);
    TEST_CHECK(cfg.estimator.health.dvl_accept_ratio_degraded == 0.65);
    TEST_CHECK(cfg.estimator.health.dvl_accept_ratio_invalid == 0.35);
    TEST_CHECK(cfg.estimator.health.z_accept_ratio_degraded == 0.55);
    TEST_CHECK(cfg.estimator.health.z_accept_ratio_invalid == 0.25);
    TEST_CHECK(cfg.estimator.health.stats_window_duration_s == 15.0);
    TEST_CHECK(!cfg.estimator.health.enable_stop_suggestion);
    TEST_CHECK(!cfg.estimator.health.enable_reloc_suggestion);
    TEST_CHECK(cfg.estimator.health.enable_speed_reduce_suggestion);

    std::filesystem::remove(yaml_path);
    return 0;
}

} // namespace

int main()
{
    int rc = test_hex_slave_addr_is_loaded();
    if (rc != 0) {
        return rc;
    }

    rc = test_health_config_fields_are_loaded();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_nav_daemon_config] all tests passed.\n";
    return 0;
}
