#include <cstdint>
#include <iostream>
#include <vector>

#include "nav_core/app/device_binding.hpp"

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

using nav_core::app::DeviceBinder;
using nav_core::app::DeviceConnectionState;
using nav_core::app::SerialPortIdentity;
using nav_core::drivers::SerialBindingConfig;

SerialPortIdentity make_identity(const char* path,
                                 const char* canonical,
                                 const char* vid = "",
                                 const char* pid = "",
                                 const char* serial = "")
{
    SerialPortIdentity out{};
    out.path = path;
    out.canonical_path = canonical;
    out.vendor_id = vid;
    out.product_id = pid;
    out.serial = serial;
    return out;
}

int test_binder_prefers_identity_filtered_by_id_path()
{
    SerialBindingConfig cfg{};
    cfg.expected_by_id_substring = "IMU_ABC";

    DeviceBinder binder(
        "imu",
        "/dev/serial/by-id/usb-IMU_ABC",
        cfg,
        [] {
            return std::vector<SerialPortIdentity>{
                make_identity("/dev/ttyUSB0", "/dev/ttyUSB0", "1a86", "7523", "wrong"),
                make_identity("/dev/serial/by-id/usb-IMU_ABC", "/dev/ttyUSB1", "10c4", "ea60",
                              "imu-001"),
            };
        });

    const auto device = binder.probe(1000);
    TEST_CHECK(device.has_value());
    TEST_EQ(device->path, std::string("/dev/serial/by-id/usb-IMU_ABC"));
    TEST_EQ(binder.status().state, DeviceConnectionState::CONNECTING);

    binder.mark_connect_success(1100, *device);
    TEST_EQ(binder.status().state, DeviceConnectionState::ONLINE);
    TEST_EQ(binder.status().active_identity.serial, std::string("imu-001"));
    return 0;
}

int test_binder_reports_mismatch_when_identity_filter_rejects_candidates()
{
    SerialBindingConfig cfg{};
    cfg.expected_vid = "10c4";
    cfg.expected_pid = "ea60";
    cfg.expected_serial = "imu-good";

    DeviceBinder binder(
        "imu",
        "/dev/serial/by-id/usb-target",
        cfg,
        [] {
            return std::vector<SerialPortIdentity>{
                make_identity("/dev/serial/by-id/usb-target", "/dev/ttyUSB0", "10c4", "ea60",
                              "imu-bad"),
            };
        });

    const auto device = binder.probe(2000);
    TEST_CHECK(!device.has_value());
    TEST_EQ(binder.status().state, DeviceConnectionState::MISMATCH);
    TEST_CHECK(!binder.should_probe(2400));
    TEST_CHECK(binder.should_probe(2500 + 500 * 1000000ll));
    return 0;
}

int test_binder_requires_identity_when_multiple_devices_present()
{
    SerialBindingConfig cfg{};

    DeviceBinder binder(
        "dvl",
        "",
        cfg,
        [] {
            return std::vector<SerialPortIdentity>{
                make_identity("/dev/ttyUSB0", "/dev/ttyUSB0", "0403", "6001", "a"),
                make_identity("/dev/ttyUSB1", "/dev/ttyUSB1", "0403", "6001", "b"),
            };
        });

    const auto device = binder.probe(3000);
    TEST_CHECK(!device.has_value());
    TEST_EQ(binder.status().state, DeviceConnectionState::MISMATCH);
    TEST_CHECK(binder.status().reason.find("multiple") != std::string::npos);
    return 0;
}

int test_binder_reconnects_after_disconnect_backoff()
{
    SerialBindingConfig cfg{};
    cfg.expected_serial = "imu-001";
    cfg.reconnect_backoff_ms = 20;

    int scan_count = 0;
    DeviceBinder binder(
        "imu",
        "",
        cfg,
        [&scan_count] {
            ++scan_count;
            return std::vector<SerialPortIdentity>{
                make_identity("/dev/serial/by-id/imu", "/dev/ttyUSB9", "10c4", "ea60",
                              "imu-001"),
            };
        });

    auto device = binder.probe(1'000'000'000ll);
    TEST_CHECK(device.has_value());
    binder.mark_connect_success(1'000'100'000ll, *device);
    TEST_EQ(binder.status().state, DeviceConnectionState::ONLINE);

    binder.mark_disconnected(1'100'000'000ll, "frame timeout");
    TEST_EQ(binder.status().state, DeviceConnectionState::RECONNECTING);
    TEST_CHECK(!binder.should_probe(1'110'000'000ll));
    TEST_CHECK(binder.should_probe(1'120'000'000ll));

    device = binder.probe(1'120'000'000ll);
    TEST_CHECK(device.has_value());
    TEST_EQ(scan_count, 2);
    return 0;
}

} // namespace

int main()
{
    int rc = 0;
    rc = test_binder_prefers_identity_filtered_by_id_path();
    if (rc != 0) return rc;
    rc = test_binder_reports_mismatch_when_identity_filter_rejects_candidates();
    if (rc != 0) return rc;
    rc = test_binder_requires_identity_when_multiple_devices_present();
    if (rc != 0) return rc;
    rc = test_binder_reconnects_after_disconnect_backoff();
    if (rc != 0) return rc;

    std::cout << "[test_device_binding] all tests passed.\n";
    return 0;
}
