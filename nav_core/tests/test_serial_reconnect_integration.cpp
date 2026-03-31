#include <chrono>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include "nav_core/app/device_binding.hpp"
#include "nav_core/app/nav_runtime_status.hpp"
#include "nav_core/drivers/dvl_driver.hpp"
#include "nav_core/drivers/imu_driver_wit.hpp"
#include "test_pty_utils.hpp"

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
using nav_core::app::NavPublishContext;
using nav_core::app::SerialPortIdentity;
using nav_core::drivers::DvlConfig;
using nav_core::drivers::DvlDriver;
using nav_core::drivers::ImuConfig;
using nav_core::drivers::ImuDriverWit;
using nav_core::drivers::SerialBindingConfig;
using test_support::PtyPeer;
using test_support::wait_until;

SerialPortIdentity make_identity(const std::string& path,
                                 const std::string& serial)
{
    SerialPortIdentity out{};
    out.path = path;
    out.canonical_path = path;
    out.vendor_id = "10c4";
    out.product_id = "ea60";
    out.serial = serial;
    return out;
}

shared::msg::NavState make_nav_for_device_state(DeviceConnectionState state)
{
    NavPublishContext ctx{};
    ctx.publish_mono_ns = 2'000'000'000ll;
    ctx.imu_enabled = true;
    ctx.dvl_enabled = false;
    ctx.imu_device_state = state;
    ctx.dvl_device_state = DeviceConnectionState::DISCONNECTED;

    shared::msg::NavState nav{};
    nav.t_ns = 0;
    nav_core::app::apply_nav_publish_semantics(ctx, false, nav);
    return nav;
}

int test_imu_reconnect_sequence_with_real_pty()
{
    auto wrong = PtyPeer::create();
    auto first = PtyPeer::create();
    auto second = PtyPeer::create();
    TEST_CHECK(wrong.has_value());
    TEST_CHECK(first.has_value());
    TEST_CHECK(second.has_value());

    SerialBindingConfig bind_cfg{};
    bind_cfg.expected_serial = "imu-good";
    bind_cfg.reconnect_backoff_ms = 10;

    int phase = 0;
    DeviceBinder binder(
        "imu",
        "",
        bind_cfg,
        [&]() {
            if (phase == 0) {
                return std::vector<SerialPortIdentity>{};
            }
            if (phase == 1) {
                return std::vector<SerialPortIdentity>{
                    make_identity(wrong->slave_path, "imu-wrong"),
                };
            }
            if (phase == 2) {
                return std::vector<SerialPortIdentity>{
                    make_identity(first->slave_path, "imu-good"),
                };
            }
            return std::vector<SerialPortIdentity>{
                make_identity(second->slave_path, "imu-good"),
            };
        });

    phase = 0;
    auto device = binder.probe(1'000'000'000ll);
    TEST_CHECK(!device.has_value());
    TEST_EQ(binder.status().state, DeviceConnectionState::DISCONNECTED);
    {
        const auto nav = make_nav_for_device_state(binder.status().state);
        TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuDeviceNotFound);
    }

    phase = 1;
    device = binder.probe(1'100'000'000ll);
    TEST_CHECK(!device.has_value());
    TEST_EQ(binder.status().state, DeviceConnectionState::MISMATCH);
    {
        const auto nav = make_nav_for_device_state(binder.status().state);
        TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuDeviceMismatch);
        TEST_CHECK(shared::msg::nav_flag_has(
            nav.status_flags, shared::msg::NAV_FLAG_IMU_BIND_MISMATCH));
    }

    phase = 2;
    device = binder.probe(1'200'000'000ll);
    TEST_CHECK(device.has_value());
    TEST_EQ(device->path, first->slave_path);

    ImuDriverWit imu;
    ImuConfig imu_cfg{};
    imu_cfg.port = device->path;
    imu_cfg.baud = 115200;
    TEST_CHECK(imu.init(imu_cfg, [](const nav_core::ImuFrame&) {}, nullptr));
    TEST_CHECK(imu.start());
    const auto first_tx = first->read_tx_bytes(std::chrono::milliseconds(500));
    TEST_CHECK(first_tx.size() >= 8);
    TEST_EQ(first_tx[0], 0x50);
    TEST_EQ(first_tx[1], 0x03);
    TEST_EQ(first_tx[2], 0x00);
    TEST_EQ(first_tx[3], 0x34);
    TEST_EQ(first_tx[4], 0x00);
    TEST_EQ(first_tx[5], 0x0f);

    binder.mark_connect_success(1'210'000'000ll, *device);
    TEST_EQ(binder.status().state, DeviceConnectionState::ONLINE);
    {
        const auto nav = make_nav_for_device_state(binder.status().state);
        TEST_CHECK(shared::msg::nav_flag_has(
            nav.status_flags, shared::msg::NAV_FLAG_IMU_DEVICE_ONLINE));
    }

    first->close_master();
    TEST_CHECK(wait_until(std::chrono::milliseconds(800), [&] { return !imu.isPortOpen(); }));
    binder.mark_disconnected(1'300'000'000ll, "pty closed");
    TEST_EQ(binder.status().state, DeviceConnectionState::RECONNECTING);
    {
        const auto nav = make_nav_for_device_state(binder.status().state);
        TEST_EQ(nav.fault_code, shared::msg::NavFaultCode::kImuDisconnected);
        TEST_CHECK(shared::msg::nav_flag_has(
            nav.status_flags, shared::msg::NAV_FLAG_IMU_RECONNECTING));
    }

    imu.stop();

    phase = 3;
    device = binder.probe(1'320'000'000ll);
    TEST_CHECK(device.has_value());
    TEST_EQ(device->path, second->slave_path);

    imu_cfg.port = device->path;
    TEST_CHECK(imu.init(imu_cfg, [](const nav_core::ImuFrame&) {}, nullptr));
    TEST_CHECK(imu.start());
    const auto second_tx = second->read_tx_bytes(std::chrono::milliseconds(500));
    TEST_CHECK(second_tx.size() >= 8);
    TEST_EQ(second_tx[0], 0x50);
    TEST_EQ(second_tx[1], 0x03);
    TEST_EQ(second_tx[2], 0x00);
    TEST_EQ(second_tx[3], 0x34);
    TEST_EQ(second_tx[4], 0x00);
    TEST_EQ(second_tx[5], 0x0f);
    binder.mark_connect_success(1'330'000'000ll, *device);

    TEST_EQ(binder.status().state, DeviceConnectionState::ONLINE);
    TEST_EQ(binder.status().active_path, second->slave_path);
    TEST_EQ(imu.lastFrameMonoNs(), 0ll);
    TEST_CHECK(!imu.hasEverReceivedFrame());
    imu.stop();
    return 0;
}

int test_dvl_driver_pty_sample_and_disconnect()
{
    auto first = PtyPeer::create();
    auto second = PtyPeer::create();
    TEST_CHECK(first.has_value());
    TEST_CHECK(second.has_value());

    std::mutex sample_mu;
    std::vector<nav_core::drivers::DvlRawData> samples;

    auto on_raw = [&](const nav_core::drivers::DvlRawData& raw) {
        std::lock_guard<std::mutex> lock(sample_mu);
        samples.push_back(raw);
    };

    DvlDriver dvl;
    DvlConfig cfg{};
    cfg.port = first->slave_path;
    cfg.baud = 115200;
    cfg.send_startup_cmds = false;
    TEST_CHECK(dvl.init(cfg, on_raw));
    TEST_CHECK(dvl.start());

    TEST_CHECK(first->write_line(":BI, +371, +10, -40, -11,A\r\n"));
    TEST_CHECK(wait_until(std::chrono::milliseconds(500), [&] {
        std::lock_guard<std::mutex> lock(sample_mu);
        return samples.size() >= 1;
    }));

    {
        std::lock_guard<std::mutex> lock(sample_mu);
        TEST_EQ(samples.front().parsed.src, std::string("BI"));
        TEST_CHECK(samples.front().parsed.bottom_lock);
    }

    const auto first_rx = dvl.lastRxMonoNs();
    TEST_CHECK(first_rx > 0);

    first->close_master();
    TEST_CHECK(wait_until(std::chrono::milliseconds(800), [&] { return !dvl.isPortOpen(); }));
    dvl.stop();

    cfg.port = second->slave_path;
    TEST_CHECK(dvl.init(cfg, on_raw));
    TEST_CHECK(dvl.start());
    TEST_CHECK(second->write_line(":BE, +100, +0, -20,A\r\n"));
    TEST_CHECK(wait_until(std::chrono::milliseconds(500), [&] {
        std::lock_guard<std::mutex> lock(sample_mu);
        return samples.size() >= 2;
    }));

    {
        std::lock_guard<std::mutex> lock(sample_mu);
        TEST_EQ(samples.back().parsed.src, std::string("BE"));
        TEST_CHECK(samples.back().recv_mono_ns >= first_rx);
    }

    second->close_master();
    TEST_CHECK(wait_until(std::chrono::milliseconds(800), [&] { return !dvl.isPortOpen(); }));
    dvl.stop();
    return 0;
}

} // namespace

int main()
{
    int rc = test_imu_reconnect_sequence_with_real_pty();
    if (rc != 0) {
        return rc;
    }
    rc = test_dvl_driver_pty_sample_and_disconnect();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_serial_reconnect_integration] all tests passed.\n";
    return 0;
}
