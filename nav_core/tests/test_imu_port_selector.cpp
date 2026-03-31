#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <thread>
#include <vector>

#include <unistd.h>

#include "nav_core/app/imu_port_selector.hpp"
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

using nav_core::app::DeviceConnectionState;
using nav_core::app::ImuPortSelectionOptions;
using nav_core::app::SerialPortIdentity;
using nav_core::app::select_imu_device_port;
using nav_core::drivers::ImuConfig;
using test_support::PtyPeer;

std::uint16_t modbus_crc16(const std::uint8_t* data, std::size_t len) noexcept
{
    std::uint16_t crc = 0xffffu;
    for (std::size_t i = 0; i < len; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit) {
            if ((crc & 0x0001u) != 0u) {
                crc = static_cast<std::uint16_t>((crc >> 1u) ^ 0xa001u);
            } else {
                crc = static_cast<std::uint16_t>(crc >> 1u);
            }
        }
    }
    return crc;
}

std::vector<std::uint8_t> build_imu_reply()
{
    std::vector<std::uint8_t> out(35u, 0u);
    out[0] = 0x50u;
    out[1] = 0x03u;
    out[2] = 30u;
    const std::uint16_t crc = modbus_crc16(out.data(), out.size() - 2u);
    out[out.size() - 2u] = static_cast<std::uint8_t>((crc >> 8u) & 0xffu);
    out[out.size() - 1u] = static_cast<std::uint8_t>(crc & 0xffu);
    return out;
}

std::vector<std::uint8_t> build_imu_request()
{
    std::vector<std::uint8_t> out{0x50u, 0x03u, 0x00u, 0x34u, 0x00u, 0x0fu, 0u, 0u};
    const std::uint16_t crc = modbus_crc16(out.data(), out.size() - 2u);
    out[6] = static_cast<std::uint8_t>((crc >> 8u) & 0xffu);
    out[7] = static_cast<std::uint8_t>(crc & 0xffu);
    return out;
}

SerialPortIdentity make_identity(const std::string& path, const std::string& serial)
{
    SerialPortIdentity out{};
    out.path = path;
    out.canonical_path = path;
    out.vendor_id = "10c4";
    out.product_id = "ea60";
    out.serial = serial;
    return out;
}

class Volt32Emitter {
public:
    explicit Volt32Emitter(const PtyPeer& peer) : peer_(peer) {}

    void start()
    {
        stop_.store(false);
        th_ = std::thread([this] {
            while (!stop_.load()) {
                peer_.write_line("CH0:12.41 CH1:0.38\r\n");
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }
        });
    }

    void stop()
    {
        stop_.store(true);
        if (th_.joinable()) {
            th_.join();
        }
    }

    ~Volt32Emitter() { stop(); }

private:
    const PtyPeer&      peer_;
    std::atomic<bool>   stop_{false};
    std::thread         th_{};
};

class ImuProbeResponder {
public:
    explicit ImuProbeResponder(const PtyPeer& peer)
        : peer_(peer), request_(build_imu_request()), reply_(build_imu_reply())
    {
    }

    void start()
    {
        stop_.store(false);
        th_ = std::thread([this] { run_(); });
    }

    void stop()
    {
        stop_.store(true);
        if (th_.joinable()) {
            th_.join();
        }
    }

    ~ImuProbeResponder() { stop(); }

private:
    void run_()
    {
        std::vector<std::uint8_t> buffer;
        buffer.reserve(64u);

        while (!stop_.load()) {
            std::uint8_t chunk[64];
            const ssize_t n = ::read(peer_.master_fd, chunk, sizeof(chunk));
            if (n > 0) {
                buffer.insert(buffer.end(), chunk, chunk + n);
                const auto it = std::search(buffer.begin(), buffer.end(),
                                            request_.begin(), request_.end());
                if (it != buffer.end()) {
                    peer_.write_bytes(reply_);
                    buffer.erase(buffer.begin(), it + request_.size());
                    continue;
                }

                if (buffer.size() > request_.size()) {
                    buffer.erase(buffer.begin(),
                                 buffer.end() - static_cast<std::ptrdiff_t>(request_.size()));
                }
                continue;
            }

            if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
                return;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
    }

    const PtyPeer&              peer_;
    std::vector<std::uint8_t>   request_{};
    std::vector<std::uint8_t>   reply_{};
    std::atomic<bool>           stop_{false};
    std::thread                 th_{};
};

int test_selector_rejects_volt32_and_selects_polled_imu()
{
    auto volt = PtyPeer::create();
    auto imu = PtyPeer::create();
    TEST_CHECK(volt.has_value());
    TEST_CHECK(imu.has_value());

    Volt32Emitter emitter(*volt);
    ImuProbeResponder responder(*imu);
    emitter.start();
    responder.start();

    ImuConfig imu_cfg{};
    imu_cfg.baud = 230400;
    imu_cfg.slave_addr = 0x50;

    ImuPortSelectionOptions options{};
    options.passive_observe_ms = 90;
    options.active_reply_timeout_ms = 60;
    options.active_probe_attempts = 2;

    const auto decision = select_imu_device_port(
        std::vector<SerialPortIdentity>{
            make_identity(volt->slave_path, "volt32"),
            make_identity(imu->slave_path, "imu"),
        },
        imu_cfg,
        {},
        "",
        options);

    emitter.stop();
    responder.stop();

    TEST_EQ(decision.state, DeviceConnectionState::CONNECTING);
    TEST_CHECK(decision.selected_device.has_value());
    TEST_EQ(decision.selected_device->path, imu->slave_path);
    TEST_CHECK(decision.reason.find("active Modbus probe") != std::string::npos);
    return 0;
}

int test_selector_reports_volt32_mismatch_when_only_voltage_port_exists()
{
    auto volt = PtyPeer::create();
    TEST_CHECK(volt.has_value());

    Volt32Emitter emitter(*volt);
    emitter.start();

    ImuConfig imu_cfg{};
    imu_cfg.baud = 230400;
    imu_cfg.slave_addr = 0x50;

    ImuPortSelectionOptions options{};
    options.passive_observe_ms = 90;
    options.active_reply_timeout_ms = 40;
    options.active_probe_attempts = 1;

    const auto decision = select_imu_device_port(
        std::vector<SerialPortIdentity>{make_identity(volt->slave_path, "volt32")},
        imu_cfg,
        {},
        "",
        options);

    emitter.stop();

    TEST_EQ(decision.state, DeviceConnectionState::MISMATCH);
    TEST_CHECK(!decision.selected_device.has_value());
    TEST_CHECK(decision.reason.find("volt32_ascii") != std::string::npos);
    return 0;
}

} // namespace

int main()
{
    int rc = test_selector_rejects_volt32_and_selects_polled_imu();
    if (rc != 0) {
        return rc;
    }
    rc = test_selector_reports_volt32_mismatch_when_only_voltage_port_exists();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_imu_port_selector] all tests passed.\n";
    return 0;
}
