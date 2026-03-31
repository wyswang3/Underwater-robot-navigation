#include <algorithm>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <thread>
#include <vector>

#include <unistd.h>

#include "nav_core/drivers/imu_modbus_probe.hpp"
#include "nav_core/drivers/imu_serial_diagnostics.hpp"
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

using nav_core::drivers::ImuModbusProbeOptions;
using nav_core::drivers::ImuModbusProbeRequest;
using nav_core::drivers::ImuSerialPeerKind;
using nav_core::drivers::run_imu_modbus_probe;
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

    const PtyPeer&            peer_;
    std::vector<std::uint8_t> request_{};
    std::vector<std::uint8_t> reply_{};
    std::atomic<bool>         stop_{false};
    std::thread               th_{};
};

int test_probe_returns_modbus_reply_and_raw_capture()
{
    auto imu = PtyPeer::create();
    TEST_CHECK(imu.has_value());

    ImuProbeResponder responder(*imu);
    responder.start();

    ImuModbusProbeOptions options{};
    options.baud = 230400;
    options.reply_timeout_ms = 60;
    options.attempts = 2;
    options.max_capture_bytes = 128u;

    const auto result = run_imu_modbus_probe(imu->slave_path,
                                             ImuModbusProbeRequest{0x50u, 0x0034u, 15u},
                                             options);

    responder.stop();

    TEST_CHECK(result.error.empty());
    TEST_EQ(result.attempts_made, 1);
    TEST_EQ(result.snapshot.peer_kind, ImuSerialPeerKind::kImuModbusReply);
    TEST_CHECK(result.snapshot.preview_hex.find("50 03 1e") != std::string::npos);
    TEST_CHECK(!result.captured_rx.empty());
    TEST_EQ(result.request[0], 0x50u);
    TEST_EQ(result.request[1], 0x03u);
    return 0;
}

} // namespace

int main()
{
    const int rc = test_probe_returns_modbus_reply_and_raw_capture();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_imu_modbus_probe] all tests passed.\n";
    return 0;
}
