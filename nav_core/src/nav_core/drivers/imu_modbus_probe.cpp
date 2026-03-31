#include "nav_core/drivers/imu_modbus_probe.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include <sys/select.h>
#include <unistd.h>

#include "nav_core/drivers/serial_port_utils.hpp"

namespace nav_core::drivers {
namespace {

constexpr int kSelectSliceMs = 20;

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

void append_captured_bytes(const std::uint8_t*      data,
                           std::size_t              len,
                           std::vector<std::uint8_t>* capture,
                           std::size_t              max_capture_bytes)
{
    if (capture == nullptr || data == nullptr || len == 0u || max_capture_bytes == 0u) {
        return;
    }

    const std::size_t remaining = max_capture_bytes > capture->size()
        ? (max_capture_bytes - capture->size())
        : 0u;
    const std::size_t copy_len = std::min(remaining, len);
    capture->insert(capture->end(), data, data + copy_len);
}

void capture_serial_window(int                         fd,
                           int                         observe_ms,
                           ImuSerialDiagnostics&       diag,
                           std::vector<std::uint8_t>* capture,
                           std::size_t                 max_capture_bytes)
{
    if (fd < 0 || observe_ms <= 0) {
        return;
    }

    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(observe_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        const auto now = std::chrono::steady_clock::now();
        const auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(
            deadline - now);
        const auto slice_us = std::min<std::chrono::microseconds>(
            remaining_us, std::chrono::milliseconds(kSelectSliceMs));
        if (slice_us.count() <= 0) {
            break;
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        timeval tv{};
        tv.tv_sec = static_cast<long>(slice_us.count() / 1000000ll);
        tv.tv_usec = static_cast<suseconds_t>(slice_us.count() % 1000000ll);

        const int ret = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            return;
        }
        if (ret == 0 || !FD_ISSET(fd, &rfds)) {
            continue;
        }

        std::uint8_t buf[256];
        const ssize_t n = ::read(fd, buf, sizeof(buf));
        if (n > 0) {
            diag.record_rx_bytes(buf, static_cast<std::size_t>(n));
            append_captured_bytes(buf,
                                  static_cast<std::size_t>(n),
                                  capture,
                                  max_capture_bytes);
            continue;
        }
        if (n == 0) {
            return;
        }
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            return;
        }
    }
}

} // namespace

std::array<std::uint8_t, 8> build_imu_modbus_read_request(
    const ImuModbusProbeRequest& request) noexcept
{
    std::array<std::uint8_t, 8> out{
        request.slave_addr,
        0x03u,
        static_cast<std::uint8_t>((request.start_reg >> 8u) & 0xffu),
        static_cast<std::uint8_t>(request.start_reg & 0xffu),
        static_cast<std::uint8_t>((request.reg_count >> 8u) & 0xffu),
        static_cast<std::uint8_t>(request.reg_count & 0xffu),
        0u,
        0u,
    };
    const std::uint16_t crc = modbus_crc16(out.data(), out.size() - 2u);
    out[6] = static_cast<std::uint8_t>((crc >> 8u) & 0xffu);
    out[7] = static_cast<std::uint8_t>(crc & 0xffu);
    return out;
}

ImuModbusProbeResult run_imu_modbus_probe(const std::string&           path,
                                          const ImuModbusProbeRequest& request,
                                          const ImuModbusProbeOptions& options)
{
    ImuModbusProbeResult out{};
    out.request = build_imu_modbus_read_request(request);

    std::string error;
    const int fd = open_serial_port_raw(path,
                                        options.baud,
                                        options.read_timeout_ds,
                                        &error);
    if (fd < 0) {
        out.error = error;
        return out;
    }

    ImuSerialDiagnostics diag(options.max_capture_bytes);
    diag.reset(request.slave_addr);

    for (int attempt = 0; attempt < std::max(1, options.attempts); ++attempt) {
        ++out.attempts_made;

        const ssize_t wn = ::write(fd, out.request.data(), out.request.size());
        out.last_write_size = static_cast<std::ptrdiff_t>(wn);
        if (wn != static_cast<ssize_t>(out.request.size())) {
            out.error = std::string("imu modbus probe write failed: ") + std::strerror(errno);
            break;
        }

        capture_serial_window(fd,
                              options.reply_timeout_ms,
                              diag,
                              &out.captured_rx,
                              options.max_capture_bytes);
        const auto snapshot = diag.snapshot();
        if (snapshot.peer_kind == ImuSerialPeerKind::kImuModbusReply) {
            break;
        }

        if (attempt + 1 < std::max(1, options.attempts) &&
            options.inter_attempt_delay_ms > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(options.inter_attempt_delay_ms));
        }
    }

    out.snapshot = diag.snapshot();
    ::close(fd);
    return out;
}

} // namespace nav_core::drivers
