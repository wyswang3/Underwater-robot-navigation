#include "nav_core/app/imu_port_selector.hpp"

#include <algorithm>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <sys/select.h>
#include <unistd.h>

#include "nav_core/drivers/imu_modbus_probe.hpp"
#include "nav_core/drivers/imu_serial_diagnostics.hpp"
#include "nav_core/drivers/serial_port_utils.hpp"

namespace nav_core::app {
namespace {

constexpr int           kSelectSliceMs     = 20;

std::string lowercase_copy(std::string value)
{
    std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return value;
}

bool path_matches(const SerialPortIdentity& device, const std::string& path) noexcept
{
    if (path.empty()) {
        return false;
    }
    return device.path == path || device.canonical_path == path;
}

int soft_binding_score(const SerialPortIdentity&         device,
                       const drivers::SerialBindingConfig& cfg,
                       const std::string&                 preferred_path)
{
    int score = 0;
    if (path_matches(device, preferred_path)) {
        score += 1000;
    }

    for (std::size_t i = 0; i < cfg.candidate_paths.size(); ++i) {
        if (path_matches(device, cfg.candidate_paths[i])) {
            score += static_cast<int>(800 - std::min<std::size_t>(i, 200u));
            break;
        }
    }

    if (!cfg.expected_by_id_substring.empty() &&
        (device.path.find(cfg.expected_by_id_substring) != std::string::npos ||
         device.canonical_path.find(cfg.expected_by_id_substring) != std::string::npos)) {
        score += 200;
    }
    if (!cfg.expected_vid.empty() &&
        lowercase_copy(device.vendor_id) == lowercase_copy(cfg.expected_vid)) {
        score += 120;
    }
    if (!cfg.expected_pid.empty() &&
        lowercase_copy(device.product_id) == lowercase_copy(cfg.expected_pid)) {
        score += 120;
    }
    if (!cfg.expected_serial.empty() &&
        device.serial.find(cfg.expected_serial) != std::string::npos) {
        score += 160;
    }

    return score;
}

std::vector<SerialPortIdentity> rank_devices(const std::vector<SerialPortIdentity>& devices,
                                             const drivers::SerialBindingConfig& cfg,
                                             const std::string& preferred_path)
{
    std::vector<SerialPortIdentity> ordered = devices;
    std::stable_sort(ordered.begin(), ordered.end(), [&](const auto& lhs, const auto& rhs) {
        const int lhs_score = soft_binding_score(lhs, cfg, preferred_path);
        const int rhs_score = soft_binding_score(rhs, cfg, preferred_path);
        if (lhs_score != rhs_score) {
            return lhs_score > rhs_score;
        }
        return lhs.path < rhs.path;
    });
    return ordered;
}

void capture_serial_window(int fd,
                           int observe_ms,
                           drivers::ImuSerialDiagnostics& diag)
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

drivers::ImuSerialDebugSnapshot make_open_error_snapshot(const std::string& error)
{
    drivers::ImuSerialDebugSnapshot out{};
    out.summary = error;
    return out;
}

drivers::ImuSerialDebugSnapshot passive_probe_candidate(const SerialPortIdentity&   device,
                                                        const drivers::ImuConfig&   imu_cfg,
                                                        ImuPortSelectionOptions     options)
{
    std::string error;
    const int fd = drivers::open_serial_port_raw(device.path,
                                                 options.passive_baud,
                                                 /*read_timeout_ds=*/1,
                                                 &error);
    if (fd < 0) {
        return make_open_error_snapshot(error);
    }

    drivers::ImuSerialDiagnostics diag;
    diag.reset(imu_cfg.slave_addr);
    capture_serial_window(fd, options.passive_observe_ms, diag);
    ::close(fd);
    return diag.snapshot();
}

drivers::ImuSerialDebugSnapshot active_probe_candidate(const SerialPortIdentity& device,
                                                       const drivers::ImuConfig& imu_cfg,
                                                       ImuPortSelectionOptions   options)
{
    drivers::ImuModbusProbeOptions probe_options{};
    probe_options.baud = imu_cfg.baud;
    probe_options.reply_timeout_ms = options.active_reply_timeout_ms;
    probe_options.attempts = options.active_probe_attempts;
    probe_options.max_capture_bytes = 512u;

    const auto probe = drivers::run_imu_modbus_probe(
        device.path,
        drivers::ImuModbusProbeRequest{imu_cfg.slave_addr, 0x0034u, 15u},
        probe_options);
    if (!probe.error.empty()) {
        return make_open_error_snapshot(probe.error);
    }
    return probe.snapshot;
}

std::string shorten_preview(const std::string& text, std::size_t max_len = 48u)
{
    if (text.size() <= max_len) {
        return text;
    }
    return text.substr(0, max_len) + "...";
}

void append_probe_preview(std::ostringstream& oss,
                          const drivers::ImuSerialDebugSnapshot& snapshot)
{
    oss << "(rx=" << snapshot.total_rx_bytes;
    if (!snapshot.preview_text.empty()) {
        oss << ",text=" << shorten_preview(snapshot.preview_text);
    } else if (!snapshot.preview_hex.empty()) {
        oss << ",hex=" << shorten_preview(snapshot.preview_hex, 96u);
    } else if (!snapshot.summary.empty()) {
        oss << ",summary=" << shorten_preview(snapshot.summary, 96u);
    }
    oss << ")";
}

std::string summarize_probe(const std::string& path,
                            const drivers::ImuSerialDebugSnapshot& passive,
                            const drivers::ImuSerialDebugSnapshot& active)
{
    std::ostringstream oss;
    oss << path << " passive=" << drivers::imu_serial_peer_kind_name(passive.peer_kind);
    append_probe_preview(oss, passive);
    if (active.total_rx_bytes > 0 || !active.summary.empty()) {
        oss << " active=" << drivers::imu_serial_peer_kind_name(active.peer_kind);
        append_probe_preview(oss, active);
    }
    return oss.str();
}

bool is_selected_imu(const drivers::ImuSerialDebugSnapshot& snapshot) noexcept
{
    return snapshot.peer_kind == drivers::ImuSerialPeerKind::kImuModbusReply;
}

} // namespace

DeviceProbeDecision select_imu_device_port(const std::vector<SerialPortIdentity>& devices,
                                           const drivers::ImuConfig&              imu_cfg,
                                           const drivers::SerialBindingConfig&    binding_cfg,
                                           const std::string&                     preferred_path,
                                           ImuPortSelectionOptions                options)
{
    if (devices.empty()) {
        return DeviceProbeDecision{
            DeviceConnectionState::DISCONNECTED,
            std::nullopt,
            "no candidate serial device found",
        };
    }

    std::vector<std::string> rejection_notes;
    const auto ordered_devices = rank_devices(devices, binding_cfg, preferred_path);

    for (const auto& device : ordered_devices) {
        const auto passive = passive_probe_candidate(device, imu_cfg, options);
        if (is_selected_imu(passive)) {
            return DeviceProbeDecision{
                DeviceConnectionState::CONNECTING,
                device,
                "selected IMU port from passive WIT Modbus reply",
            };
        }

        if (passive.peer_kind == drivers::ImuSerialPeerKind::kVolt32Ascii) {
            rejection_notes.push_back(summarize_probe(device.path, passive, {}));
            continue;
        }

        const auto active = active_probe_candidate(device, imu_cfg, options);
        if (is_selected_imu(active)) {
            return DeviceProbeDecision{
                DeviceConnectionState::CONNECTING,
                device,
                "selected IMU port from active Modbus probe",
            };
        }

        rejection_notes.push_back(summarize_probe(device.path, passive, active));
    }

    std::ostringstream oss;
    oss << "IMU auto probe rejected " << ordered_devices.size() << " serial candidate(s)";
    if (!rejection_notes.empty()) {
        oss << ": ";
        for (std::size_t i = 0; i < rejection_notes.size(); ++i) {
            if (i != 0u) {
                oss << "; ";
            }
            oss << rejection_notes[i];
        }
    }

    return DeviceProbeDecision{
        DeviceConnectionState::MISMATCH,
        std::nullopt,
        oss.str(),
    };
}

DeviceBinder::SelectFn make_imu_port_selector(const drivers::ImuConfig& imu_cfg,
                                              ImuPortSelectionOptions   options)
{
    return [imu_cfg, options](const std::vector<SerialPortIdentity>& devices,
                              const drivers::SerialBindingConfig&    binding_cfg,
                              const std::string&                     preferred_path) {
        return select_imu_device_port(devices, imu_cfg, binding_cfg, preferred_path, options);
    };
}

} // namespace nav_core::app
