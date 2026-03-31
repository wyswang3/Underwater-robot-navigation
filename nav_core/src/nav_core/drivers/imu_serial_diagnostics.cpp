#include "nav_core/drivers/imu_serial_diagnostics.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <string>
#include <vector>

namespace nav_core::drivers {
namespace {

constexpr std::uint8_t kModbusReadFunction = 0x03;
constexpr std::uint8_t kImuModbusStartReg  = 0x34;
constexpr std::uint8_t kImuRegisterCount   = 15;
constexpr std::size_t  kReplyByteCount     = static_cast<std::size_t>(kImuRegisterCount) * 2;
constexpr std::size_t  kReplyFrameLen      = 3u + kReplyByteCount + 2u;

bool is_known_wit_frame_type(std::uint8_t value) noexcept
{
    return value >= 0x50 && value <= 0x5c;
}

std::string trim_copy(const std::string& text)
{
    std::size_t begin = 0;
    while (begin < text.size() && std::isspace(static_cast<unsigned char>(text[begin])) != 0) {
        ++begin;
    }

    std::size_t end = text.size();
    while (end > begin && std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
        --end;
    }
    return text.substr(begin, end - begin);
}

std::uint16_t modbus_crc16(const std::uint8_t* data, std::size_t len) noexcept
{
    std::uint16_t crc = 0xffff;
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

struct ModbusReplyScanResult {
    std::size_t              frame_count{0};
    std::vector<std::uint8_t> first_frame{};
};

ModbusReplyScanResult scan_imu_modbus_reply_frames(const std::vector<std::uint8_t>& sample,
                                                   std::uint8_t                     slave_addr)
{
    ModbusReplyScanResult result{};
    if (sample.size() < kReplyFrameLen) {
        return result;
    }

    const std::size_t limit = sample.size() - kReplyFrameLen + 1u;
    for (std::size_t i = 0; i < limit; ++i) {
        if (sample[i] != slave_addr ||
            sample[i + 1] != kModbusReadFunction ||
            sample[i + 2] != kReplyByteCount) {
            continue;
        }

        const auto* frame_data = sample.data() + i;
        const auto  crc_expect = static_cast<std::uint16_t>(
            frame_data[kReplyFrameLen - 2] |
            (static_cast<std::uint16_t>(frame_data[kReplyFrameLen - 1]) << 8u));
        const auto crc_actual = modbus_crc16(frame_data, kReplyFrameLen - 2u);
        if (crc_actual != crc_expect) {
            continue;
        }

        ++result.frame_count;
        if (result.first_frame.empty()) {
            result.first_frame.assign(frame_data, frame_data + kReplyFrameLen);
        }
    }
    return result;
}

std::size_t count_legacy_wit_sync_frames(const std::vector<std::uint8_t>& sample) noexcept
{
    std::size_t valid_frames = 0;
    if (sample.size() < 11u) {
        return 0;
    }

    for (std::size_t i = 0; i + 10u < sample.size(); ++i) {
        if (sample[i] != 0x55u || !is_known_wit_frame_type(sample[i + 1u])) {
            continue;
        }

        std::uint8_t checksum = 0;
        for (std::size_t j = 0; j < 10u; ++j) {
            checksum = static_cast<std::uint8_t>(checksum + sample[i + j]);
        }
        if (checksum == sample[i + 10u]) {
            ++valid_frames;
        }
    }
    return valid_frames;
}

std::string format_hex_line(const std::uint8_t* data, std::size_t len)
{
    if (data == nullptr || len == 0u) {
        return {};
    }

    std::string out;
    out.reserve(len * 3u);
    char buf[4];
    for (std::size_t i = 0; i < len; ++i) {
        if (!out.empty()) {
            out.push_back(' ');
        }
        std::snprintf(buf, sizeof(buf), "%02x", static_cast<unsigned>(data[i]));
        out.append(buf);
    }
    return out;
}

std::string format_hex_preview(const std::vector<std::uint8_t>& sample, std::size_t max_bytes = 32u)
{
    const std::size_t count = std::min(max_bytes, sample.size());
    return format_hex_line(sample.data(), count);
}

std::string extract_first_ascii_line(const std::vector<std::uint8_t>& sample)
{
    std::string current;
    for (const auto byte : sample) {
        if (byte == '\r' || byte == '\n') {
            const auto trimmed = trim_copy(current);
            if (trimmed.size() >= 4u) {
                return trimmed;
            }
            current.clear();
            continue;
        }

        const auto ch = static_cast<unsigned char>(byte);
        if (std::isprint(ch) != 0 || std::isspace(ch) != 0) {
            current.push_back(static_cast<char>(byte));
        } else if (!current.empty()) {
            current.push_back(' ');
        }
    }

    const auto trimmed = trim_copy(current);
    return trimmed.size() >= 4u ? trimmed : std::string{};
}

bool starts_with_channel_tag(const std::string& line)
{
    if (line.size() < 4u || (line[0] != 'C' && line[0] != 'c') || (line[1] != 'H' && line[1] != 'h')) {
        return false;
    }

    std::size_t i = 2u;
    while (i < line.size() && std::isdigit(static_cast<unsigned char>(line[i])) != 0) {
        ++i;
    }
    return i > 2u && i < line.size() && line[i] == ':';
}

std::string extract_first_channel_line(const std::vector<std::uint8_t>& sample)
{
    std::string current;
    for (const auto byte : sample) {
        if (byte == '\r' || byte == '\n') {
            const auto trimmed = trim_copy(current);
            if (starts_with_channel_tag(trimmed)) {
                return trimmed;
            }
            current.clear();
            continue;
        }

        const auto ch = static_cast<unsigned char>(byte);
        if (std::isprint(ch) != 0 || std::isspace(ch) != 0) {
            current.push_back(static_cast<char>(byte));
        } else if (!current.empty()) {
            current.push_back(' ');
        }
    }

    const auto trimmed = trim_copy(current);
    return starts_with_channel_tag(trimmed) ? trimmed : std::string{};
}

} // namespace

const char* imu_serial_peer_kind_name(ImuSerialPeerKind kind) noexcept
{
    switch (kind) {
    case ImuSerialPeerKind::kImuModbusReply:
        return "imu_modbus_reply";
    case ImuSerialPeerKind::kVolt32Ascii:
        return "volt32_ascii";
    case ImuSerialPeerKind::kLegacyWitSync:
        return "legacy_wit_sync";
    case ImuSerialPeerKind::kOtherAscii:
        return "other_ascii";
    case ImuSerialPeerKind::kOtherBinary:
        return "other_binary";
    case ImuSerialPeerKind::kUnknown:
    default:
        return "unknown";
    }
}

ImuSerialDiagnostics::ImuSerialDiagnostics(std::size_t max_capture_bytes)
    : max_capture_bytes_(max_capture_bytes > 0u ? max_capture_bytes : 256u)
{
    capture_.reserve(max_capture_bytes_);
}

void ImuSerialDiagnostics::reset(std::uint8_t slave_addr)
{
    std::lock_guard<std::mutex> lock(mu_);
    slave_addr_ = slave_addr;
    total_rx_bytes_ = 0u;
    capture_truncated_ = false;
    parseable_frame_seen_ = false;
    capture_.clear();
}

void ImuSerialDiagnostics::record_rx_bytes(const std::uint8_t* data, std::size_t len)
{
    if (data == nullptr || len == 0u) {
        return;
    }

    std::lock_guard<std::mutex> lock(mu_);
    total_rx_bytes_ += len;

    const std::size_t remaining = max_capture_bytes_ > capture_.size()
        ? (max_capture_bytes_ - capture_.size())
        : 0u;
    const std::size_t copy_len = std::min(remaining, len);
    capture_.insert(capture_.end(), data, data + copy_len);
    if (copy_len < len) {
        capture_truncated_ = true;
    }
}

void ImuSerialDiagnostics::mark_parseable_frame()
{
    std::lock_guard<std::mutex> lock(mu_);
    parseable_frame_seen_ = true;
}

ImuSerialDebugSnapshot ImuSerialDiagnostics::snapshot() const
{
    ImuSerialDebugSnapshot out{};
    std::uint8_t slave_addr = 0x50;
    std::vector<std::uint8_t> capture;

    {
        std::lock_guard<std::mutex> lock(mu_);
        out.total_rx_bytes = total_rx_bytes_;
        out.captured_rx_bytes = capture_.size();
        out.parseable_frame_seen = parseable_frame_seen_;
        out.capture_truncated = capture_truncated_;
        slave_addr = slave_addr_;
        capture = capture_;
    }

    const auto modbus = scan_imu_modbus_reply_frames(capture, slave_addr);
    out.detected_imu_reply_frames = modbus.frame_count;
    out.detected_legacy_sync_frames = count_legacy_wit_sync_frames(capture);

    if (out.total_rx_bytes == 0u) {
        out.peer_kind = ImuSerialPeerKind::kUnknown;
        out.summary = "no RX bytes captured before timeout";
        return out;
    }

    if (modbus.frame_count > 0u) {
        out.peer_kind = ImuSerialPeerKind::kImuModbusReply;
        out.summary = out.parseable_frame_seen
            ? "WIT Modbus reply observed and SDK callback already fired"
            : "WIT-style Modbus reply observed, but SDK did not emit a register update";
        out.preview_hex = format_hex_line(modbus.first_frame.data(), modbus.first_frame.size());
        return out;
    }

    const auto volt_line = extract_first_channel_line(capture);
    if (!volt_line.empty()) {
        out.peer_kind = ImuSerialPeerKind::kVolt32Ascii;
        out.summary = "Volt32-style CHn text observed; IMU path may have rebound to the voltage sensor";
        out.preview_text = volt_line;
        out.preview_hex = format_hex_preview(capture);
        return out;
    }

    if (out.detected_legacy_sync_frames > 0u) {
        out.peer_kind = ImuSerialPeerKind::kLegacyWitSync;
        out.summary = "legacy WIT 0x55 sync frames observed; device is not speaking the expected Modbus polling reply";
        out.preview_hex = format_hex_preview(capture);
        return out;
    }

    const auto ascii_line = extract_first_ascii_line(capture);
    if (!ascii_line.empty()) {
        out.peer_kind = ImuSerialPeerKind::kOtherAscii;
        out.summary = "ASCII serial text observed, but it does not match WIT Modbus reply grammar";
        out.preview_text = ascii_line;
        out.preview_hex = format_hex_preview(capture);
        return out;
    }

    out.peer_kind = ImuSerialPeerKind::kOtherBinary;
    out.summary = "binary bytes observed, but no known WIT Modbus reply matched";
    out.preview_hex = format_hex_preview(capture);
    return out;
}

} // namespace nav_core::drivers
