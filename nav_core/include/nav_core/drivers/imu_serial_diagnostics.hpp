#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace nav_core::drivers {

enum class ImuSerialPeerKind : std::uint8_t {
    kUnknown = 0,
    kImuModbusReply,
    kVolt32Ascii,
    kLegacyWitSync,
    kOtherAscii,
    kOtherBinary,
};

const char* imu_serial_peer_kind_name(ImuSerialPeerKind kind) noexcept;

struct ImuSerialDebugSnapshot {
    std::size_t total_rx_bytes{0};
    std::size_t captured_rx_bytes{0};
    std::size_t detected_imu_reply_frames{0};
    std::size_t detected_legacy_sync_frames{0};
    bool        parseable_frame_seen{false};
    bool        capture_truncated{false};

    ImuSerialPeerKind peer_kind{ImuSerialPeerKind::kUnknown};
    std::string       summary{};
    std::string       preview_text{};
    std::string       preview_hex{};
};

class ImuSerialDiagnostics {
public:
    explicit ImuSerialDiagnostics(std::size_t max_capture_bytes = 256);

    void reset(std::uint8_t slave_addr);
    void record_rx_bytes(const std::uint8_t* data, std::size_t len);
    void mark_parseable_frame();

    ImuSerialDebugSnapshot snapshot() const;

private:
    const std::size_t max_capture_bytes_;

    mutable std::mutex      mu_{};
    std::uint8_t            slave_addr_{0x50};
    std::size_t             total_rx_bytes_{0};
    bool                    capture_truncated_{false};
    bool                    parseable_frame_seen_{false};
    std::vector<std::uint8_t> capture_{};
};

} // namespace nav_core::drivers
