// nav_core/include/nav_core/drivers/imu_serial_diagnostics.hpp
//
// @file  imu_serial_diagnostics.hpp
// @brief IMU 串口原始字节诊断模块。
//
// 角色：
//   - 在“串口已打开，但 IMU 没有正常产出寄存器/帧”时，缓存一小段原始 RX 字节；
//   - 基于字节特征识别当前串口更像 WIT Modbus、Volt32 文本、旧 WIT 同步帧还是其他未知协议；
//   - 给 nav_daemon 的 stderr / nav_events.csv 提供可读的现场诊断摘要。
//
// 边界：
//   - 这是排障模块，不参与导航解算语义；
//   - 只抓少量首帧字节，不试图成为完整协议栈。
//
#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

namespace nav_core::drivers {

/// 当前串口对端在“原始字节特征”层面的粗分类结果。
enum class ImuSerialPeerKind : std::uint8_t {
    kUnknown = 0,
    kImuModbusReply,
    kVolt32Ascii,
    kLegacyWitSync,
    kOtherAscii,
    kOtherBinary,
};

const char* imu_serial_peer_kind_name(ImuSerialPeerKind kind) noexcept;

/**
 * @brief 面向日志输出的一次串口诊断快照。
 *
 * 字段含义：
 *  - 既保留统计量，也保留一条尽量短的 text/hex 预览；
 *  - 上层用它直接打印故障原因，不再需要理解底层捕获细节。
 */
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
    // 额外的“单帧 dump”（十六进制），用于现场排障。
    //
    // 约定：
    // - 若识别到 Modbus 回复：这里放完整一帧（含 CRC）；
    // - 否则放捕获缓冲的前 N 字节（N 受实现限制，避免日志爆炸）。
    std::string       frame_dump_hex{};
};

/**
 * @brief 线程安全的 IMU 串口诊断缓存器。
 *
 * 用法：
 *  - 驱动线程在 read() 后调用 `record_rx_bytes()`；
 *  - 解析出有效寄存器更新后调用 `mark_parseable_frame()`；
 *  - 上层在超时或报错时调用 `snapshot()` 取一份摘要。
 */
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
