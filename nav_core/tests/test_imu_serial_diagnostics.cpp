#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "nav_core/drivers/imu_serial_diagnostics.hpp"

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

std::vector<std::uint8_t> make_imu_reply_frame()
{
    std::vector<std::uint8_t> frame{0x50, 0x03, 30};
    for (std::uint8_t i = 0; i < 30; ++i) {
        frame.push_back(static_cast<std::uint8_t>(i + 1u));
    }
    const auto crc = modbus_crc16(frame.data(), frame.size());
    frame.push_back(static_cast<std::uint8_t>((crc >> 8u) & 0xffu));
    frame.push_back(static_cast<std::uint8_t>(crc & 0xffu));
    return frame;
}

std::vector<std::uint8_t> make_imu_reply_frame_lohi()
{
    std::vector<std::uint8_t> frame{0x50, 0x03, 30};
    for (std::uint8_t i = 0; i < 30; ++i) {
        frame.push_back(static_cast<std::uint8_t>(i + 1u));
    }
    const auto crc = modbus_crc16(frame.data(), frame.size());
    frame.push_back(static_cast<std::uint8_t>(crc & 0xffu));
    frame.push_back(static_cast<std::uint8_t>((crc >> 8u) & 0xffu));
    return frame;
}

int test_detect_imu_modbus_reply()
{
    nav_core::drivers::ImuSerialDiagnostics diag;
    diag.reset(0x50);

    const auto frame = make_imu_reply_frame();
    diag.record_rx_bytes(frame.data(), frame.size());

    const auto snap = diag.snapshot();
    TEST_EQ(snap.peer_kind, nav_core::drivers::ImuSerialPeerKind::kImuModbusReply);
    TEST_EQ(snap.detected_imu_reply_frames, std::size_t{1});
    TEST_CHECK(snap.preview_hex.find("50 03 1e") != std::string::npos);
    return 0;
}

int test_detect_imu_modbus_reply_lohi()
{
    nav_core::drivers::ImuSerialDiagnostics diag;
    diag.reset(0x50);

    const auto frame = make_imu_reply_frame_lohi();
    diag.record_rx_bytes(frame.data(), frame.size());

    const auto snap = diag.snapshot();
    TEST_EQ(snap.peer_kind, nav_core::drivers::ImuSerialPeerKind::kImuModbusReply);
    TEST_EQ(snap.detected_imu_reply_frames, std::size_t{1});
    TEST_CHECK(snap.preview_hex.find("50 03 1e") != std::string::npos);
    return 0;
}

int test_detect_volt32_text_line()
{
    nav_core::drivers::ImuSerialDiagnostics diag;
    diag.reset(0x50);

    const char* sample = "CH0: 12.45V\r\nCH1: 0.83A\r\n";
    diag.record_rx_bytes(reinterpret_cast<const std::uint8_t*>(sample), std::strlen(sample));

    const auto snap = diag.snapshot();
    TEST_EQ(snap.peer_kind, nav_core::drivers::ImuSerialPeerKind::kVolt32Ascii);
    TEST_CHECK(snap.preview_text.find("CH0: 12.45V") != std::string::npos);
    TEST_CHECK(snap.summary.find("Volt32") != std::string::npos);
    return 0;
}

} // namespace

int main()
{
    int rc = test_detect_imu_modbus_reply();
    if (rc != 0) {
        return rc;
    }
    rc = test_detect_imu_modbus_reply_lohi();
    if (rc != 0) {
        return rc;
    }
    rc = test_detect_volt32_text_line();
    if (rc != 0) {
        return rc;
    }

    std::cout << "[test_imu_serial_diagnostics] all tests passed.\n";
    return 0;
}
