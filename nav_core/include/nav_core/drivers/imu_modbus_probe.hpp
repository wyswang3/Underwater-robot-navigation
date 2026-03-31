// nav_core/include/nav_core/drivers/imu_modbus_probe.hpp
//
// @file  imu_modbus_probe.hpp
// @brief WIT IMU 的主动 Modbus 探测模块。
//
// 角色：
//   - 负责按给定串口参数主动发送 WIT Modbus 读寄存器请求；
//   - 在一个短观察窗口内抓取原始返回字节，并复用 IMU 串口诊断器做协议分类；
//   - 为 IMU 端口自动识别、现场串口排障工具提供同一套探测实现。
//
// 不负责：
//   - IMU 长期保活、重连状态机；
//   - 寄存器物理量转换或 ESKF 融合。
//
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "nav_core/drivers/imu_serial_diagnostics.hpp"

namespace nav_core::drivers {

enum class ModbusCrcWireOrder : std::uint8_t {
    /// CRC 高字节在前（WitMotion SDK 当前实现使用该顺序）
    kHiLo = 0,
    /// CRC 低字节在前（标准 Modbus RTU 常见顺序）
    kLoHi = 1,
};

struct ImuModbusProbeRequest {
    std::uint8_t  slave_addr{0x50};
    std::uint16_t start_reg{0x0034u};
    std::uint16_t reg_count{15u};
};

struct ImuModbusProbeOptions {
    int         baud{230400};
    int         read_timeout_ds{1};
    int         reply_timeout_ms{80};
    int         inter_attempt_delay_ms{10};
    int         attempts{2};
    std::size_t max_capture_bytes{512u};
    ModbusCrcWireOrder crc_order{ModbusCrcWireOrder::kHiLo};
    bool        try_both_crc_orders{false};
};

struct ImuModbusProbeResult {
    std::array<std::uint8_t, 8> request{};
    int                         attempts_made{0};
    std::ptrdiff_t              last_write_size{0};
    std::vector<std::uint8_t>   captured_rx{};
    ImuSerialDebugSnapshot      snapshot{};
    std::string                 error{};
};

std::array<std::uint8_t, 8> build_imu_modbus_read_request(
    const ImuModbusProbeRequest& request,
    ModbusCrcWireOrder           crc_order = ModbusCrcWireOrder::kHiLo) noexcept;

ImuModbusProbeResult run_imu_modbus_probe(
    const std::string&          path,
    const ImuModbusProbeRequest& request = {},
    const ImuModbusProbeOptions& options = {});

} // namespace nav_core::drivers
