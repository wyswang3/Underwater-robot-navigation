// nav_core/src/nav_core/app/tools/imu_modbus_probe.cpp
//
// 独立 IMU Modbus 探测工具：
//   - 对指定串口发送一帧 WIT Modbus 读寄存器请求；
//   - 打印 TX/RX 原始十六进制，便于现场确认设备真实返回；
//   - 复用 nav_core 内部诊断器，输出协议分类结果，避免和主程序逻辑漂移。

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

#include "nav_core/drivers/imu_modbus_probe.hpp"
#include "nav_core/drivers/imu_serial_diagnostics.hpp"

namespace {

struct Args {
    std::string port{"/dev/ttyUSB1"};
    int         baud{230400};
    std::uint8_t slave_addr{0x50};
    std::uint16_t start_reg{0x0034u};
    std::uint16_t reg_count{15u};
    int         attempts{3};
    int         reply_timeout_ms{120};
    std::size_t capture_limit{1024u};
    std::string output_path{};
    // 默认按标准 Modbus RTU 线序发送 CRC（low byte first）。
    std::string crc_order{"lohi"}; // hilo | lohi | both
    bool        scan_common{false};
    bool        scan_addr{false};
};

bool parse_u32_auto(const std::string& text, std::uint32_t& out)
{
    try {
        std::size_t pos = 0;
        const unsigned long value = std::stoul(text, &pos, 0);
        if (pos != text.size()) {
            return false;
        }
        out = static_cast<std::uint32_t>(value);
        return true;
    } catch (...) {
        return false;
    }
}

std::string format_hex_line(const std::uint8_t* data, std::size_t len)
{
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (std::size_t i = 0; i < len; ++i) {
        if (i != 0u) {
            oss << ' ';
        }
        oss << std::setw(2) << static_cast<unsigned>(data[i]);
    }
    return oss.str();
}

std::string format_hex_dump(const std::vector<std::uint8_t>& data)
{
    if (data.empty()) {
        return "<empty>";
    }

    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (std::size_t offset = 0; offset < data.size(); offset += 16u) {
        oss << std::setw(4) << offset << "  ";
        const std::size_t end = std::min(offset + 16u, data.size());
        for (std::size_t i = offset; i < end; ++i) {
            if (i != offset) {
                oss << ' ';
            }
            oss << std::setw(2) << static_cast<unsigned>(data[i]);
        }
        if (end < data.size()) {
            oss << '\n';
        }
    }
    return oss.str();
}

void print_usage(const char* argv0)
{
    std::cerr
        << "Usage: " << argv0 << " [options]\n"
        << "  --port <path>              default: /dev/ttyUSB1\n"
        << "  --baud <baud>              default: 230400\n"
        << "  --addr <id>                default: 0x50\n"
        << "  --start-reg <reg>          default: 0x34\n"
        << "  --reg-count <count>        default: 15\n"
        << "  --attempts <n>             default: 3\n"
        << "  --reply-timeout-ms <ms>    default: 120\n"
        << "  --capture-limit <bytes>    default: 1024\n"
        << "  --crc-order <hilo|lohi|both> default: lohi\n"
        << "  --scan-common              scan common baud/addr combos\n"
        << "  --scan-addr                scan addr 1..247 at selected baud\n"
        << "  --output <file>            optional raw RX binary output\n";
}

bool parse_args(int argc, char** argv, Args& out)
{
    for (int i = 1; i < argc; ++i) {
        const std::string key = argv[i];
        auto require_value = [&](const char* opt) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "[imu_probe][ERR] missing value for " << opt << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (key == "--port") {
            const char* value = require_value("--port");
            if (value == nullptr) return false;
            out.port = value;
            continue;
        }
        if (key == "--baud" || key == "--addr" || key == "--start-reg" ||
            key == "--reg-count" || key == "--attempts" ||
            key == "--reply-timeout-ms" || key == "--capture-limit") {
            const char* value = require_value(key.c_str());
            if (value == nullptr) return false;
            std::uint32_t parsed = 0;
            if (!parse_u32_auto(value, parsed)) {
                std::cerr << "[imu_probe][ERR] invalid value for " << key << ": " << value << "\n";
                return false;
            }
            if (key == "--baud") {
                out.baud = static_cast<int>(parsed);
            } else if (key == "--addr") {
                if (parsed > 0xffu) {
                    std::cerr << "[imu_probe][ERR] addr out of range: " << parsed << "\n";
                    return false;
                }
                out.slave_addr = static_cast<std::uint8_t>(parsed);
            } else if (key == "--start-reg") {
                if (parsed > 0xffffu) {
                    std::cerr << "[imu_probe][ERR] start-reg out of range: " << parsed << "\n";
                    return false;
                }
                out.start_reg = static_cast<std::uint16_t>(parsed);
            } else if (key == "--reg-count") {
                if (parsed == 0u || parsed > 0xffffu) {
                    std::cerr << "[imu_probe][ERR] reg-count out of range: " << parsed << "\n";
                    return false;
                }
                out.reg_count = static_cast<std::uint16_t>(parsed);
            } else if (key == "--attempts") {
                out.attempts = static_cast<int>(parsed);
            } else if (key == "--reply-timeout-ms") {
                out.reply_timeout_ms = static_cast<int>(parsed);
            } else if (key == "--capture-limit") {
                out.capture_limit = static_cast<std::size_t>(parsed);
            }
            continue;
        }
        if (key == "--output") {
            const char* value = require_value("--output");
            if (value == nullptr) return false;
            out.output_path = value;
            continue;
        }
        if (key == "--crc-order") {
            const char* value = require_value("--crc-order");
            if (value == nullptr) return false;
            out.crc_order = value;
            continue;
        }
        if (key == "--scan-common") {
            out.scan_common = true;
            continue;
        }
        if (key == "--scan-addr") {
            out.scan_addr = true;
            continue;
        }
        if (key == "-h" || key == "--help") {
            print_usage(argv[0]);
            std::exit(0);
        }

        std::cerr << "[imu_probe][ERR] unknown option: " << key << "\n";
        return false;
    }

    if (out.attempts <= 0) {
        out.attempts = 1;
    }
    if (out.reply_timeout_ms <= 0) {
        out.reply_timeout_ms = 80;
    }
    if (out.capture_limit == 0u) {
        out.capture_limit = 512u;
    }
    return true;
}

nav_core::drivers::ImuModbusProbeOptions make_options(const Args& args)
{
    nav_core::drivers::ImuModbusProbeOptions options{};
    options.baud = args.baud;
    options.reply_timeout_ms = args.reply_timeout_ms;
    options.attempts = args.attempts;
    options.max_capture_bytes = args.capture_limit;

    // "both" 模式下仍指定一个“优先顺序”，默认从标准 lo-hi 开始尝试。
    if (args.crc_order == "hilo") {
        options.crc_order = nav_core::drivers::ModbusCrcWireOrder::kHiLo;
    } else {
        options.crc_order = nav_core::drivers::ModbusCrcWireOrder::kLoHi;
    }
    options.try_both_crc_orders = (args.crc_order == "both");
    return options;
}

int run_once(const Args& args)
{
    const auto options = make_options(args);
    const auto result = nav_core::drivers::run_imu_modbus_probe(
        args.port,
        nav_core::drivers::ImuModbusProbeRequest{
            args.slave_addr,
            args.start_reg,
            args.reg_count,
        },
        options);

    std::cout << "[imu_probe] port=" << args.port
              << " baud=" << args.baud
              << " addr=0x" << std::hex << std::setw(2) << std::setfill('0')
              << static_cast<unsigned>(args.slave_addr)
              << std::dec
              << " start_reg=0x" << std::hex << std::setw(4) << std::setfill('0')
              << static_cast<unsigned>(args.start_reg)
              << std::dec
              << " reg_count=" << args.reg_count
              << " attempts=" << args.attempts
              << " crc_order=" << args.crc_order
              << "\n";
    std::cout << "[imu_probe] tx_request: "
              << format_hex_line(result.request.data(), result.request.size()) << "\n";

    if (!result.error.empty()) {
        std::cerr << "[imu_probe][ERR] " << result.error << "\n";
        return 1;
    }

    const auto& snap = result.snapshot;
    std::cout << "[imu_probe] attempts_made=" << result.attempts_made
              << " rx_bytes=" << snap.total_rx_bytes
              << " kind=" << nav_core::drivers::imu_serial_peer_kind_name(snap.peer_kind)
              << " summary=" << snap.summary << "\n";
    if (!snap.preview_text.empty()) {
        std::cout << "[imu_probe] preview_text: " << snap.preview_text << "\n";
    }
    if (!snap.preview_hex.empty()) {
        std::cout << "[imu_probe] preview_hex: " << snap.preview_hex << "\n";
    }
    std::cout << "[imu_probe] rx_dump:\n" << format_hex_dump(result.captured_rx) << "\n";

    if (!args.output_path.empty()) {
        std::ofstream out(args.output_path, std::ios::binary);
        if (!out.is_open()) {
            std::cerr << "[imu_probe][ERR] failed to open output file: "
                      << args.output_path << "\n";
            return 1;
        }
        if (!result.captured_rx.empty()) {
            out.write(reinterpret_cast<const char*>(result.captured_rx.data()),
                      static_cast<std::streamsize>(result.captured_rx.size()));
        }
        std::cout << "[imu_probe] wrote raw rx bytes to " << args.output_path << "\n";
    }

    if (snap.peer_kind == nav_core::drivers::ImuSerialPeerKind::kImuModbusReply) {
        return 0;
    }
    if (snap.total_rx_bytes == 0u) {
        std::cerr
            << "[imu_probe][HINT] rx_bytes=0 means no bytes came back at all.\n"
            << "[imu_probe][HINT] Prioritize: power stable, RS485 A/B swapped check, GND common, adapter ok.\n"
            << "[imu_probe][HINT] If Python can read on same setup, try --crc-order both and --baud 115200.\n";
    }
    return snap.total_rx_bytes > 0u ? 2 : 1;
}

} // namespace

int main(int argc, char** argv)
{
    Args args;
    if (!parse_args(argc, argv, args)) {
        print_usage(argv[0]);
        return 1;
    }

    if (!args.scan_common && !args.scan_addr) {
        return run_once(args);
    }

    if (args.scan_common) {
        const int bauds[] = {230400, 115200};
        const std::uint8_t addrs[] = {0x50u, 0x51u, 0x52u, 0x01u};
        for (const int baud : bauds) {
            for (const auto addr : addrs) {
                Args trial = args;
                trial.baud = baud;
                trial.slave_addr = addr;
                trial.attempts = 2;
                trial.reply_timeout_ms = std::max(80, args.reply_timeout_ms);
                trial.output_path.clear();
                std::cout << "\n";
                const int rc = run_once(trial);
                if (rc == 0) {
                    return 0;
                }
            }
        }
    }

    if (args.scan_addr) {
        for (std::uint32_t addr = 1; addr <= 247; ++addr) {
            Args trial = args;
            trial.slave_addr = static_cast<std::uint8_t>(addr);
            trial.attempts = 1;
            trial.reply_timeout_ms = std::max(80, args.reply_timeout_ms);
            trial.output_path.clear();
            std::cout << "\n";
            const int rc = run_once(trial);
            if (rc == 0) {
                return 0;
            }
        }
    }

    return 1;
}
