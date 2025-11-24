#include <atomic>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

#include <sys/stat.h>
#include <sys/types.h>

#include "imu_driver_wit.h"
#include "imu_types.h"
#include "dvl_driver.h"
#include "eskf.h"
#include "bin_logger.h"

// ================ 全局停止标志（信号处理） ================

namespace {

std::atomic<bool> g_stop{false};

void signal_handler(int) {
    g_stop.store(true);
}

bool ensure_dir(const std::string& path) {
    // 简单版本：如果目录不存在则尝试创建
    struct stat st;
    if (stat(path.c_str(), &st) == 0) {
        // 已存在且是目录
        return (st.st_mode & S_IFDIR) != 0;
    }
    // 不存在则创建
    if (mkdir(path.c_str(), 0755) != 0) {
        std::perror("mkdir");
        return false;
    }
    return true;
}

// 去掉路径末尾的 '/'
std::string rstrip_slash(const std::string& path) {
    if (path.empty()) return path;
    std::string p = path;
    while (!p.empty() && (p.back() == '/' || p.back() == '\\')) {
        p.pop_back();
    }
    return p;
}

// 命令行帮助
void print_usage(const char* prog) {
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --imu-port <path>      default: /dev/ttyUSB0\n"
              << "  --imu-baud <baud>      default: 230400\n"
              << "  --imu-addr <hex>       default: 0x50\n"
              << "  --dvl-port <path>      default: /dev/ttyUSB1\n"
              << "  --dvl-baud <baud>      default: 115200\n"
              << "  --log-dir <dir>        default: ./data\n"
              << "  -h, --help             show this help\n";
}

} // anonymous namespace

// ================ 二进制日志包结构 ================

#pragma pack(push, 1)

struct ImuLogPacket {
    int64_t mono_ns;
    int64_t est_ns;
    float   lin_acc[3];
    float   ang_vel[3];
    float   euler[3];
    float   temperature;
    uint8_t valid;
    uint8_t reserved[3];
};

struct DvlLogPacket {
    int64_t mono_ns;
    int64_t est_ns;
    float   vel[3];
    int32_t valid;
    int32_t quality;
};

struct EskfLogPacket {
    int64_t mono_ns;
    int64_t est_ns;
    float   pos[3];
    float   vel[3];
    float   euler[3];
    float   bias_accel[3];
    float   bias_gyro[3];
    uint8_t valid;
    uint8_t status;
    uint8_t reserved[2];
};

#pragma pack(pop)

// ================ 主程序 ================

int main(int argc, char** argv) {
    // 默认参数（可以通过命令行覆盖）
    std::string imu_port  = "/dev/ttyUSB0";
    int         imu_baud  = 230400;
    uint8_t     imu_addr  = 0x50;        // HWT9073-485 默认从站地址

    std::string dvl_port  = "/dev/ttyUSB1";
    int         dvl_baud  = 115200;

    std::string log_dir   = "./data";

    // -------- 命令行解析（简单版） --------
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--imu-port" && i + 1 < argc) {
            imu_port = argv[++i];
        } else if (arg == "--imu-baud" && i + 1 < argc) {
            imu_baud = std::atoi(argv[++i]);
        } else if (arg == "--imu-addr" && i + 1 < argc) {
            // 支持十进制或 0x.. 十六进制
            const char* p = argv[++i];
            if (std::strlen(p) > 2 && (p[0] == '0') && (p[1] == 'x' || p[1] == 'X')) {
                imu_addr = static_cast<uint8_t>(std::strtol(p, nullptr, 16));
            } else {
                imu_addr = static_cast<uint8_t>(std::strtol(p, nullptr, 10));
            }
        } else if (arg == "--dvl-port" && i + 1 < argc) {
            dvl_port = argv[++i];
        } else if (arg == "--dvl-baud" && i + 1 < argc) {
            dvl_baud = std::atoi(argv[++i]);
        } else if (arg == "--log-dir" && i + 1 < argc) {
            log_dir = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;
        } else {
            std::cerr << "[navd] Unknown argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    log_dir = rstrip_slash(log_dir);
    if (!ensure_dir(log_dir)) {
        std::cerr << "[navd] Failed to ensure log dir: " << log_dir << "\n";
        return 1;
    }

    // -------- 设置信号处理，支持 Ctrl+C 优雅退出 --------
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // -------- 配置 ESKF 参数（保守值，可后续从 YAML 读取） --------
    nav_core::EskfConfig eskf_cfg;
    eskf_cfg.imu_rate_hz       = 100.0;
    eskf_cfg.dvl_rate_hz       = 10.0;
    eskf_cfg.gravity           = 9.80665;   // m/s^2
    eskf_cfg.accel_noise_std   = 0.15;      // m/s^2，保守估计
    eskf_cfg.gyro_noise_std    = 0.02;      // rad/s，保守估计
    eskf_cfg.dvl_vel_noise_std = 0.02;      // m/s，保守估计
    eskf_cfg.process_noise_scale = 1.0;
    eskf_cfg.meas_noise_scale    = 1.0;

    nav_core::Eskf eskf(eskf_cfg);

    // -------- 二进制日志打开 --------
    nav_core::BinLogger imu_logger;
    nav_core::BinLogger dvl_logger;
    nav_core::BinLogger state_logger;

    std::string imu_log_path   = log_dir + "/imu.bin";
    std::string dvl_log_path   = log_dir + "/dvl.bin";
    std::string state_log_path = log_dir + "/eskf.bin";

    if (!imu_logger.open(imu_log_path, false)) {
        std::cerr << "[navd] Failed to open IMU log file: " << imu_log_path << "\n";
        return 1;
    }
    if (!dvl_logger.open(dvl_log_path, false)) {
        std::cerr << "[navd] Failed to open DVL log file: " << dvl_log_path << "\n";
        return 1;
    }
    if (!state_logger.open(state_log_path, false)) {
        std::cerr << "[navd] Failed to open ESKF log file: " << state_log_path << "\n";
        return 1;
    }

    // -------- 过滤配置（保守值） --------
    nav_core::ImuFilterConfig imu_filter_cfg;
    imu_filter_cfg.enable_accel   = true;
    imu_filter_cfg.enable_gyro    = true;
    imu_filter_cfg.enable_euler   = true;
    imu_filter_cfg.max_abs_accel  = 50.0;   // m/s^2
    imu_filter_cfg.max_abs_gyro   = 10.0;   // rad/s
    imu_filter_cfg.max_euler_abs  = 3.5;    // rad (~200 deg)

    nav_core::DvlFilterConfig dvl_filter_cfg;
    dvl_filter_cfg.only_valid_flag  = true; // 只接受 A（valid）数据
    dvl_filter_cfg.require_all_axes = false;
    dvl_filter_cfg.max_abs_vel_mps  = 3.0;  // 最大水下速度 3 m/s
    dvl_filter_cfg.min_quality      = 0;    // 暂不使用，后续可扩展

    // -------- 回调：IMU 数据处理 --------
    auto on_imu_frame = [&](const nav_core::ImuFrame& f) {
        // 1) 先喂给 ESKF
        eskf.handleImu(f);

        // 2) 记录 IMU 原始数据
        ImuLogPacket pkt{};
        pkt.mono_ns = f.mono_ns;
        pkt.est_ns  = f.est_ns;
        pkt.lin_acc[0] = f.lin_acc[0];
        pkt.lin_acc[1] = f.lin_acc[1];
        pkt.lin_acc[2] = f.lin_acc[2];
        pkt.ang_vel[0] = f.ang_vel[0];
        pkt.ang_vel[1] = f.ang_vel[1];
        pkt.ang_vel[2] = f.ang_vel[2];
        pkt.euler[0]   = f.euler[0];
        pkt.euler[1]   = f.euler[1];
        pkt.euler[2]   = f.euler[2];
        pkt.temperature = f.temperature;
        pkt.valid       = static_cast<uint8_t>(f.valid ? 1 : 0);
        imu_logger.writePod(pkt);

        // 3) 记录当前 ESKF 状态（可选，频率与 IMU 一致）
        nav_core::EskfState st;
        if (eskf.latestState(st) && st.valid) {
            EskfLogPacket sp{};
            sp.mono_ns = st.mono_ns;
            sp.est_ns  = st.est_ns;
            for (int i = 0; i < 3; ++i) {
                sp.pos[i]        = st.pos[i];
                sp.vel[i]        = st.vel[i];
                sp.euler[i]      = st.euler[i];
                sp.bias_accel[i] = st.bias_accel[i];
                sp.bias_gyro[i]  = st.bias_gyro[i];
            }
            sp.valid  = static_cast<uint8_t>(st.valid ? 1 : 0);
            sp.status = static_cast<uint8_t>(st.status & 0xFF);
            state_logger.writePod(sp);
        }
    };

    // -------- 回调：DVL 数据处理 --------
    auto on_dvl_frame = [&](const nav_core::DvlFrame& f) {
        // 1) DVL 更新 ESKF（速度校正）
        eskf.handleDvl(f);

        // 2) 记录 DVL 帧
        DvlLogPacket pkt{};
        pkt.mono_ns = f.mono_ns;
        pkt.est_ns  = f.est_ns;
        pkt.vel[0]  = f.vel[0];
        pkt.vel[1]  = f.vel[1];
        pkt.vel[2]  = f.vel[2];
        pkt.valid   = static_cast<int32_t>(f.valid ? 1 : 0);
        pkt.quality = static_cast<int32_t>(f.quality);
        dvl_logger.writePod(pkt);
    };

    // 原始回调目前不做处理，可以在需要时用于调试
    nav_core::ImuDriverWit::RawCallback imu_raw_cb; // 空
    nav_core::DvlDriver::RawCallback    dvl_raw_cb; // 空

    // -------- 启动 IMU / DVL 驱动 --------
    nav_core::ImuDriverWit imu_driver(
        imu_port,
        imu_baud,
        imu_addr,
        imu_filter_cfg,
        on_imu_frame,
        imu_raw_cb
    );

    nav_core::DvlDriver dvl_driver(
        dvl_port,
        dvl_baud,
        dvl_filter_cfg,
        on_dvl_frame,
        dvl_raw_cb
    );

    if (!imu_driver.start()) {
        std::cerr << "[navd] Failed to start IMU driver on " << imu_port << "\n";
        return 1;
    }
    if (!dvl_driver.start()) {
        std::cerr << "[navd] Failed to start DVL driver on " << dvl_port << "\n";
        imu_driver.stop();
        return 1;
    }

    std::cout << "[navd] Started navigation daemon\n"
              << "       IMU: " << imu_port << " @" << imu_baud
              << " addr=0x" << std::hex << int(imu_addr) << std::dec << "\n"
              << "       DVL: " << dvl_port << " @" << dvl_baud << "\n"
              << "       log dir: " << log_dir << "\n";

    // -------- 主循环：保持进程存活，直到收到停止信号 --------
    using namespace std::chrono_literals;
    while (!g_stop.load()) {
        std::this_thread::sleep_for(1s);
        // 这里可以周期性打印状态（可选）
        // nav_core::EskfState st;
        // if (eskf.latestState(st) && st.valid) {
        //     std::cout << "[navd] pos=("
        //               << st.pos[0] << ", "
        //               << st.pos[1] << ", "
        //               << st.pos[2] << ")\n";
        // }
    }

    std::cout << "[navd] Stopping...\n";

    // -------- 清理资源 --------
    dvl_driver.stop();
    imu_driver.stop();

    imu_logger.flush();
    dvl_logger.flush();
    state_logger.flush();

    imu_logger.close();
    dvl_logger.close();
    state_logger.close();

    std::cout << "[navd] Exit.\n";
    return 0;
}
