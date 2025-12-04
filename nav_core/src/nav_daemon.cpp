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
#include <cerrno>

#ifdef _WIN32
#include <direct.h>
#endif

// 业务相关头文件（通过 nav_core 前缀暴露，封装了 IMU、DVL、ESKF 等功能）
#include "nav_core/imu_driver_wit.hpp"      // IMU 驱动（维特 HWT9073-485）
#include "nav_core/types.hpp"               // 包含 ImuFrame / ImuFilterConfig / DvlFilterConfig 等结构体定义
#include "nav_core/dvl_driver.hpp"          // DVL 驱动及 DvlFrame 定义
#include "nav_core/eskf.hpp"                // ESKF 状态估计类
#include "nav_core/bin_logger.hpp"          // 二进制日志写入类
#include "nav_core/imu_rt_filter.hpp"       // 实时 IMU 滤波器类，包含航向积分功能
#include <nav_core/log_packets.hpp>         // 定义日志包格式，如 ImuLogPacket / DvlLogPacket / EskfLogPacket

// 导航状态消息与发布接口
#include <shared/msg/nav_state.hpp>  // 导航状态消息定义（命名空间 shared::msg）
#include "nav_core/nav_state_publisher.hpp" // 共享内存发布器类

namespace {

// 导航核心类的别名，简化代码中引用
using nav_core::Eskf;
using nav_core::EskfConfig;
using nav_core::EskfState;
using nav_core::BinLogger;
using nav_core::ImuFrame;
using nav_core::DvlFrame;
using nav_core::ImuFilterConfig;
using nav_core::DvlFilterConfig;
using nav_core::RealTimeImuFilterCpp;

// 导航共享消息的命名空间，确保使用正确的命名空间
using shared::msg::NavState;
using shared::msg::NavHealth;
using shared::msg::NavStatusFlags;

constexpr double G_STD       = 9.80665;         // 标准重力加速度（m/s²）
constexpr double PI_STD      = 3.14159265358979323846; // 圆周率
constexpr double RAD2DEG     = 180.0 / PI_STD;  // 弧度转角度系数
constexpr double DEG2RAD     = PI_STD / 180.0;  // 角度转弧度系数
constexpr double YAW_STD_DEG = 5.0;              // 航向标准偏差（角度）
constexpr double YAW_STD_RAD = YAW_STD_DEG * DEG2RAD; // 航向标准偏差（弧度）

// Ctrl+C / SIGTERM 停止标志
std::atomic<bool> g_stop{false};

// 最近一次 DVL 是否有效（用于 NavState 标志位）
std::atomic<bool> g_last_dvl_valid{false};

// 信号处理函数：在接收到 Ctrl+C 或 SIGTERM 信号时设置停止标志
void signal_handler(int)
{
    g_stop.store(true);
}

// 简单确保目录存在（只管单级目录，例如 "./logs"、"./data"）
bool ensure_dir(const std::string& path)
{
    if (path.empty()) {
        return false;
    }

    struct stat st{};
    if (stat(path.c_str(), &st) == 0) {
        return (st.st_mode & S_IFDIR) != 0;  // 已存在且是目录
    }

#ifdef _WIN32
    const int ret = _mkdir(path.c_str());
#else
    const int ret = mkdir(path.c_str(), 0755);
#endif

    if (ret != 0) {
        std::cerr << "[navd] mkdir('" << path << "') failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }
    return true;
}

// 去掉路径末尾的 '/' 或 '\'
std::string rstrip_slash(const std::string& path)
{
    if (path.empty()) {
        return path;
    }
    std::string p = path;
    while (!p.empty() && (p.back() == '/' || p.back() == '\\')) {
        p.pop_back();
    }
    return p;
}

// 获取今天日期字符串：YYYY-MM-DD，用于日志目录分卷
std::string today_date()
{
    using namespace std::chrono;
    const auto now = system_clock::now();
    const std::time_t tt = system_clock::to_time_t(now);

    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%d", &tm);
    return std::string(buf);
}

// 命令行使用说明
void print_usage(const char* prog)
{
    std::cout << "Usage: " << prog << " [options]\n"
              << "  --imu-port <path>      default: /dev/ttyUSB0\n"
              << "  --imu-baud <baud>      default: 230400\n"
              << "  --imu-addr <hex>       default: 0x50\n"
              << "  --dvl-port <path>      default: /dev/ttyUSB1\n"
              << "  --dvl-baud <baud>      default: 115200\n"
              << "  --log-dir <dir>        default: ./logs\n"
              << "  -h, --help             show this help\n";
}

// 将 ESKF 状态转换为 NavState（控制进程订阅的核心消息）
NavState make_nav_state_from_eskf(const EskfState& st,
                                  std::uint16_t status_flags)
{
    NavState s{};

    const std::int64_t t_ns = (st.est_ns != 0) ? st.est_ns : st.mono_ns;
    s.t_ns = t_ns;

    // 位置 / 速度 / 姿态
    for (int i = 0; i < 3; ++i) {
        s.pos[i] = st.pos[i];
        s.vel[i] = st.vel[i];
        s.rpy[i] = st.euler[i];
    }

    s.ang_vel[0] = 0.0f;
    s.ang_vel[1] = 0.0f;
    s.ang_vel[2] = 0.0f;

    s.depth = -st.pos[2];  // 假定 z 向上，depth = -z

    s.health   = st.valid ? NavHealth::OK : NavHealth::DEGRADED;
    s.status   = status_flags;
    s.reserved = 0;

    return s;
}

// 根据当前传感器状态生成 NavStatusFlags bitmask
std::uint16_t compute_status_flags(bool imu_valid,
                                   bool dvl_valid,
                                   const EskfState& st)
{
    std::uint16_t flags = 0;

    if (imu_valid) {
        flags |= shared::msg::NAV_FLAG_IMU_OK;
    }
    if (dvl_valid) {
        flags |= shared::msg::NAV_FLAG_DVL_OK;
    }
    if (st.valid) {
        flags |= shared::msg::NAV_FLAG_ESKF_OK;
    }

    return flags;
}

} // anonymous namespace

int main(int argc, char** argv)
{
    // -------------------------------------------------
    // 1. 默认参数（可被命令行覆盖）
    // -------------------------------------------------
    std::string imu_port  = "/dev/ttyUSB0";
    int         imu_baud  = 230400;
    uint8_t     imu_addr  = 0x50;        // HWT9073-485 默认从站地址

    std::string dvl_port  = "/dev/ttyUSB1";
    int         dvl_baud  = 115200;

    // log_root 作为“根目录”，实际写入会在 log_root/DATE/ 下
    std::string log_root  = "./logs";

    // -------------------------------------------------
    // 2. 命令行解析（简单版）
    // -------------------------------------------------
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--imu-port" && i + 1 < argc) {
            imu_port = argv[++i];

        } else if (arg == "--imu-baud" && i + 1 < argc) {
            imu_baud = std::atoi(argv[++i]);

        } else if (arg == "--imu-addr" && i + 1 < argc) {
            const char* p = argv[++i];
            if (std::strlen(p) > 2 && p[0] == '0' &&
                (p[1] == 'x' || p[1] == 'X')) {
                imu_addr = static_cast<uint8_t>(std::strtol(p, nullptr, 16));
            } else {
                imu_addr = static_cast<uint8_t>(std::strtol(p, nullptr, 10));
            }

        } else if (arg == "--dvl-port" && i + 1 < argc) {
            dvl_port = argv[++i];

        } else if (arg == "--dvl-baud" && i + 1 < argc) {
            dvl_baud = std::atoi(argv[++i]);

        } else if (arg == "--log-dir" && i + 1 < argc) {
            log_root = argv[++i];

        } else if (arg == "-h" || arg == "--help") {
            print_usage(argv[0]);
            return 0;

        } else {
            std::cerr << "[navd] Unknown argument: " << arg << "\n";
            print_usage(argv[0]);
            return 1;
        }
    }

    // -------------------------------------------------
    // 3. 日志根目录 / 日期子目录准备
    // -------------------------------------------------
    log_root = rstrip_slash(log_root);
    if (!ensure_dir(log_root)) {
        std::cerr << "[navd] Failed to ensure log root dir: " << log_root << "\n";
        return 1;
    }

    const std::string date_str = today_date();
    const std::string log_dir  = log_root + "/" + date_str;
    if (!ensure_dir(log_dir)) {
        std::cerr << "[navd] Failed to ensure log dir: " << log_dir << "\n";
        return 1;
    }

    // -------------------------------------------------
    // 4. 设置信号处理，支持 Ctrl+C / SIGTERM 优雅退出
    // -------------------------------------------------
    std::signal(SIGINT,  signal_handler);
    std::signal(SIGTERM, signal_handler);

    // -------------------------------------------------
    // 5. 配置 ESKF 参数（保守值，可后续从 YAML 读取）
    // -------------------------------------------------
    EskfConfig eskf_cfg;
    eskf_cfg.imu_rate_hz         = 100.0;   // IMU 采样频率（Hz）
    eskf_cfg.dvl_rate_hz         = 10.0;    // DVL 期望频率（Hz）
    eskf_cfg.gravity             = G_STD;
    eskf_cfg.accel_noise_std     = 0.15;    // m/s^2，保守估计
    eskf_cfg.gyro_noise_std      = 0.02;    // rad/s，保守估计
    eskf_cfg.dvl_vel_noise_std   = 0.02;    // m/s，保守估计
    eskf_cfg.process_noise_scale = 1.0;
    eskf_cfg.meas_noise_scale    = 1.0;

    Eskf eskf(eskf_cfg);

    // -------------------------------------------------
    // 6. 二进制日志：IMU / DVL / ESKF
    // -------------------------------------------------
    BinLogger imu_logger;
    BinLogger dvl_logger;
    BinLogger state_logger;

    const std::string imu_log_path   = log_dir + "/imu.bin";
    const std::string dvl_log_path   = log_dir + "/dvl.bin";
    const std::string state_log_path = log_dir + "/eskf.bin";

    auto open_logger = [](BinLogger& logger,
                          const std::string& path,
                          const char*        name) -> bool {
        if (!logger.open(path, false)) {
            std::cerr << "[navd] Failed to open " << name
                      << " log file: " << path << "\n";
            return false;
        }
        return true;
    };

    if (!open_logger(imu_logger,   imu_log_path,   "IMU"))   return 1;
    if (!open_logger(dvl_logger,   dvl_log_path,   "DVL"))   return 1;
    if (!open_logger(state_logger, state_log_path, "ESKF"))  return 1;

    // -------------------------------------------------
    // 7. 传感器“质量过滤”参数（IMU / DVL）
    // -------------------------------------------------
    ImuFilterConfig imu_filter_cfg;
    imu_filter_cfg.enable_accel   = true;
    imu_filter_cfg.enable_gyro    = true;
    imu_filter_cfg.enable_euler   = true;
    imu_filter_cfg.max_abs_accel  = 50.0;   // m/s^2
    imu_filter_cfg.max_abs_gyro   = 10.0;   // rad/s
    imu_filter_cfg.max_euler_abs  = 3.5;    // rad (~200 deg)

    DvlFilterConfig dvl_filter_cfg;
    dvl_filter_cfg.only_valid_flag  = true; // 只接受 A（valid）数据
    dvl_filter_cfg.require_all_axes = false;
    dvl_filter_cfg.max_abs_vel_mps  = 3.0;  // 最大水下速度 3 m/s
    dvl_filter_cfg.min_quality      = 0;    // 暂不使用，后续可扩展

    // -------------------------------------------------
    // 8. 创建 IMU 实时滤波器：用于平滑 yaw 并做低频航向观测
    // -------------------------------------------------
    RealTimeImuFilterCpp imu_rt_filter(
        100.0,  // fs
        1.0,    // cutoff
        true,   // calibrate
        5.0,    // calib_seconds
        true,   // second_order
        200.0   // clamp_gyro_dps
    );

    int yaw_decim_counter = 0;

    // -------------------------------------------------
    // 9. 导航状态发布器（共享内存）
    // -------------------------------------------------
    nav_core::NavStatePublisher nav_pub;
    nav_core::NavStatePublisherConfig pub_cfg;
    pub_cfg.enable   = true;
    pub_cfg.shm_name = "/rov_nav_state_v1"; // 控制进程需使用相同名称
    pub_cfg.shm_size = 0;                   // 使用默认 sizeof(ShmLayout)

    if (!nav_pub.init(pub_cfg)) {
        std::cerr << "[navd] NavStatePublisher init failed; continue without IPC.\n";
    }

    // -------------------------------------------------
    // 10. 回调：IMU 数据处理（EKF 预测 + yaw 观测 + 日志 + NavState 发布）
    // -------------------------------------------------
    auto on_imu_frame = [&](const ImuFrame& f) {
        eskf.handleImu(f);

        {
            const double t_sec = static_cast<double>(f.mono_ns) * 1e-9;

            std::array<double,3> acc_g{
                static_cast<double>(f.lin_acc[0]) / G_STD,
                static_cast<double>(f.lin_acc[1]) / G_STD,
                static_cast<double>(f.lin_acc[2]) / G_STD
            };

            std::array<double,3> gyro_dps{
                static_cast<double>(f.ang_vel[0]) * RAD2DEG,
                static_cast<double>(f.ang_vel[1]) * RAD2DEG,
                static_cast<double>(f.ang_vel[2]) * RAD2DEG
            };

            auto opt_out = imu_rt_filter.process(t_sec, acc_g, gyro_dps);

            if (opt_out.has_value()) {
                const nav_core::ImuFiltOutput& filt = opt_out.value();

                if (++yaw_decim_counter >= 10) {
                    yaw_decim_counter = 0;

                    const double yaw_rad = filt.yaw_deg * DEG2RAD;
                    const double R_yaw   = YAW_STD_RAD * YAW_STD_RAD;

                    eskf.handleYawObservation(f.mono_ns, yaw_rad, R_yaw);
                }
            }
        }

        nav_core::ImuLogPacket pkt{};
        pkt.mono_ns = f.mono_ns;
        pkt.est_ns  = f.est_ns;
        for (int i = 0; i < 3; ++i) {
            pkt.lin_acc[i] = f.lin_acc[i];
            pkt.ang_vel[i] = f.ang_vel[i];
            pkt.euler[i]   = f.euler[i];
        }
        pkt.temperature = f.temperature;
        pkt.valid       = static_cast<std::uint8_t>(f.valid ? 1 : 0);
        imu_logger.writePod(pkt);

        EskfState st;
        if (eskf.latestState(st) && st.valid) {
            nav_core::EskfLogPacket sp{};
            sp.mono_ns = st.mono_ns;
            sp.est_ns  = st.est_ns;
            for (int i = 0; i < 3; ++i) {
                sp.pos[i]        = st.pos[i];
                sp.vel[i]        = st.vel[i];
                sp.euler[i]      = st.euler[i];
                sp.bias_accel[i] = st.bias_accel[i];
                sp.bias_gyro[i]  = st.bias_gyro[i];
            }
            sp.valid  = static_cast<std::uint8_t>(st.valid ? 1 : 0);
            sp.status = static_cast<std::uint8_t>(st.status & 0xFF);
            state_logger.writePod(sp);

            const bool imu_ok  = f.valid;
            const bool dvl_ok  = g_last_dvl_valid.load();
            const std::uint16_t flags = compute_status_flags(imu_ok, dvl_ok, st);
            const NavState nav_state  = make_nav_state_from_eskf(st, flags);

            if (nav_pub.ok()) {
                (void)nav_pub.publish(nav_state);
            }
        }
    };

    auto on_dvl_frame = [&](const DvlFrame& f) {
        eskf.handleDvl(f);

        nav_core::DvlLogPacket pkt{};
        pkt.mono_ns = f.mono_ns;
        pkt.est_ns  = f.est_ns;
        pkt.vel[0]  = f.vel[0];
        pkt.vel[1]  = f.vel[1];
        pkt.vel[2]  = f.vel[2];
        pkt.valid   = static_cast<std::int32_t>(f.valid ? 1 : 0);
        pkt.quality = static_cast<std::int32_t>(f.quality);
        dvl_logger.writePod(pkt);

        g_last_dvl_valid.store(f.valid);
    };

    nav_core::ImuDriverWit::RawCallback imu_raw_cb;
    nav_core::DvlDriver::RawCallback    dvl_raw_cb;

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

    const bool dvl_started = dvl_driver.start();
    if (!dvl_started) {
        std::cerr << "[navd] Failed to start DVL driver on " << dvl_port
                  << " (continue with IMU only)\n";
    }

    std::cout << "[navd] Started navigation daemon\n"
              << "       IMU: " << imu_port << " @" << imu_baud
              << " addr=0x" << std::hex << static_cast<int>(imu_addr) << std::dec << "\n"
              << "       DVL: " << dvl_port << " @" << dvl_baud
              << (dvl_started ? "" : " [FAILED]") << "\n"
              << "       log root: " << log_root << "\n"
              << "       log dir : " << log_dir  << "\n";

    using namespace std::chrono_literals;
    while (!g_stop.load()) {
        std::this_thread::sleep_for(1s);
    }

    std::cout << "[navd] Stopping...\n";

    if (dvl_started) {
        dvl_driver.stop();
    }
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
