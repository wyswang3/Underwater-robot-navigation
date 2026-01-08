// nav_core/src/drivers/dvl_driver.cpp
//
// Hover H1000 DVL 串口驱动 + 文本协议解析 + 基本质量过滤
//
// 职责：
//   - 串口 IO + 后台线程：按行读取 PD6/EPD6 文本；
//   - 调用 nav_core::dvl_protocol 解析为 ParsedLine；
//   - 使用统一时间基 (nav_core::core::timebase) 打时间戳；
//   - 应用 DvlFilterConfig 做基础门控，生成 DvlFrame；
//   - 维护位移/深度最近有效值，空洞时回填；
//   - 发送 CZ / CS / PR / PM 等厂商定义命令。
//
// 不负责：
//   - ESKF / 轨迹估计 / ZUPT / 噪声地板等高层逻辑。

#include <nav_core/drivers/dvl_driver.hpp>
#include <nav_core/core/timebase.hpp>
#include <nav_core/drivers/dvl_protocol.hpp>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

// POSIX 串口
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace nav_core::drivers {

// 为时间戳取别名
namespace tb = nav_core::core::timebase;
namespace dp = nav_core::dvl_protocol;

// ============================================================================
// 匿名命名空间：小工具
// ============================================================================

namespace {

inline bool isFinite(double x) noexcept {
    return std::isfinite(x);
}

speed_t baudToTermios(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
        default:
            std::cerr << "[DVL] unsupported baud " << baud
                      << ", fallback to 115200\n";
            return B115200;
    }
}

// 去掉前后空白字符
inline std::string trim(const std::string& s) {
    std::size_t b = 0;
    while (b < s.size() &&
           std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }
    if (b == s.size()) {
        return {};
    }
    std::size_t e = s.size();
    while (e > b &&
           std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }
    return s.substr(b, e - b);
}

} // namespace

// ============================================================================
// 构造 / 析构 / init
// ============================================================================

DvlDriver::DvlDriver() = default;

DvlDriver::DvlDriver(const DvlConfig& cfg,
                     FrameCallback    on_frame,
                     RawCallback      on_raw)
{
    init(cfg, std::move(on_frame), std::move(on_raw));
}

DvlDriver::DvlDriver(const std::string&     port,
                     int                    baud,
                     const DvlFilterConfig& filter,
                     FrameCallback          on_frame,
                     RawCallback            on_raw)
{
    DvlConfig cfg;
    cfg.port   = port;
    cfg.baud   = baud;
    cfg.filter = filter;
    init(cfg, std::move(on_frame), std::move(on_raw));
}

DvlDriver::~DvlDriver() {
    stop();
}

bool DvlDriver::init(const DvlConfig& cfg,
                     FrameCallback    on_frame,
                     RawCallback      on_raw)
{
    cfg_      = cfg;
    port_     = cfg.port;
    baud_     = cfg.baud;
    on_frame_ = std::move(on_frame);
    on_raw_   = std::move(on_raw);

    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        filter_cfg_ = cfg.filter;
    }

    fd_ = -1;

    stop_requested_.store(false);
    running_.store(false);

    n_lines_.store(0);
    n_parsed_ok_.store(0);
    n_parsed_fail_.store(0);
    n_filtered_out_.store(0);
    last_rx_mono_ns_.store(0);

    // 最近有效 ENU 位移/深度缓存初始化
    last_dist_enu_m_[0] = 0.0;
    last_dist_enu_m_[1] = 0.0;
    last_dist_enu_m_[2] = 0.0;
    last_depth_m_       = 0.0;
    has_last_dist_      = false;
    has_last_depth_     = false;

    return true;
}

// ============================================================================
// start / stop / filter config
// ============================================================================

bool DvlDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        // 已在运行
        return true;
    }

    stop_requested_.store(false);

    if (!openPort()) {
        std::cerr << "[DVL] openPort() failed on " << port_ << "\n";
        running_.store(false);
        return false;
    }

    // 上电后的“安全初始化序列”：CZ -> PR10 -> PM10 -> CS
    // 这里完全使用厂商命令格式，不自创协议。
    {
        // 确保停机，避免空气中 ping
        if (!sendCommandCZ()) {
            std::cerr << "[DVL] WARN: send CZ failed during start()\n";
        }
        // 启动工作
        if (!sendCommandCS()) {
            std::cerr << "[DVL] WARN: send CS failed during start()\n";
        }

        // 设置发射呼率 = 10
        if (!sendCommandPR(10)) {
            std::cerr << "[DVL] WARN: send PR10 failed during start()\n";
        }

        // 设置平均次数 = 10
        if (!sendCommandPM(10)) {
            std::cerr << "[DVL] WARN: send PM10 failed during start()\n";
        }

    }

    try {
        th_ = std::thread(&DvlDriver::threadFunc, this);
    } catch (...) {
        std::cerr << "[DVL] failed to start thread\n";
        closePort();
        running_.store(false);
        throw;
    }

    std::cerr << "[DVL] started on " << port_ << " @" << baud_ << "\n";
    return true;
}

void DvlDriver::stop() noexcept {
    if (!running_.load()) {
        return;
    }

    stop_requested_.store(true);

    // 提前关闭串口，打断 select/read
    closePort();

    if (th_.joinable()) {
        th_.join();
    }

    running_.store(false);

    // 退出前再发一遍 CZ 确保停机（若串口已断开则会失败）
    if (fd_ >= 0) {
        if (!sendCommandCZ()) {
            std::cerr << "[DVL] WARN: send CZ failed during stop()\n";
        }
    }

    std::cerr << "[DVL] stopped\n";
}

void DvlDriver::setFilterConfig(const DvlFilterConfig& cfg) {
    std::lock_guard<std::mutex> lk(filter_mutex_);
    filter_cfg_ = cfg;
}

DvlFilterConfig DvlDriver::filterConfig() const {
    std::lock_guard<std::mutex> lk(filter_mutex_);
    return filter_cfg_;
}

// ============================================================================
// 串口打开 / 关闭
// ============================================================================

bool DvlDriver::openPort() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "[DVL] open(" << port_ << ") failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "[DVL] tcgetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfmakeraw(&tio);

    const speed_t s = baudToTermios(baud_);
    cfsetispeed(&tio, s);
    cfsetospeed(&tio, s);

    tio.c_cflag |= (CLOCAL | CREAD);
    const tcflag_t csize_mask  = CSIZE;
    const tcflag_t parenb_mask = PARENB;
    const tcflag_t cstopb_mask = CSTOPB;

    tio.c_cflag &= ~parenb_mask;
    tio.c_cflag |= CS8;          // 8N1
    tio.c_cflag &= ~csize_mask;
    tio.c_cflag &= ~cstopb_mask;

    // 配合 select：非阻塞 + 超时
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 10;        // read 超时 1.0s

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "[DVL] tcsetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    std::cerr << "[DVL] serial opened: " << port_
              << " @" << baud_ << "\n";
    return true;
}

void DvlDriver::closePort() noexcept {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cerr << "[DVL] serial closed\n";
    }
}

// ============================================================================
// 后台线程：串口读 + 按行切分 + 协议解析
// ============================================================================

void DvlDriver::threadFunc() {
    std::string line_buf;
    line_buf.reserve(1024);

    while (!stop_requested_.load()) {
        if (fd_ < 0) {
            if (!openPort()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        // 1) select 等待可读
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = 200 * 1000; // 200 ms

        const int ret = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[DVL] select() error: "
                      << std::strerror(errno) << "\n";
            closePort();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (ret == 0) {
            // timeout
            continue;
        }

        if (!FD_ISSET(fd_, &rfds)) {
            continue;
        }

        // 2) 读串口
        char    buf[256];
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0) {
            std::cerr << "[DVL] read() error: "
                      << std::strerror(errno) << "\n";
            closePort();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (n == 0) {
            continue;
        }

        line_buf.append(buf, static_cast<std::size_t>(n));

        // 3) 按行切分
        for (;;) {
            std::size_t pos = line_buf.find_first_of("\r\n");
            if (pos == std::string::npos) {
                break;
            }

            std::string one = line_buf.substr(0, pos);
            line_buf.erase(0, pos + 1);

            one = trim(one);
            if (one.empty()) {
                continue;
            }

            n_lines_++;

            // 只尝试解析 PD6/EPD6 速度/距离帧，其它行（TS/SA 等）直接忽略
            dp::ParsedLine parsed{};
            if (!dp::parse_pd6_line(one, parsed)) {
                n_parsed_fail_++;
                continue;
            }
            n_parsed_ok_++;

            // 4) 打统一时间戳
            auto ts = tb::stamp("dvl0", tb::SensorKind::DVL);

            DvlRawData raw{};
            raw.mono_ns = ts.host_time_ns;
            raw.est_ns  = ts.corrected_time_ns;
            raw.parsed  = std::move(parsed);

            last_rx_mono_ns_.store(raw.mono_ns);

            // 5) 进入过滤 + 回调
            handleRawSample(raw);
        }
    }

    std::cerr << "[DVL] threadFunc exit. stats: "
              << "lines="         << n_lines_.load()
              << ", parsed_ok="   << n_parsed_ok_.load()
              << ", parsed_fail=" << n_parsed_fail_.load()
              << ", filtered_out="<< n_filtered_out_.load()
              << "\n";
}

// ============================================================================
// 过滤入口：Raw → Frame + 回调
// ============================================================================

void DvlDriver::handleRawSample(const DvlRawData& raw) {
    if (on_raw_) {
        on_raw_(raw);
    }

    DvlFrame frame{};
    if (passFilter(raw, frame)) {
        if (on_frame_) {
            on_frame_(frame);
        }
    } else {
        n_filtered_out_++;
    }
}

// ============================================================================
// 过滤逻辑：DvlRawData → DvlFrame
// ============================================================================
//
// 策略：
//   1) 只接受 valid == A（cfg.only_valid_flag 为真时）；
//   2) 三轴速度有限性 / 幅值；
//   3) 高度（离底距离）在 [min_altitude_m, max_altitude_m]；
//   4) 位移/深度字段若当前帧无效，用最近一次有效值回填；
//   5) 时间戳直接用 raw.mono_ns / raw.est_ns。
//
bool DvlDriver::passFilter(const DvlRawData& raw, DvlFrame& out_frame) {
    const auto& p = raw.parsed;

    // 0) 取一份过滤配置
    DvlFilterConfig cfg;
    {
        std::lock_guard<std::mutex> lk(filter_mutex_);
        cfg = filter_cfg_;
    }

    // 1) A/V 有效标志
    if (cfg.only_valid_flag) {
        if (p.valid != 1) {
            return false;
        }
    }

    // 2) 速度（无论坐标系）先转成 m/s 做门控
    const double vx = p.ve_mps();
    const double vy = p.vn_mps();
    const double vz = p.vu_mps();

    if (cfg.require_all_axes) {
        if (!isFinite(vx) || !isFinite(vy) || !isFinite(vz)) {
            return false;
        }
    }

    if (cfg.max_abs_vel_mps > 0.0f) {
        if (std::fabs(vx) > cfg.max_abs_vel_mps ||
            std::fabs(vy) > cfg.max_abs_vel_mps ||
            std::fabs(vz) > cfg.max_abs_vel_mps) {
            return false;
        }
    }

    // 3) 高度门控（使用 parsed.depth_m 作为离底高度 / 深度）
    double altitude = p.depth_m;  // 约定：解析层已按 m 输出
    if (cfg.enable_altitude) {
        if (isFinite(altitude)) {
            if (cfg.max_altitude_m > 0.0f &&
                altitude > cfg.max_altitude_m) {
                return false;
            }
            if (cfg.min_altitude_m > 0.0f &&
                altitude < cfg.min_altitude_m) {
                return false;
            }
        }
    }

    // 4) 质量门控（占位，可按需要扩展）
    if (cfg.min_quality > 0) {
        if (p.quality < cfg.min_quality) {
            return false;
        }
    }

    // ---------- 填充输出帧 ----------

    out_frame.mono_ns = raw.mono_ns;
    out_frame.est_ns  = raw.est_ns;

    using nav_core::dvl_protocol::TrackType;

    // p.track_type 是 dvl_protocol::TrackType
    const auto raw_tt = static_cast<std::uint8_t>(p.track_type);
    out_frame.track_type = static_cast<nav_core::DvlTrackType>(raw_tt);

    out_frame.bottom_lock = p.bottom_lock;

    // 4.1 速度写入坐标系对应字段
    if (p.coord_frame == dp::CoordFrame::Earth) {
        out_frame.vel_enu_mps[0] = static_cast<float>(vx);
        out_frame.vel_enu_mps[1] = static_cast<float>(vy);
        out_frame.vel_enu_mps[2] = static_cast<float>(vz);
        out_frame.has_enu_vel    = true;
    } else if (p.coord_frame == dp::CoordFrame::Ship ||
               p.coord_frame == dp::CoordFrame::Instrument) {
        out_frame.vel_body_mps[0] = static_cast<float>(vx);
        out_frame.vel_body_mps[1] = static_cast<float>(vy);
        out_frame.vel_body_mps[2] = static_cast<float>(vz);
        out_frame.has_body_vel    = true;
    }

    // 4.2 位移 & 深度：使用“最近一次有效值”策略
    bool have_current_dist =
        isFinite(p.e_m) && isFinite(p.n_m) && isFinite(p.u_m);
    bool have_current_depth = isFinite(p.depth_m);

    if (have_current_dist) {
        last_dist_enu_m_[0] = p.e_m;
        last_dist_enu_m_[1] = p.n_m;
        last_dist_enu_m_[2] = p.u_m;
        has_last_dist_      = true;
    }

    if (have_current_depth) {
        last_depth_m_   = p.depth_m;
        has_last_depth_ = true;
    }

    if (has_last_dist_) {
        out_frame.dist_enu_m[0] = static_cast<float>(last_dist_enu_m_[0]);
        out_frame.dist_enu_m[1] = static_cast<float>(last_dist_enu_m_[1]);
        out_frame.dist_enu_m[2] = static_cast<float>(last_dist_enu_m_[2]);
        out_frame.has_dist_enu  = true;
    } else {
        // 尚无任何有效位移，填 0 但 has_dist_enu=false
        out_frame.dist_enu_m[0] = 0.f;
        out_frame.dist_enu_m[1] = 0.f;
        out_frame.dist_enu_m[2] = 0.f;
        out_frame.has_dist_enu  = false;
    }

    if (has_last_depth_) {
        out_frame.altitude_m = static_cast<float>(last_depth_m_);
    } else {
        out_frame.altitude_m = 0.f;
    }

    // 4.3 质量 / 有效标志
    out_frame.fom     = static_cast<float>(p.quality); // 或按需缩放
    out_frame.quality = p.quality;
    out_frame.valid   = true;

    return true;
}

// ============================================================================
// 命令发送实现（CZ / CS / PR / PM）
// ============================================================================

int DvlDriver::writeBytes(const std::uint8_t* buf, std::size_t len) {
    if (fd_ < 0 || buf == nullptr || len == 0) {
        return -1;
    }

    const ssize_t n = ::write(fd_, buf, len);
    if (n < 0) {
        std::cerr << "[DVL] write() error: "
                  << std::strerror(errno) << "\n";
        return -1;
    }
    if (static_cast<std::size_t>(n) != len) {
        std::cerr << "[DVL] write() partial: " << n
                  << " / " << len << "\n";
    }
    return static_cast<int>(n);
}

bool DvlDriver::sendRawCommand16(const std::string& ascii_cmd) {
    const dp::CommandBytes bytes = dp::make_command_from_ascii(ascii_cmd);
    const int n = writeBytes(bytes.data(), bytes.size());
    return (n == static_cast<int>(bytes.size()));
}

// 私有工具函数实现
bool DvlDriver::sendCommand(const dp::CommandBytes& bytes, const char* /*tag*/)
{
    // 防御：空命令直接视为失败，避免 writeBytes(…, 0) 这种无意义调用
    if (bytes.empty()) {
        return false;
    }

    const std::size_t expected = bytes.size();
    const int n = writeBytes(bytes.data(), expected);

    // 串口写失败（通常约定为负数），直接返回 false
    if (n < 0) {
        return false;
    }

    // 若底层只写出部分字节，也视为失败（目前策略是“要么全发成功，要么失败”）
    return static_cast<std::size_t>(n) == expected;
}

// -------------------- 对外接口 --------------------

bool DvlDriver::sendCommandCZ()
{
    const dp::CommandBytes bytes = dp::make_command_cz();
    return sendCommand(bytes, "CZ");
}

bool DvlDriver::sendCommandCS()
{
    const dp::CommandBytes bytes = dp::make_command_cs();
    return sendCommand(bytes, "CS");
}

bool DvlDriver::sendCommandPR(int ping_rate)
{
    const dp::CommandBytes bytes = dp::make_command_pr(ping_rate);
    return sendCommand(bytes, "PR");
}

bool DvlDriver::sendCommandPM(int avg_count)
{
    const dp::CommandBytes bytes = dp::make_command_pm(avg_count);
    return sendCommand(bytes, "PM");
}

} // namespace nav_core::drivers
