// nav_core/src/drivers/dvl_driver.cpp
//
// Hover H1000 DVL 串口驱动 + 文本协议解析
//
// 当前职责：
//   - 串口 IO + 后台线程：按行读取 PD6/EPD6 文本；
//   - 调用 nav_core::dvl_protocol 解析为 ParsedLine；
//   - 使用统一时间基 (nav_core::core::timebase) 打时间戳；
//   - 将带时间戳的 DvlRawData 通过 RawCallback 推给上层（例如 DVL_logger）；
//   - 负责发送 CZ / CS / PR / PM 等命令；
//   - 提供简单的 isAlive() 链路健康判定。
//
// 不负责：
//   - DvlFrame 生成、导航级过滤、坐标系转换、ESKF/轨迹估计、ZUPT、噪声地板等。
//   - BI/BE/BD 分类保存与预处理由上层 Python / 预处理模块完成。

#include <nav_core/drivers/dvl_driver.hpp>
#include <nav_core/core/timebase.hpp>
#include <nav_core/drivers/dvl_protocol.hpp>
#include <nav_core/preprocess/dvl_rt_preprocessor.hpp> 

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
constexpr int kDefaultPingRate = 10;  ///< DVL 默认发射呼率
constexpr int kDefaultAvgCount = 10;  ///< DVL 默认平均次数

// 波特率转换：int → termios speed_t
inline speed_t baudToTermios(int baud) {
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
    // ★ 默认发射呼率 / 平均次数：都设为 10

}

} // namespace

// ============================================================================
// 构造 / 析构 / init
// ============================================================================

DvlDriver::DvlDriver() = default;

DvlDriver::DvlDriver(const DvlConfig& cfg,
                     RawCallback      on_raw)
{
    init(cfg, std::move(on_raw));
}

DvlDriver::DvlDriver(const std::string& port,
                     int                baud,
                     RawCallback        on_raw)
{
    DvlConfig cfg;
    cfg.port = port;
    cfg.baud = baud;
    init(cfg, std::move(on_raw));
}

DvlDriver::~DvlDriver() {
    stop();
}

bool DvlDriver::init(const DvlConfig& cfg,
                     RawCallback      on_raw)
{
    cfg_    = cfg;
    port_   = cfg.port;
    baud_   = cfg.baud;
    on_raw_ = std::move(on_raw);

    fd_ = -1;

    stop_requested_.store(false);
    running_.store(false);

    n_lines_.store(0);
    n_parsed_ok_.store(0);
    n_parsed_fail_.store(0);
    last_rx_mono_ns_.store(0);

    return true;
}

// ============================================================================
// start / stop
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

    // 上电后的安全初始化序列（可选）：
    // CZ -> CS -> PR(ping_rate) -> PM(avg_count) 
    if (cfg_.send_startup_cmds) {
        // 1) 确保停机，避免空气中 ping
        if (!sendCommandCZ()) {
            std::cerr << "[DVL] WARN: send CZ failed during start()\n";
        }
        // 2) 启动 ping
        if (!sendCommandCS()) {
            std::cerr << "[DVL] WARN: send CS failed during start()\n";
        }

        // 3) 设置发射呼率（优先用配置；<=0 则回退到默认 10）
        const int ping_rate = (cfg_.ping_rate > 0)
                                ? cfg_.ping_rate
                                : kDefaultPingRate;
        if (!sendCommandPR(ping_rate)) {
            std::cerr << "[DVL] WARN: send PR" << ping_rate
                      << " failed during start()\n";
        }

        // 4) 设置平均次数
         const int avg_count = (cfg_.avg_count > 0)
                                ? cfg_.avg_count
                                : kDefaultAvgCount;
        if (!sendCommandPM(avg_count)) {
            std::cerr << "[DVL] WARN: send PM" << avg_count
                      << " failed during start()\n";
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

    // 停止前尽量再发一次 CZ 确保停机（若串口已断开则会失败）
    if (fd_ >= 0) {
        if (!sendCommandCZ()) {
            std::cerr << "[DVL] WARN: send CZ failed during stop()\n";
        }
    }

    // 关闭串口打断 select/read
    closePort();

    if (th_.joinable()) {
        th_.join();
    }

    running_.store(false);

    std::cerr << "[DVL] stopped\n";
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

            // 尝试解析 PD6/EPD6 速度/距离帧，其它行（TS/SA 等）parse_pd6_line 会返回 false
            dp::ParsedLine parsed{};
            if (!dp::parse_pd6_line(one, parsed)) {
                n_parsed_fail_++;
                continue;
            }
            n_parsed_ok_++;

            // 4) 打统一时间戳
            auto ts = tb::stamp("dvl0", tb::SensorKind::DVL);

            DvlRawData raw{};
            raw.recv_mono_ns   = ts.host_time_ns;
            raw.sensor_time_ns = ts.corrected_time_ns;
            raw.mono_ns = (raw.sensor_time_ns > 0) ? raw.sensor_time_ns : raw.recv_mono_ns;
            raw.est_ns  = raw.mono_ns;
            raw.parsed  = std::move(parsed);

            last_rx_mono_ns_.store(raw.recv_mono_ns);

            // 5) 直接回调 Raw（不再在驱动内做过滤/Frame 生成）
            handleRawSample(raw);
        }
    }

    std::cerr << "[DVL] threadFunc exit. stats: "
              << "lines="         << n_lines_.load()
              << ", parsed_ok="   << n_parsed_ok_.load()
              << ", parsed_fail=" << n_parsed_fail_.load()
              << "\n";
}

// ============================================================================
// Raw 回调
// ============================================================================

void DvlDriver::handleRawSample(const DvlRawData& raw) {
    if (on_raw_) {
        on_raw_(raw);
    }
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

bool DvlDriver::sendCommand(const DvlCommandBytes& bytes, const char* /*tag*/) {
    if (bytes.empty()) {
        return false;
    }

    const std::size_t expected = bytes.size();
    const int n = writeBytes(bytes.data(), expected);

    if (n < 0) {
        return false;
    }
    return static_cast<std::size_t>(n) == expected;
}

bool DvlDriver::sendCommandCZ() {
    const dp::CommandBytes bytes = dp::make_command_cz();
    return sendCommand(bytes, "CZ");
}

bool DvlDriver::sendCommandCS() {
    const dp::CommandBytes bytes = dp::make_command_cs();
    return sendCommand(bytes, "CS");
}

bool DvlDriver::sendCommandPR(int ping_rate) {
    const dp::CommandBytes bytes = dp::make_command_pr(ping_rate);
    return sendCommand(bytes, "PR");
}

bool DvlDriver::sendCommandPM(int avg_count) {
    const dp::CommandBytes bytes = dp::make_command_pm(avg_count);
    return sendCommand(bytes, "PM");
}
// ============================================================================
// DvlRawData -> preprocess::DvlRawSample 桥接工具
// ============================================================================

bool makeDvlRawSample(const DvlRawData& in,
                      ::nav_core::preprocess::DvlRawSample& out)
{
    using ::nav_core::preprocess::DvlRawSample;

    const dp::ParsedLine& pl = in.parsed;

    // 只支持 BI/BE/BS/BD 这四种导航相关帧，其他帧返回 false 让上层跳过
    std::uint8_t kind = 0xFF;
    if      (pl.src == "BI") kind = 0;  // Instrument / BottomTrack 速度
    else if (pl.src == "BE") kind = 1;  // Earth / BottomTrack 速度
    else if (pl.src == "BS") kind = 2;  // Ship / BottomTrack 速度（预留）
    else if (pl.src == "BD") kind = 3;  // Earth / 位移帧（累计距离）
    else {
        return false;   // 例如 WI/WE/WD/TS/SA 等暂不参与导航预处理
    }

    out = DvlRawSample{};  // 全部清零

    // 时间戳
    out.sensor_time_ns = in.sensor_time_ns;
    out.recv_mono_ns = in.recv_mono_ns;
    out.consume_mono_ns = 0;
    out.mono_ns = in.mono_ns;
    out.est_ns  = in.est_ns;

    // 基本标志
    out.kind        = kind;
    out.bottom_lock = pl.bottom_lock;
    out.fom         = static_cast<float>(pl.quality);
    out.status      = 0u;  // 如未来在 ParsedLine 中解析 status bit，可在这里填充

    // Beam 距离 / 相关系数：当前协议解析层未提供 beam-level 数据，先清零占位
    for (int i = 0; i < 4; ++i) {
        out.range_m[i] = 0.0f;
        out.corr[i]    = 0.0f;
    }

    // 速度分量：
    // 1) Instrument / Ship frame → vel_inst_mps（仪器/船体坐标系速度）
    if (pl.coord_frame == dp::CoordFrame::Instrument ||
        pl.coord_frame == dp::CoordFrame::Ship)
    {
        if (pl.has_e) out.vel_inst_mps.x = static_cast<float>(pl.ve_mps());
        if (pl.has_n) out.vel_inst_mps.y = static_cast<float>(pl.vn_mps());
        if (pl.has_u) out.vel_inst_mps.z = static_cast<float>(pl.vu_mps());
    }

    // 2) Earth frame → vel_earth_mps（ENU: [vE, vN, vU]）
    if (pl.coord_frame == dp::CoordFrame::Earth) {
        if (pl.has_e) out.vel_earth_mps.x = static_cast<float>(pl.ve_mps());
        if (pl.has_n) out.vel_earth_mps.y = static_cast<float>(pl.vn_mps());
        if (pl.has_u) out.vel_earth_mps.z = static_cast<float>(pl.vu_mps());
    }

    // 位移 / 深度：目前 DvlRawSample 只预留了 range/corr，不直接放 ENU 位移；
    // 如后续需要 BD 的累计 ENU 距离，可考虑扩展 DvlRawSample 结构体。
    // 这里先不对 e_m/n_m/u_m/depth_m 做更多映射，保持语义简单。

    return true;
}

} // namespace nav_core::drivers
