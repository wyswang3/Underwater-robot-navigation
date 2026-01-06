// nav_core/src/dvl_driver.cpp
//
// Hover H1000 DVL 串口驱动 + 文本协议解析 + 基本质量过滤
//
// 职责边界：
//   - 负责：
//       * 串口打开/关闭、后台线程循环读取；
//       * 将文本帧解析成 DvlRawData；
//       * 应用简单过滤规则，生成 DvlFrame；
//       * 通过回调把数据交给上层（ESKF / 记录器）。
//   - 不负责：
//       * 复杂滤波 / 轨迹推算（交给上层模块，比如 ESKF）；
//       * 线程同步策略（上层只要把回调写成“短执行时间”即可）。
//
// 使用方式：
//   nav_core::DvlFilterConfig cfg;
//   cfg.only_valid_flag  = true;
//   cfg.require_all_axes = true;
//   cfg.max_abs_vel_mps  = 5.0;
//
//   nav_core::DvlDriver dvl("/dev/ttyACM0", 115200, cfg,
//       [](const nav_core::DvlFrame& f) { /* 用于 ESKF / 控制 */ },
//       [](const nav_core::DvlRawData& r) { /* 可选：原始日志 */ });
//
//   dvl.start();
//   ...
//   dvl.stop();

#include <nav_core/dvl_driver.hpp>
#include <nav_core/timebase.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

// POSIX 串口相关
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <ctime>

namespace nav_core {

// ============================================================================
// 匿名命名空间：内部小工具
// ============================================================================

namespace {

inline bool isFinite(double x) {
    return std::isfinite(x);
}

// 简单的波特率映射（只覆盖我们可能用到的）
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
            // 不认识的就强制退回 115200，至少能工作
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

// “安全 atoi”：支持 +/−，过滤 88888/NAN 等占位值。
// tok: 形如 "+123" 或 "-456" 的字符串；
// out: 输出 double；
// 返回：true = 解析成功，false = 占位值或非法。
inline bool safeAtoi(const std::string& tok, double& out) {
    std::string t = trim(tok);
    if (t.empty()) {
        return false;
    }

    std::string up;
    up.reserve(t.size());
    for (char c : t) {
        up.push_back(static_cast<char>(
            std::toupper(static_cast<unsigned char>(c))));
    }
    // Hover H1000 常用 "88888"/"NAN" 作为无效占位
    if (up == "88888" || up == "+88888" || up == "-88888" || up == "NAN") {
        return false;
    }

    char* endp = nullptr;
    long  v    = std::strtol(t.c_str(), &endp, 10);
    if (endp == t.c_str()) {
        // 没有任何数字
        return false;
    }

    out = static_cast<double>(v);
    return true;
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
    cfg_        = cfg;
    port_       = cfg.port;
    baud_       = cfg.baud;
    on_frame_   = std::move(on_frame);
    on_raw_     = std::move(on_raw);

    {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        filter_cfg_ = cfg.filter;
    }

    fd_ = -1;

    stop_requested_.store(false);
    running_.store(false);

    n_lines_.store(0);
    n_parsed_ok_.store(0);
    n_parsed_fail_.store(0);
    n_filtered_out_.store(0);

    return true;
}

// ============================================================================
// 公共接口：start / stop / filter config
// ============================================================================

bool DvlDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        // 已经在运行，直接视为成功
        return true;
    }

    stop_requested_.store(false);

    if (!openPort()) {
        std::cerr << "[DVL] openPort() failed on " << port_ << "\n";
        running_.store(false);
        return false;
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

    // 关闭串口可以打断 select/read，线程会尽快退出
    closePort();

    if (th_.joinable()) {
        th_.join();
    }

    running_.store(false);

    std::cerr << "[DVL] stopped\n";
}

void DvlDriver::setFilterConfig(const DvlFilterConfig& cfg) {
    std::lock_guard<std::mutex> lock(filter_mutex_);
    filter_cfg_ = cfg;
}

DvlFilterConfig DvlDriver::filterConfig() const {
    std::lock_guard<std::mutex> lock(filter_mutex_);
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


    // 配合 select 使用：非阻塞 + 超时
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
//
// 线程模型：
//   - 使用 select(fd) 等待数据，避免长时间阻塞；
//   - 读到数据后追加到 line_buf；
//   - 按 '\r' / '\n' 切行；
//   - 每行调用 parseLine() → DvlRawData；
//   - 再交给 handleRawSample() 做过滤和回调。
//
void DvlDriver::threadFunc() {
    std::string line_buf;
    line_buf.reserve(1024);

    while (!stop_requested_.load()) {
        if (fd_ < 0) {
            // 可能是打开失败或中途掉线，这里做简单重连
            if (!openPort()) {
                std::this_thread::sleep_for(
                    std::chrono::milliseconds(500));
                continue;
            }
        }

        // 1) select 等待串口可读
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
            std::this_thread::sleep_for(
                std::chrono::milliseconds(200));
            continue;
        }
        if (ret == 0) {
            // 超时，无数据
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
            std::this_thread::sleep_for(
                std::chrono::milliseconds(200));
            continue;
        }
        if (n == 0) {
            // 对端关闭或暂时无数据
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

            n_lines_++;  // 收到一行有效文本

            DvlRawData raw{};
            if (!parseLine(one, raw)) {
                n_parsed_fail_++;
                continue;
            }
            n_parsed_ok_++;

            // 交给过滤与回调
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
// 协议解析：一行 Hover H1000 文本 → DvlRawData
// ============================================================================
//
// 注意：这里实现的是「鲁棒版」解析：
//   - 在整行里找到第一个 ":BE/BS/BI/WE/WS/WI" 帧；
//   - 从 payload 里“扫”出最多 3 个整数速度（mm/s）；
//   - 忽略 88888/NAN 等占位；
//   - 有效标志从最后一个 'A'/'V' 决定；
//   - 时间戳使用 timebase::stamp() 统一到 MonoNS + EstNS。
//
bool DvlDriver::parseLine(const std::string& line, DvlRawData& out_raw) {
    // 1) 基本清理
    std::string s = trim(line);
    if (s.empty()) {
        return false;
    }

    // 去掉外围引号（如果有）
    if (!s.empty() && (s.front() == '"' || s.front() == '\'')) {
        s.erase(s.begin());
    }
    if (!s.empty() && (s.back() == '"' || s.back() == '\'')) {
        s.pop_back();
    }
    if (s.empty()) {
        return false;
    }

    // 2) 内部函数：解析单个速度帧，比如 "BE,+123,-45,+6,A"
    auto parseVelocityFrame = [](const std::string& frame_raw,
                                 DvlRawData&        d,
                                 std::time_t        ts_wall) -> bool
    {
        if (frame_raw.size() < 2) {
            return false;
        }

        // 前两个字符是消息类型：BE/BS/BI/WE/WS/WI
        std::string msg = frame_raw.substr(0, 2);
        for (char& c : msg) {
            c = static_cast<char>(
                std::toupper(static_cast<unsigned char>(c)));
        }
        if (msg != "BE" && msg != "BS" && msg != "BI" &&
            msg != "WE" && msg != "WS" && msg != "WI") {
            return false;
        }

        // 剩余部分是 payload
        std::string payload = frame_raw.substr(2);

        // 从 payload 中“扫描”整数（允许任意分隔符）
        std::vector<double> vel_mm;
        vel_mm.reserve(3);

        for (std::size_t i = 0; i < payload.size();) {
            char c = payload[i];

            // 寻找 [+-]?\d+
            if (c == '+' || c == '-' ||
                std::isdigit(static_cast<unsigned char>(c))) {
                std::size_t j = i;

                // 可选符号
                if (c == '+' || c == '-') {
                    ++j;
                    if (j >= payload.size() ||
                        !std::isdigit(
                            static_cast<unsigned char>(payload[j]))) {
                        // "+" 或 "-" 后面不是数字，当噪声丢掉
                        ++i;
                        continue;
                    }
                }

                // 后续连续数字
                while (j < payload.size() &&
                       std::isdigit(
                           static_cast<unsigned char>(payload[j]))) {
                    ++j;
                }

                std::string tok = payload.substr(i, j - i);
                double      v   = 0.0;
                if (safeAtoi(tok, v)) {
                    vel_mm.push_back(v);
                    if (vel_mm.size() >= 3) {
                        break;
                    }
                }

                i = j;
            } else {
                ++i;
            }
        }

        if (vel_mm.empty()) {
            return false;  // 没扫到任何有效速度
        }

        // 有效标志：从 payload 末尾向前找最后一个 'A' or 'V'
        int valid_flag = -1;
        for (std::size_t i = payload.size(); i > 0; --i) {
            char c = payload[i - 1];
            if (!std::isalpha(static_cast<unsigned char>(c))) {
                continue;
            }
            c = static_cast<char>(
                std::toupper(static_cast<unsigned char>(c)));
            if (c == 'A') { valid_flag = 1; break; }
            if (c == 'V') { valid_flag = 0; break; }
        }

        d.timestamp_s = static_cast<double>(ts_wall);  // 秒级 wall-clock
        d.src         = msg;
        d.ve_mm       = vel_mm.size() > 0 ? vel_mm[0] : 0.0;
        d.vn_mm       = vel_mm.size() > 1 ? vel_mm[1] : 0.0;
        d.vu_mm       = vel_mm.size() > 2 ? vel_mm[2] : 0.0;
        d.depth_m     = 0.0;
        d.e_m         = 0.0;
        d.n_m         = 0.0;
        d.u_m         = 0.0;
        d.valid       = valid_flag;
        return true;
    };

    // 3) 在整行中查找第一个 :BE/BS/BI/WE/WS/WI
    std::time_t ts_wall = ::time(nullptr);
    const char* tags[]  = {"BE", "BS", "BI", "WE", "WS", "WI"};

    bool parsed = false;
    for (const char* tag : tags) {
        std::string needle = std::string(":") + tag;
        std::size_t pos    = s.find(needle);
        if (pos == std::string::npos) {
            continue;
        }

        // ":" 后开始，到下一次 ":" 或行尾
        std::size_t frame_start = pos + 1;  // 指向 "B" 或 "W"
        std::size_t next_colon  = s.find(':', frame_start);
        std::string frame = (next_colon == std::string::npos)
                            ? s.substr(frame_start)
                            : s.substr(frame_start,
                                       next_colon - frame_start);

        frame = trim(frame);
        if (frame.empty()) {
            continue;
        }

        if (parseVelocityFrame(frame, out_raw, ts_wall)) {
            parsed = true;
            break;  // 只取第一帧即可
        }
    }

    if (!parsed) {
        return false;
    }

    // 4) 统一时间戳（使用 uwnav::timebase）
    using uwnav::timebase::SensorKind;
    using uwnav::timebase::stamp;

    auto ts = stamp("dvl0", SensorKind::DVL);
    out_raw.mono_ns = ts.host_time_ns;      // 主机 steady_clock 时间
    out_raw.est_ns  = ts.corrected_time_ns; // 延迟补偿后的统一时间轴

    return true;
}

// ============================================================================
// 将原始数据交给过滤与回调
// ============================================================================
//
// 这里的逻辑很简单：
//   - 如果用户需要原始数据（on_raw_ 有值），先回调 raw；
//   - 再调 passFilter()，通过则触发 on_frame_，否则统计 filtered_out。
//   - 注意：passFilter 会把 raw 的时间戳复制给 DvlFrame。
//
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
// 过滤策略：
//   1) only_valid_flag 为真时，只接受 valid==A（锁底且质量好）；
//   2) require_all_axes 为真时，三轴速度都必须是有限数；
//   3) 三轴速度绝对值不能超过 max_abs_vel_mps（防尖峰）；
//   4) 预留质量阈值 min_quality（目前还未使用）；
//   5) 时间戳直接沿用 raw.mono_ns / raw.est_ns，不再重复打 stamp()。
//
bool DvlDriver::passFilter(const DvlRawData& raw, DvlFrame& out_frame) {
    // 拷贝一份当前过滤配置，避免长时间持有锁
    DvlFilterConfig cfg;
    {
        std::lock_guard<std::mutex> lock(filter_mutex_);
        cfg = filter_cfg_;
    }

    // 1) 只接受 valid == A 的帧
    if (cfg.only_valid_flag) {
        if (raw.valid != 1) {
            return false;
        }
    }

    // 2) 三轴速度有穷性检查
    const double ve = raw.ve_mps();
    const double vn = raw.vn_mps();
    const double vu = raw.vu_mps();

    if (cfg.require_all_axes) {
        if (!isFinite(ve) || !isFinite(vn) || !isFinite(vu)) {
            return false;
        }
    }

    // 3) 最大速度幅值门控（简单防止“误爆”）
    if (cfg.max_abs_vel_mps > 0.0) {
        if (std::fabs(ve) > cfg.max_abs_vel_mps ||
            std::fabs(vn) > cfg.max_abs_vel_mps ||
            std::fabs(vu) > cfg.max_abs_vel_mps) {
            return false;
        }
    }

    // 4) 质量门控（预留）
    if (cfg.min_quality > 0) {
        // 将来可以在 DvlRawData 中加入 quality 字段后，在此处判断：
        // if (raw.quality < cfg.min_quality) return false;
    }

    // 5) 填充输出帧
    out_frame.mono_ns = raw.mono_ns;
    out_frame.est_ns  = raw.est_ns;

    out_frame.vel[0] = static_cast<float>(ve);
    out_frame.vel[1] = static_cast<float>(vn);
    out_frame.vel[2] = static_cast<float>(vu);

    out_frame.valid   = true;
    out_frame.quality = 0;  // 将来可改为 DVL 的质量等级

    return true;
}

} // namespace nav_core
