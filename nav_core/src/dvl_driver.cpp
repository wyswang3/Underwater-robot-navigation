#include "nav_core/dvl_driver.h"

#include "nav_core/timebase.h"
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <cctype>
#include <ctime>
#include <vector>
#include <iostream>   // 暂时用于日志，后面可替换为你的 logging 封装

namespace nav_core {

// ---------- 小工具 ----------

namespace {

inline bool isFinite(double x) {
    return std::isfinite(x);
}

// 简单的波特率映射
speed_t baudToTermios(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
#ifdef B460800
        case 460800: return B460800;
#endif
        default:
            // 默认 115200
            return B115200;
    }
}

// 非常简化的 trim（前后空白）
inline std::string trim(const std::string& s) {
    std::size_t b = 0;
    while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b]))) ++b;
    if (b == s.size()) return {};
    std::size_t e = s.size();
    while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

// 安全 atoi（返回 bool）
inline bool safeAtoi(const std::string& tok, double& out) {
    std::string t = trim(tok);
    if (t.empty()) return false;
    std::string up;
    up.reserve(t.size());
    for (char c : t) {
        up.push_back(static_cast<char>(std::toupper(static_cast<unsigned char>(c))));
    }
    if (up == "88888" || up == "+88888" || up == "-88888" || up == "NAN") {
        return false;
    }
    char* endp = nullptr;
    long v = std::strtol(t.c_str(), &endp, 10);
    if (endp == t.c_str()) {
        return false;
    }
    out = static_cast<double>(v);
    return true;
}

} // anonymous namespace

// ---------- DvlDriver 实现 ----------

DvlDriver::DvlDriver(const std::string& port,
                     int baud,
                     const DvlFilterConfig& filter,
                     FrameCallback on_frame,
                     RawCallback   on_raw)
    : port_(port),
      baud_(baud),
      fd_(-1),
      on_frame_(std::move(on_frame)),
      on_raw_(std::move(on_raw)),
      filter_cfg_(filter)
{
}

DvlDriver::~DvlDriver() {
    stop();
}

bool DvlDriver::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        // 已经在运行
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
    return true;
}

void DvlDriver::stop() {
    if (!running_.load()) {
        return;
    }

    stop_requested_.store(true);

    // 关闭串口可以打断 select/read
    closePort();

    if (th_.joinable()) {
        th_.join();
    }

    running_.store(false);
}

void DvlDriver::setFilterConfig(const DvlFilterConfig& cfg) {
    filter_cfg_ = cfg;
}

DvlFilterConfig DvlDriver::filterConfig() const {
    return filter_cfg_;
}

bool DvlDriver::openPort() {
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "[DVL] open(" << port_ << ") failed: " << std::strerror(errno) << "\n";
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "[DVL] tcgetattr failed: " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    cfmakeraw(&tio);  // 原始模式

    speed_t s = baudToTermios(baud_);
    cfsetispeed(&tio, s);
    cfsetospeed(&tio, s);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;         // 8 数据位
    tio.c_cflag &= ~PARENB;     // 无校验
    tio.c_cflag &= ~CSTOPB;     // 1 stop bit

    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 10;  // 读超时 1.0s（配合 select 使用，无伤大雅）

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "[DVL] tcsetattr failed: " << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    std::cerr << "[DVL] serial opened: " << port_ << " @" << baud_ << "\n";
    return true;
}

void DvlDriver::closePort() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cerr << "[DVL] serial closed\n";
    }
}

void DvlDriver::threadFunc() {
    std::string line_buf;
    line_buf.reserve(1024);

    while (!stop_requested_.load()) {
        if (fd_ < 0) {
            // 可能是打开失败或中途掉线，这里简单尝试重连
            if (!openPort()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
        }

        // 使用 select 做一个简单的超时等待，避免 read 永久阻塞
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        timeval tv{};
        tv.tv_sec  = 0;
        tv.tv_usec = 200 * 1000; // 200ms

        int ret = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[DVL] select() error: " << std::strerror(errno) << "\n";
            closePort();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (ret == 0) {
            // 超时，无数据
            continue;
        }

        if (!FD_ISSET(fd_, &rfds)) {
            continue;
        }

        char buf[256];
        ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n < 0) {
            std::cerr << "[DVL] read() error: " << std::strerror(errno) << "\n";
            closePort();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        if (n == 0) {
            // 对端关闭或暂时无数据
            continue;
        }

        line_buf.append(buf, static_cast<std::size_t>(n));

        // 按 \r 或 \n 分行
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

            n_lines_++; // 按“有效行”计数

            DvlRawData raw;
            if (!parseLine(one, raw)) {
                n_parsed_fail_++;
                continue;
            }
            n_parsed_ok_++;

            // 将解析结果交给过滤与回调处理
            handleRawSample(raw);
        }
    }

    std::cerr << "[DVL] threadFunc exit. stats: "
              << "lines=" << n_lines_.load()
              << ", parsed_ok=" << n_parsed_ok_.load()
              << ", parsed_fail=" << n_parsed_fail_.load()
              << ", filtered_out=" << n_filtered_out_.load()
              << "\n";
}

// 这里是协议解析入口：把一行文本解析成 DvlRawData
bool DvlDriver::parseLine(const std::string& line, DvlRawData& out_raw) {
    // ==== 1. 清理字符串 ====
    std::string s = trim(line);
    if (s.empty()) {
        return false;
    }

    // 把外围的引号去掉："..."
    if (!s.empty() && (s.front() == '"' || s.front() == '\'')) {
        s.erase(s.begin());
    }
    if (!s.empty() && (s.back() == '"' || s.back() == '\'')) {
        s.pop_back();
    }
    if (s.empty()) {
        return false;
    }

    auto now = ::time(nullptr);  // 暂用系统秒级时间

    // ==== 2. 解析单个速度帧的内部函数（更鲁棒） ====
    auto parseVelocityFrame = [](const std::string& frame_raw,
                                 DvlRawData& d,
                                 std::time_t ts) -> bool
    {
        if (frame_raw.size() < 2) {
            return false;
        }

        // frame_raw 形如 "BE,+88888,+888+,VB..." 或 "BI   5,  1,   +,..."
        // 先取前两个字符作为消息类型
        std::string msg = frame_raw.substr(0, 2);
        for (auto& c : msg) {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        if (msg != "BE" && msg != "BS" && msg != "BI" &&
            msg != "WE" && msg != "WS" && msg != "WI") {
            return false;
        }

        // 去掉前两字符，剩下的是 payload
        std::string payload = frame_raw.substr(2);

        // 从 payload 中“扫描”出最多 3 个整数（允许+/-、任意空格/逗号/杂字符分隔）
        std::vector<double> vel_mm;
        vel_mm.reserve(3);

        for (std::size_t i = 0; i < payload.size();) {
            char c = payload[i];

            // 寻找 [+-]?\d+ 型整数
            if (c == '+' || c == '-' || std::isdigit(static_cast<unsigned char>(c))) {
                std::size_t j = i;

                // 可选符号
                if (c == '+' || c == '-') {
                    ++j;
                    if (j >= payload.size() ||
                        !std::isdigit(static_cast<unsigned char>(payload[j]))) {
                        // "+" 或 "-" 后面不是数字，当作噪声跳过
                        ++i;
                        continue;
                    }
                }

                // 后面连续的数字
                while (j < payload.size() &&
                       std::isdigit(static_cast<unsigned char>(payload[j]))) {
                    ++j;
                }

                std::string tok = payload.substr(i, j - i);
                double v = 0.0;
                if (safeAtoi(tok, v)) {          // safeAtoi 来自前面匿名命名空间
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
            // 一点有效整数都没找到
            return false;
        }

        // 有效标志：找到最后一个 'A' 或 'V'
        int valid_flag = -1;
        for (std::size_t i = payload.size(); i > 0; --i) {
            char c = payload[i - 1];
            if (!std::isalpha(static_cast<unsigned char>(c))) {
                continue;
            }
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            if (c == 'A') { valid_flag = 1; break; }
            if (c == 'V') { valid_flag = 0; break; }
        }

        d.timestamp_s = static_cast<double>(ts);
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

    // ==== 3. 在整行中查找第一个 BE/BS/BI/WE/WS/WI 帧 ====

    // 有些行是以 ":" 开头，有些是中间才出现 ":BE"
    // 我们不再按 ":" 分块，而是直接在整行里找 ":[tag]"
    const char* tags[] = {"BE", "BS", "BI", "WE", "WS", "WI"};

    bool parsed = false;
    for (const char* tag : tags) {
        std::string needle = std::string(":") + tag;
        std::size_t pos = s.find(needle);
        if (pos == std::string::npos) {
            continue;
        }

        // 从 ":" 后开始，到下一次 ":" 或行尾
        std::size_t frame_start = pos + 1;  // 指向 "B"
        std::size_t next_colon  = s.find(':', frame_start);
        std::string frame = (next_colon == std::string::npos)
                            ? s.substr(frame_start)
                            : s.substr(frame_start, next_colon - frame_start);

        frame = trim(frame);
        if (frame.empty()) {
            continue;
        }

        if (parseVelocityFrame(frame, out_raw, now)) {
            parsed = true;
            break;  // 只取第一个速度帧
        }
    }

    if (!parsed) {
        return false;
    }
    return true;
}


// 接收一个解析好的原始样本：先 raw 回调，再做过滤，最后输出 DvlFrame
void DvlDriver::handleRawSample(const DvlRawData& raw) {
    if (on_raw_) {
        on_raw_(raw);
    }

    if (!on_frame_) {
        return; // 没有上层订阅速度帧，直接返回
    }

    DvlFrame frame{};
    if (!passFilter(raw, frame)) {
        n_filtered_out_++;
        return;
    }

    on_frame_(frame);
}

// 过滤逻辑 + 填充 DvlFrame
bool DvlDriver::passFilter(const DvlRawData& raw, DvlFrame& out_frame) {
    DvlFilterConfig cfg = filter_cfg_; // 拷贝一份，避免读写竞争

    // 1) only_valid_flag: 只接受 valid==A
    if (cfg.only_valid_flag) {
        if (raw.valid != 1) {
            return false;
        }
    }

    // 2) require_all_axes: 三轴速度必须是有穷数
    double ve = raw.ve_mps();
    double vn = raw.vn_mps();
    double vu = raw.vu_mps();

    if (cfg.require_all_axes) {
        if (!isFinite(ve) || !isFinite(vn) || !isFinite(vu)) {
            return false;
        }
    }

    // 3) 最大速度幅值门控（防止异常尖刺）
    if (cfg.max_abs_vel_mps > 0.0) {
        if (std::fabs(ve) > cfg.max_abs_vel_mps ||
            std::fabs(vn) > cfg.max_abs_vel_mps ||
            std::fabs(vu) > cfg.max_abs_vel_mps)
        {
            return false;
        }
    }

    // 4) 质量门控（预留，将来可根据协议字段设置 raw.quality）
    if (cfg.min_quality > 0) {
        // 目前 DvlRawData 还没有 quality 字段，这里先忽略，默认通过
    }

    // 若通过过滤，则生成 DvlFrame
    int64_t mono_ns = 0;
    int64_t est_ns  = 0;
    stamp(mono_ns, est_ns);

    out_frame.mono_ns = mono_ns;
    out_frame.est_ns  = est_ns;
    out_frame.vel[0]  = static_cast<float>(ve);
    out_frame.vel[1]  = static_cast<float>(vn);
    out_frame.vel[2]  = static_cast<float>(vu);
    out_frame.valid   = true;
    out_frame.quality = 0; // 将来可填真实质量

    return true;
}

} // namespace nav_core
