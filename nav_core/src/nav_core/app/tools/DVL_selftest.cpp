// DVL_selftest.cpp
//
// Hover H1000 DVL 自检工具：
//   - 阶段 1：不上发任何命令，观察是否有 BI/BE/BD 帧，检查“上电是否自动 ping”；
//   - 阶段 2：按顺序发送 CZ -> CS -> PR10 -> PM10，再观察 BI/BE/BD 帧；
//   - 输出每个阶段的统计信息和最终结论。
// 
// 依赖：nav_core/dvl_protocol.hpp （命令格式和 PD6 解析逻辑）
// 编译：把本文件加入 nav_core 的 CMake，可生成独立可执行文件，例如 DVL_selftest。

#include <nav_core/drivers/dvl_protocol.hpp>

#include <algorithm>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>
#include <thread>


#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace dvlp = nav_core::dvl_protocol;

// ========================== 简单串口工具函数 ==========================

static speed_t baudToTermios(int baud)
{
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
            std::cerr << "[SELFTEST] unsupported baud " << baud
                      << ", fallback to 115200\n";
            return B115200;
    }
}

static bool openSerial(const std::string& port, int baud, int& fd)
{
    fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        std::cerr << "[SELFTEST] open(" << port << ") failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    termios tio{};
    if (tcgetattr(fd, &tio) != 0) {
        std::cerr << "[SELFTEST] tcgetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd);
        fd = -1;
        return false;
    }

    cfmakeraw(&tio);

    const speed_t s = baudToTermios(baud);
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

    // 非阻塞 + 超时（配合 select 使用）
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 5; // 0.5s

    if (tcsetattr(fd, TCSANOW, &tio) != 0) {
        std::cerr << "[SELFTEST] tcsetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd);
        fd = -1;
        return false;
    }

    std::cerr << "[SELFTEST] serial opened: " << port
              << " @" << baud << "\n";
    return true;
}

static void closeSerial(int& fd) noexcept
{
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
        std::cerr << "[SELFTEST] serial closed\n";
    }
}

static bool writeAll(int fd, const std::uint8_t* buf, std::size_t len)
{
    std::size_t total = 0;
    while (total < len) {
        ssize_t n = ::write(fd, buf + total, len - total);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[SELFTEST] write() failed: "
                      << std::strerror(errno) << "\n";
            return false;
        }
        if (n == 0) {
            // 不太正常，但避免死循环
            std::cerr << "[SELFTEST] write() returned 0\n";
            return false;
        }
        total += static_cast<std::size_t>(n);
    }
    return true;
}

// 读取一行（以 '\n' 或 '\r' 结束），带超时；
// 返回值：true  读到一行，line 输出不包含 '\r'/'\n';
//        false 超时或错误（line 可能部分累积，但不完整）
static bool readOneLine(int fd, std::string& line_out, int timeout_ms)
{
    line_out.clear();
    std::string buf;
    buf.reserve(256);

    const auto t_start = std::chrono::steady_clock::now();
    while (true) {
        // 超时检查
        auto dt_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                         std::chrono::steady_clock::now() - t_start)
                         .count();
        if (dt_ms >= timeout_ms) {
            return false; // 超时
        }

        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);

        int remain_ms = timeout_ms - static_cast<int>(dt_ms);
        if (remain_ms < 0) remain_ms = 0;

        timeval tv{};
        tv.tv_sec  = remain_ms / 1000;
        tv.tv_usec = (remain_ms % 1000) * 1000;

        int ret = ::select(fd + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                continue;
            }
            std::cerr << "[SELFTEST] select() error: "
                      << std::strerror(errno) << "\n";
            return false;
        }
        if (ret == 0) {
            // select 超时
            return false;
        }

        if (!FD_ISSET(fd, &rfds)) {
            continue;
        }

        char c;
        ssize_t n = ::read(fd, &c, 1);
        if (n < 0) {
            if (errno == EINTR) continue;
            std::cerr << "[SELFTEST] read() error: "
                      << std::strerror(errno) << "\n";
            return false;
        }
        if (n == 0) {
            // 暂时无数据
            continue;
        }

        if (c == '\n' || c == '\r') {
            if (!buf.empty()) {
                line_out = buf;
                return true;
            } else {
                // 连续的 CR/LF，继续读
                continue;
            }
        } else {
            buf.push_back(c);
            if (buf.size() >= 1024) {
                // 太长了，当作一行截断
                line_out = buf;
                return true;
            }
        }
    }
}

// 去掉首尾空白
static std::string trim(const std::string& s)
{
    std::size_t b = 0;
    while (b < s.size() &&
           std::isspace(static_cast<unsigned char>(s[b]))) {
        ++b;
    }
    if (b == s.size()) return {};

    std::size_t e = s.size();
    while (e > b &&
           std::isspace(static_cast<unsigned char>(s[e - 1]))) {
        --e;
    }
    return s.substr(b, e - b);
}

// ========================== 阶段统计结构 ==========================

struct PhaseStats {
    std::uint64_t lines_total{0};
    std::uint64_t lines_pd6_ok{0};
    std::uint64_t cnt_BI{0};
    std::uint64_t cnt_BE{0};
    std::uint64_t cnt_BD{0};

    void print(const std::string& name) const {
        std::cout << "=== Phase [" << name << "] ===\n";
        std::cout << "  total lines       : " << lines_total   << "\n";
        std::cout << "  PD6 parsed OK     : " << lines_pd6_ok << "\n";
        std::cout << "  BI frames         : " << cnt_BI       << "\n";
        std::cout << "  BE frames         : " << cnt_BE       << "\n";
        std::cout << "  BD frames         : " << cnt_BD       << "\n";
    }

    bool has_any_bottom() const {
        return (cnt_BI + cnt_BE + cnt_BD) > 0;
    }
};

// ========================== 阶段观测逻辑 ==========================
//
// duration_sec    : 观测时长（秒）
// print_each_line : 是否打印每一行原始文本（调试用）
//

static PhaseStats run_phase(int fd,
                            int duration_sec,
                            const std::string& phase_name,
                            bool print_each_line)
{
    PhaseStats st;
    using clock = std::chrono::steady_clock;

    const auto t_start = clock::now();
    const auto t_end   = t_start + std::chrono::seconds(duration_sec);

    std::cout << "[SELFTEST] Start phase [" << phase_name
              << "] duration=" << duration_sec << " s\n";

    while (clock::now() < t_end) {
        std::string line;
        // 这里给每次 readOneLine 一个较短超时，例如 200ms，
        // 以便及时检查整体阶段超时。
        if (!readOneLine(fd, line, /*timeout_ms=*/200)) {
            continue; // 没有完整一行，继续
        }

        ++st.lines_total;

        std::string t = trim(line);
        if (t.empty()) {
            continue;
        }

        if (print_each_line) {
            std::cout << "[RAW] " << t << "\n";
        }

        dvlp::ParsedLine parsed{};
        if (!dvlp::parse_pd6_line(t, parsed)) {
            continue;
        }

        ++st.lines_pd6_ok;

        // 根据 src 统计 BI / BE / BD
        if (parsed.src.size() >= 2) {
            const char c0 = static_cast<char>(
                std::toupper(static_cast<unsigned char>(parsed.src[0])));
            const char c1 = static_cast<char>(
                std::toupper(static_cast<unsigned char>(parsed.src[1])));

            if (c0 == 'B') {
                if (c1 == 'I') ++st.cnt_BI;
                else if (c1 == 'E') ++st.cnt_BE;
                else if (c1 == 'D') ++st.cnt_BD;
            }
        }
    }

    st.print(phase_name);
    return st;
}

// ========================== 命令发送封装 ==========================

static bool send_cmd16(int fd,
                       const dvlp::CommandBytes& cmd,
                       std::string_view tag)
{
    std::cout << "[SELFTEST] SEND " << tag << " (16 bytes)\n";
    if (!writeAll(fd, cmd.data(), cmd.size())) {
        std::cerr << "[SELFTEST] send " << tag << " failed\n";
        return false;
    }
    // 适当等待设备处理
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return true;
}

// ========================== main：自检入口 ==========================

static std::atomic<bool> g_stop_flag{false};

static void sigint_handler(int)
{
    g_stop_flag.store(true);
}

int main(int argc, char** argv)
{
    std::string port = "/dev/ttyACM0";
    int         baud = 115200;
    int         phase1_sec = 5;   // 不下发命令观察时间
    int         phase2_sec = 5;   // 发命令后观察时间
    bool        verbose    = false;

    if (argc >= 2) {
        port = argv[1];
    }
    if (argc >= 3) {
        baud = std::atoi(argv[2]);
        if (baud <= 0) baud = 115200;
    }

    std::cout << "DVL_selftest\n";
    std::cout << "  port = " << port << "\n";
    std::cout << "  baud = " << baud << "\n";
    std::cout << "Usage: " << argv[0] << " [port] [baud]\n";
    std::cout << "  Example: " << argv[0]
              << " /dev/ttyACM0 115200\n\n";

    std::signal(SIGINT, sigint_handler);

    int fd = -1;
    if (!openSerial(port, baud, fd)) {
        return 1;
    }

    // ========== Phase 0: 先发一个 CZ，确保停机 ==========
    {
        auto cz = dvlp::make_command_cz();
        send_cmd16(fd, cz, "CZ (safety stop before test)");
    }

    // ========== Phase 1: 不下发命令，观察 BI/BE/BD ==========
    PhaseStats st1 = run_phase(fd, phase1_sec, "NO_COMMAND", verbose);
    if (g_stop_flag.load()) {
        std::cout << "[SELFTEST] Interrupted by user.\n";
        closeSerial(fd);
        return 0;
    }

    // ========== 下发命令：CZ -> CS -> PR10 -> PM10 ==========
    std::cout << "\n[SELFTEST] Sending init command sequence: "
                 "CZ -> CS -> PR10 -> PM10\n";

    // 再发一次 CZ 确保处于“停机再启动”的清晰状态
    send_cmd16(fd, dvlp::make_command_cz(), "CZ");

    // CS：启动
    send_cmd16(fd, dvlp::make_command_cs(), "CS");

    // PR10：呼率 = 10
    send_cmd16(fd, dvlp::make_command_pr(10), "PR10");

    // PM10：平均次数 = 10
    send_cmd16(fd, dvlp::make_command_pm(10), "PM10");

    // 给设备一点时间切换状态
    std::this_thread::sleep_for(std::chrono::seconds(1));

    if (g_stop_flag.load()) {
        std::cout << "[SELFTEST] Interrupted by user.\n";
        closeSerial(fd);
        return 0;
    }

    // ========== Phase 2: 下发命令后，观察 BI/BE/BD ==========
    PhaseStats st2 = run_phase(fd, phase2_sec, "WITH_COMMANDS", verbose);

    // ========== 结束时再发 CZ，确保停机 ==========
    std::cout << "\n[SELFTEST] Sending final CZ to ensure DVL stop.\n";
    send_cmd16(fd, dvlp::make_command_cz(), "CZ(final)");

    closeSerial(fd);

    // ========== 打印最终结论 ==========
    std::cout << "\n========== SELFTEST SUMMARY ==========\n";

    bool phase1_has_bottom = st1.has_any_bottom();
    bool phase2_has_bottom = st2.has_any_bottom();

    std::cout << "Phase NO_COMMAND has bottom-track frames (BI/BE/BD)? "
              << (phase1_has_bottom ? "YES" : "NO") << "\n";
    std::cout << "Phase WITH_COMMANDS has bottom-track frames (BI/BE/BD)? "
              << (phase2_has_bottom ? "YES" : "NO") << "\n";

    std::cout << "\nConclusion:\n";
    if (!phase1_has_bottom && phase2_has_bottom) {
        std::cout << "  OK: DVL 在空气中不会自动 ping，"
                     "只有在下发 CS/PR/PM 之后才开始返回 BI/BE/BD 帧。\n";
    } else if (phase1_has_bottom) {
        std::cout << "  WARNING: 在未下发任何命令的阶段已经检测到 BI/BE/BD 帧。\n"
                     "           说明 DVL 上电后可能自动工作，存在空气中 ping 的风险，"
                     "请检查设备配置或手册。\n";
        if (!phase2_has_bottom) {
            std::cout << "  另外：下发 CS/PR/PM 后未明显检测到 BI/BE/BD，"
                         "请确认串口连接和命令格式。\n";
        }
    } else if (!phase2_has_bottom) {
        std::cout << "  NOTE: 两个阶段都没有检测到 BI/BE/BD 帧。\n"
                     "        可能原因：DVL 未连接 / 串口或波特率配置错误 / "
                     "设备未处于 PD6 输出模式。\n";
    } else {
        // phase1 无，phase2 有之外的组合其实已覆盖，剩下只是逻辑兜底
        std::cout << "  NOTE: 结果略异常，请结合日志仔细检查。\n";
    }

    std::cout << "======================================\n";

    return 0;
}
