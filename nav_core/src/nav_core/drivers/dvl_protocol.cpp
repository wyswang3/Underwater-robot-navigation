// nav_core/src/dvl_protocol.cpp
//
// Hover H1000 (PD6 / EPD6) 文本协议工具实现：
//   - 单行报文解析（PD6/EPD6 → ParsedLine）
//   - 命令打包（ASCII 命令 → 16 字节固定帧）
//   - 命令回显检测（DVL 是否返回了相同命令）
//
// 说明：
//   - 本文件不依赖任何 IO（串口/线程），仅处理字符串和字节数组；
//   - 不维护历史状态（无“上一帧 BD”之类），位移/深度的“沿用上一帧”
//     逻辑由 DvlDriver / online_estimator 实现；
//   - 速度维度若缺失或出现占位值（+88888），统一以 0 填充，保证数据链不断。

#include <nav_core/drivers/dvl_protocol.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <string>

namespace nav_core::dvl_protocol {

namespace {

// -------------------- 字符串小工具 --------------------

inline std::string_view trim(std::string_view sv) noexcept
{
    while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.front()))) {
        sv.remove_prefix(1);
    }
    while (!sv.empty() && std::isspace(static_cast<unsigned char>(sv.back()))) {
        sv.remove_suffix(1);
    }
    return sv;
}

inline bool ieq_char(char a, char b) noexcept
{
    return std::toupper(static_cast<unsigned char>(a)) ==
           std::toupper(static_cast<unsigned char>(b));
}

inline bool ieq_str(std::string_view a, std::string_view b) noexcept
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!ieq_char(a[i], b[i])) return false;
    }
    return true;
}

// -------------------- token/数字解析 --------------------

inline bool is_placeholder_token(std::string_view sv) noexcept
{
    sv = trim(sv);
    if (sv.empty()) return true;  // 空视作“没有实值”

    // 常见占位形式：+88888 / 88888 / -88888
    if (sv == "+88888" || sv == "88888" || sv == "-88888") {
        return true;
    }
    return false;
}

inline bool parse_int_token(std::string_view sv, int& out) noexcept
{
    sv = trim(sv);
    if (sv.empty()) return false;

    const char* first = sv.data();
    const char* last  = sv.data() + sv.size();

    std::from_chars_result res = std::from_chars(first, last, out, 10);
    if (res.ec != std::errc{}) {
        return false;
    }
    return true;
}

inline bool parse_double_token(std::string_view sv, double& out) noexcept
{
    sv = trim(sv);
    if (sv.empty()) return false;

    // 使用 strtod 比较稳妥一些（charconv 对浮点在部分标准库里支持不完整）
    std::string tmp{sv};
    char* endp = nullptr;
    errno      = 0;
    const double v = std::strtod(tmp.c_str(), &endp);
    if (endp == tmp.c_str() || errno == ERANGE) {
        return false;
    }
    out = v;
    return true;
}

// 将一行 ":BI, +371, +10, -40, -11,A" 切成 token
// content = "BI, +371, +10, -40, -11,A"
template <std::size_t N>
inline std::size_t split_tokens(std::string_view content,
                                std::array<std::string_view, N>& tokens) noexcept
{
    tokens.fill(std::string_view{});
    std::size_t count = 0;

    std::size_t start = 0;
    while (start <= content.size() && count < N) {
        const std::size_t pos = content.find(',', start);
        if (pos == std::string_view::npos) {
            tokens[count++] = content.substr(start);
            break;
        }
        tokens[count++] = content.substr(start, pos - start);
        start = pos + 1;
    }
    return count;
}

// -------------------- 通用 A/V 有效位解析 --------------------

inline int parse_valid_from_status_token(std::string_view sv) noexcept
{
    sv = trim(sv);
    if (sv.empty()) return -1;
    const char c = sv.front();
    if (ieq_char(c, 'A')) return 1;
    if (ieq_char(c, 'V')) return 0;
    return -1;
}

// -------------------- 速度帧解析：BI / BS / BE / WI / WS / WE --------------------

bool parse_velocity_frame(const std::string& src,
                          const std::array<std::string_view, 16>& tok,
                          std::size_t ntok,
                          ParsedLine& out)
{
    if (ntok < 2) {
        return false;  // 连 src 单独一段以外没有任何数值，不认为是合法速度帧
    }

    // 速度 token 约定：token[1], token[2], token[3]
    // 最后一个 token 用作 A/V 状态（BI 有额外一个 range 字段，状态仍然在最后）
    const std::string_view t_ve = (ntok > 1) ? tok[1] : std::string_view{};
    const std::string_view t_vn = (ntok > 2) ? tok[2] : std::string_view{};
    const std::string_view t_vu = (ntok > 3) ? tok[3] : std::string_view{};

    // 速度：缺失/占位 → 0
    int ve_i = 0, vn_i = 0, vu_i = 0;
    if (!is_placeholder_token(t_ve)) {
        int tmp = 0;
        if (parse_int_token(t_ve, tmp)) {
            ve_i = tmp;
        }
    }
    if (!is_placeholder_token(t_vn)) {
        int tmp = 0;
        if (parse_int_token(t_vn, tmp)) {
            vn_i = tmp;
        }
    }
    if (!is_placeholder_token(t_vu)) {
        int tmp = 0;
        if (parse_int_token(t_vu, tmp)) {
            vu_i = tmp;
        }
    }

    out.ve_mm = static_cast<double>(ve_i);
    out.vn_mm = static_cast<double>(vn_i);
    out.vu_mm = static_cast<double>(vu_i);

    // 坐标系 & track_type
    if (!src.empty()) {
        const char c0 = static_cast<char>(std::toupper(static_cast<unsigned char>(src[0])));
        const char c1 = (src.size() > 1)
                      ? static_cast<char>(std::toupper(static_cast<unsigned char>(src[1])))
                      : '\0';

        if (c0 == 'B') {
            out.track_type = TrackType::BottomTrack;
        } else if (c0 == 'W') {
            out.track_type = TrackType::WaterTrack;
        } else {
            out.track_type = TrackType::Unknown;
        }

        if (c1 == 'I') {
            out.coord_frame = CoordFrame::Instrument;
        } else if (c1 == 'S') {
            out.coord_frame = CoordFrame::Ship;
        } else if (c1 == 'E') {
            out.coord_frame = CoordFrame::Earth;
        } else {
            out.coord_frame = CoordFrame::Unknown;
        }
    }

    // A/V 状态：最后一个 token
    if (ntok >= 2) {
        const std::string_view t_status = tok[ntok - 1];
        out.valid = parse_valid_from_status_token(t_status);
    } else {
        out.valid = -1;
    }

    if (out.track_type == TrackType::BottomTrack && out.valid == 1) {
        out.bottom_lock = true;
    } else {
        out.bottom_lock = false;
    }

    // 这里不解析质量字段，保持 out.quality = 0
    return true;
}

// -------------------- 位移 / 深度帧解析：BD / WD --------------------

bool parse_displacement_frame(const std::string& src,
                              const std::array<std::string_view, 16>& tok,
                              std::size_t ntok,
                              ParsedLine& out)
{
    // 期待至少有 6 个字段：src, e, n, u, depth, quality
    if (ntok < 2) {
        return false;
    }

    const std::string_view t_e   = (ntok > 1) ? tok[1] : std::string_view{};
    const std::string_view t_n   = (ntok > 2) ? tok[2] : std::string_view{};
    const std::string_view t_u   = (ntok > 3) ? tok[3] : std::string_view{};
    const std::string_view t_dep = (ntok > 4) ? tok[4] : std::string_view{};
    const std::string_view t_q   = (ntok > 5) ? tok[5] : std::string_view{};

    const bool all_placeholder =
        is_placeholder_token(t_e) &&
        is_placeholder_token(t_n) &&
        is_placeholder_token(t_u) &&
        is_placeholder_token(t_dep);

    out.has_e     = false;
    out.has_n     = false;
    out.has_u     = false;
    out.has_depth = false;

    if (all_placeholder) {
        // 整行没有任何位移/深度信息
        out.e_m     = 0.0;
        out.n_m     = 0.0;
        out.u_m     = 0.0;
        out.depth_m = 0.0;
        out.valid   = 0;
    } else {
        double v = 0.0;

        if (!is_placeholder_token(t_e) && parse_double_token(t_e, v)) {
            out.e_m   = v;
            out.has_e = true;
        } else {
            out.e_m   = 0.0;
        }

        if (!is_placeholder_token(t_n) && parse_double_token(t_n, v)) {
            out.n_m   = v;
            out.has_n = true;
        } else {
            out.n_m   = 0.0;
        }

        if (!is_placeholder_token(t_u) && parse_double_token(t_u, v)) {
            out.u_m   = v;
            out.has_u = true;
        } else {
            out.u_m   = 0.0;
        }

        if (!is_placeholder_token(t_dep) && parse_double_token(t_dep, v)) {
            out.depth_m   = v;
            out.has_depth = true;
        } else {
            out.depth_m   = 0.0;
        }

        out.valid = 1;
    }

    // 质量字段：0.00 ~ 1.00 → 0~100
    double qv = 0.0;
    if (parse_double_token(t_q, qv) && qv >= 0.0) {
        out.quality = static_cast<int>(qv * 100.0 + 0.5);
    } else {
        out.quality = 0;
    }

    // 坐标系 & track_type
    if (!src.empty()) {
        const char c0 = static_cast<char>(std::toupper(static_cast<unsigned char>(src[0])));
        if (c0 == 'B') {
            out.track_type = TrackType::BottomTrack;
        } else if (c0 == 'W') {
            out.track_type = TrackType::WaterTrack;
        } else {
            out.track_type = TrackType::Unknown;
        }
    } else {
        out.track_type = TrackType::Unknown;
    }

    out.coord_frame = CoordFrame::Earth;

    if (out.track_type == TrackType::BottomTrack && out.valid == 1) {
        out.bottom_lock = true;
    } else {
        out.bottom_lock = false;
    }

    return true;
}

// 判断是否是我们支持的 PD6/EPD6 数据帧类型
inline bool is_supported_src(std::string_view src) noexcept
{
    return ieq_str(src, "BI") ||
           ieq_str(src, "BS") ||
           ieq_str(src, "BE") ||
           ieq_str(src, "BD") ||
           ieq_str(src, "WI") ||
           ieq_str(src, "WS") ||
           ieq_str(src, "WE") ||
           ieq_str(src, "WD");
}

} // anonymous namespace

// ============================ 3. 文本解析接口实现 ============================

bool parse_pd6_line(std::string_view line, ParsedLine& out)
{
    // 默认清空输出
    out = ParsedLine{};

    if (line.empty()) {
        return false;
    }

    line = trim(line);

    // 去掉首尾引号（离线 CSV 里常见）
    if (line.size() >= 2 && line.front() == '"' && line.back() == '"') {
        line = line.substr(1, line.size() - 2);
        line = trim(line);
    }

    if (line.empty()) {
        return false;
    }

    // 找到 ':'，后面才是 PD6/EPD6 内容
    const std::size_t pos_colon = line.find(':');
    if (pos_colon == std::string::npos) {
        return false;
    }

    std::string_view content = line.substr(pos_colon + 1); // 去掉前导 ':'
    content                   = trim(content);
    if (content.empty()) {
        return false;
    }

    // 切割 token
    std::array<std::string_view, 16> tok{};
    const std::size_t ntok = split_tokens(content, tok);
    if (ntok == 0) {
        return false;
    }

    std::string_view src_sv = trim(tok[0]);
    if (src_sv.empty()) {
        return false;
    }

    std::string src{src_sv};
    std::transform(src.begin(), src.end(), src.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });

    if (!is_supported_src(src)) {
        // 非 BI/BS/BE/BD/WI/WS/WE/WD，不视作 PD6/EPD6 速度/位移帧
        return false;
    }

    out.src = src;

    // timestamp_s 暂时不从 TS 帧推导，这里保持 0
    out.timestamp_s = 0.0;

    // 根据 src 分派
    if (src == "BD" || src == "WD") {
        return parse_displacement_frame(src, tok, ntok, out);
    } else {
        return parse_velocity_frame(src, tok, ntok, out);
    }
}

// ============================ 4. 命令打包接口实现 ============================
CommandBytes make_command_from_ascii(std::string_view ascii_cmd)
{
    CommandBytes bytes{};

    // 默认全部填充为 ASCII '0'，符合手册中 "PR10\r000000000000" 的风格
    bytes.fill(static_cast<std::uint8_t>('0'));

    // 去掉首尾空白和 CR/LF
    auto trim = [](std::string_view s) -> std::string_view {
        std::size_t beg = 0;
        std::size_t end = s.size();
        while (beg < end &&
               (s[beg] == ' ' || s[beg] == '\t' ||
                s[beg] == '\r' || s[beg] == '\n')) {
            ++beg;
        }
        while (end > beg &&
               (s[end - 1] == ' ' || s[end - 1] == '\t' ||
                s[end - 1] == '\r' || s[end - 1] == '\n')) {
            --end;
        }
        return s.substr(beg, end - beg);
    };

    ascii_cmd = trim(ascii_cmd);

    // 预留 1 字节给 '\r'：ASCII 部分最长 15
    const std::size_t max_ascii_len = 15;
    const std::size_t ascii_len = std::min<std::size_t>(ascii_cmd.size(), max_ascii_len);

    // 拷贝 ASCII 命令体
    for (std::size_t i = 0; i < ascii_len; ++i) {
        bytes[i] = static_cast<std::uint8_t>(ascii_cmd[i]);
    }

    // 在 ASCII 命令体后面放一个 '\r'
    if (ascii_len < bytes.size()) {
        bytes[ascii_len] = static_cast<std::uint8_t>('\r');
        // 之后的字节保持为 '0'，前面 fill() 已经处理
    } else {
        // ascii_len == 16 的极端情况：被截断，最后一个字节就是命令最后一位，
        // 没有多余空间放 '\r'，这种情况一般不会出现，除非命令体异常长。
        // 如有需要，可以在这里打印一条调试日志。
    }

    return bytes;
}

// ---- CS / CZ：启动 / 停止 ----

CommandBytes make_command_cz()
{
    // 强制停止 ping
    return make_command_from_ascii("CZ");
}

CommandBytes make_command_cs()
{
    // 启动 ping（不带参数）
    return make_command_from_ascii("CS");
}

// ---- PR / PM：发射呼率 / 平均次数 ----

std::string make_pr_ascii(int ping_rate)
{
    // 协议格式：PR<数字>，例如 "PR10"
    std::string s;
    s.reserve(8);
    s += "PR";
    s += std::to_string(ping_rate);
    return s;
}

std::string make_pm_ascii(int avg_count)
{
    // 协议格式：PM<数字>，例如 "PM10"
    std::string s;
    s.reserve(8);
    s += "PM";
    s += std::to_string(avg_count);
    return s;
}

CommandBytes make_command_pr(int ping_rate)
{
    const std::string ascii = make_pr_ascii(ping_rate);
    return make_command_from_ascii(ascii);
}

CommandBytes make_command_pm(int avg_count)
{
    const std::string ascii = make_pm_ascii(avg_count);
    return make_command_from_ascii(ascii);
}


// ============================ 5. 命令回显 / 确认检测实现 ============================

std::string normalize_command_ascii(std::string_view cmd)
{
    cmd = trim(cmd);

    // 如果开头是 ':'，去掉（DVL 可能回显成 ":CS 10 10"）
    if (!cmd.empty() && cmd.front() == ':') {
        cmd.remove_prefix(1);
        cmd = trim(cmd);
    }

    std::string out;
    out.reserve(cmd.size());

    auto is_ws = [](char c) noexcept {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n';
    };

    bool pending_space = false;
    for (char ch : cmd) {
        if (is_ws(ch)) {
            // 折叠所有空白为单一空格（稍后在遇到下一个非空白时再插入）
            pending_space = true;
            continue;
        }

        if (pending_space && !out.empty()) {
            out.push_back(' ');
        }
        pending_space = false;

        out.push_back(static_cast<char>(
            std::toupper(static_cast<unsigned char>(ch))));
    }

    // 末尾不需要额外处理：我们只在遇到非空白字符前插入空格

    return out;
}

bool is_command_echo(std::string_view sent_ascii_cmd,
                     std::string_view echoed_line)
{
    const std::string norm_sent  = normalize_command_ascii(sent_ascii_cmd);
    const std::string norm_echo  = normalize_command_ascii(echoed_line);

    if (norm_sent.empty() || norm_echo.empty()) {
        return false;
    }

    return norm_sent == norm_echo;
}

bool is_cs_command_echo(int ping_rate, int avg_count, std::string_view line)
{
    // 当前协议层 make_command_cs() 已不再接收参数，ping_rate/avg_count 先忽略
    (void)ping_rate;
    (void)avg_count;

    const CommandBytes bytes = make_command_cs();

    // 假设命令是 ASCII 文本（一般 DVL 控制命令是文本串）
    std::string ascii;
    ascii.reserve(bytes.size());
    for (auto b : bytes) {
        ascii.push_back(static_cast<char>(b));
    }

    // 直接比较 echo 行和期望命令
    // 如果日志里有 \r\n，可以先 trim 掉末尾的 CR/LF
    std::string_view trimmed = line;
    while (!trimmed.empty() && (trimmed.back() == '\r' || trimmed.back() == '\n')) {
        trimmed.remove_suffix(1);
    }

    while (!ascii.empty() && (ascii.back() == '\r' || ascii.back() == '\n')) {
        ascii.pop_back();
    }

    return trimmed == ascii;
}

} // namespace nav_core::dvl_protocol
