// nav_core/src/dvl_protocol.cpp
//
// Hover H1000 (PD6 / EPD6) 文本协议工具实现：
//   - 单行报文解析（PD6/EPD6 → ParsedLine）
//   - 命令打包（CMD2/ARG → 16 字节固定帧）
//   - 命令回显检测（DVL 是否返回了相同命令）
//
// 说明：
//   - 本文件不依赖任何 IO（串口/线程），仅处理字符串和字节数组；
//   - 不维护历史状态（无“上一帧 BD”之类），位移/深度的“沿用上一帧”
//     逻辑由 DvlDriver / estimator 实现；
//   - 速度维度若缺失或出现占位值（+8888 / +88888 / 9999 等），统一以 0 填充，
//     保证数据链不断；
//   - 浮点解析为 NaN / Inf 时一律视作失败，不向下游传播 NaN。

#include <nav_core/drivers/dvl_protocol.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cerrno>
#include <cstdlib>
#include <string>

namespace nav_core::dvl_protocol {

namespace {

// -------------------- 字符串小工具 --------------------

inline std::string_view trim(std::string_view sv) noexcept
{
    while (!sv.empty() &&
           std::isspace(static_cast<unsigned char>(sv.front()))) {
        sv.remove_prefix(1);
    }
    while (!sv.empty() &&
           std::isspace(static_cast<unsigned char>(sv.back()))) {
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

// -------------------- token / 数字解析 --------------------

/// @brief 判断一个 token 是否是典型占位符（+8888 / 8888 / 9999 / 88888 / 99999）。
inline bool is_placeholder_token(std::string_view sv) noexcept
{
    sv = trim(sv);
    if (sv.empty()) return true;  // 空视作“没有实值”

    // 去掉前导符号
    if (!sv.empty() && (sv.front() == '+' || sv.front() == '-')) {
        sv.remove_prefix(1);
        sv = trim(sv);
    }

    // 常见占位形式：8888 / 88888 / 9999 / 99999
    if (sv == "8888" || sv == "88888" ||
        sv == "9999" || sv == "99999")
    {
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

    std::string tmp{sv};
    char* endp = nullptr;
    errno      = 0;
    const double v = std::strtod(tmp.c_str(), &endp);

    if (endp == tmp.c_str() || errno == ERANGE) {
        return false;
    }
    if (!std::isfinite(v)) {
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
        if (pos == std::string::npos) {
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
        // 连 src 之外没有任何数值，不认为是合法速度帧
        return false;
    }

    // 速度 token 约定：token[1], token[2], token[3]
    // 最后一个 token 用作 A/V 状态（BI 有额外一个 range 字段，状态仍然在最后）
    const std::string_view t_ve = (ntok > 1) ? tok[1] : std::string_view{};
    const std::string_view t_vn = (ntok > 2) ? tok[2] : std::string_view{};
    const std::string_view t_vu = (ntok > 3) ? tok[3] : std::string_view{};

    int ve_i = 0, vn_i = 0, vu_i = 0;

    // 速度：缺失 / 占位 → 0，避免 NaN 传到滤波器
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

    out.bottom_lock =
        (out.track_type == TrackType::BottomTrack && out.valid == 1);

    // 质量字段对纯速度帧暂不解析，保持 out.quality = 0
    return true;
}

// -------------------- 位移 / 深度帧解析：BD / WD --------------------

bool parse_displacement_frame(const std::string& src,
                              const std::array<std::string_view, 16>& tok,
                              std::size_t ntok,
                              ParsedLine& out)
{
    // 预期：src, e, n, u, depth, quality
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
    out.bottom_lock =
        (out.track_type == TrackType::BottomTrack && out.valid == 1);

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

    // DVL 内部时间戳（TS 帧），此处不从该行推导，保持 0，由上层统一打时间基
    out.timestamp_s = 0.0;

    // 根据 src 分派
    if (src == "BD" || src == "WD") {
        return parse_displacement_frame(src, tok, ntok, out);
    } else {
        return parse_velocity_frame(src, tok, ntok, out);
    }
}

// ============================ 4. 命令打包接口实现 ============================

CommandBytes build_command16(std::string_view cmd2, std::string_view arg)
{
    CommandBytes bytes{};
    // 默认填充 ASCII '0'
    bytes.fill(static_cast<std::uint8_t>('0'));

    cmd2 = trim(cmd2);
    arg  = trim(arg);

    // 命令 2 字符，统一大写
    std::string cmd2_up;
    cmd2_up.reserve(2);
    for (char c : cmd2) {
        cmd2_up.push_back(static_cast<char>(
            std::toupper(static_cast<unsigned char>(c))));
    }

    std::string s;
    s.reserve(16);
    s += cmd2_up;
    if (!arg.empty()) {
        s.push_back(' ');
        s += std::string(arg);
    }
    s.push_back('\r');

    // 限制总长不超过 16
    if (s.size() > 16) {
        s.resize(16);
    }

    const std::size_t n = std::min<std::size_t>(s.size(), bytes.size());
    for (std::size_t i = 0; i < n; ++i) {
        bytes[i] = static_cast<std::uint8_t>(s[i]);
    }

    return bytes;
}

std::string format_pr_pm_arg_2d(int value)
{
    if (value < 0)   value = 0;
    if (value > 99)  value = 99;

    std::string s = std::to_string(value);
    if (s.size() == 1) {
        // 前导空格 + 一位数
        return std::string(" ") + s;
    }
    if (s.size() == 2) {
        return s;
    }
    // 理论上不会到这里（前面已经 clamp 0..99），防御性兜底
    return s.substr(s.size() - 2);
}

CommandBytes make_command_cz()
{
    return build_command16("CZ", std::string_view{});
}

CommandBytes make_command_cs()
{
    return build_command16("CS", std::string_view{});
}

CommandBytes make_command_pr(int ping_rate)
{
    const std::string arg = format_pr_pm_arg_2d(ping_rate);
    return build_command16("PR", arg);
}

CommandBytes make_command_pm(int avg_count)
{
    const std::string arg = format_pr_pm_arg_2d(avg_count);
    return build_command16("PM", arg);
}

std::string make_ascii_cmd(std::string_view cmd2, std::string_view arg)
{
    cmd2 = trim(cmd2);
    arg  = trim(arg);

    std::string out;
    out.reserve(8);

    for (char c : cmd2) {
        out.push_back(static_cast<char>(
            std::toupper(static_cast<unsigned char>(c))));
    }

    if (!arg.empty()) {
        out.push_back(' ');
        out.append(arg.begin(), arg.end());
    }
    return out;
}

// ============================ 5. 命令回显 / 确认检测实现 ============================

std::string normalize_command_ascii(std::string_view cmd)
{
    cmd = trim(cmd);

    // 去掉前导 ':'（RESP 里常见 ":::PR 10..."）
    while (!cmd.empty() && cmd.front() == ':') {
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
            // 折叠所有空白为单一空格
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

    return out;
}

bool is_command_echo(std::string_view sent_ascii_cmd,
                     std::string_view echoed_line)
{
    const std::string norm_sent = normalize_command_ascii(sent_ascii_cmd);
    const std::string norm_echo = normalize_command_ascii(echoed_line);

    if (norm_sent.empty() || norm_echo.empty()) {
        return false;
    }

    // 对于 ":::PR 10\00\00..." 这种 RESP，这里用“包含关系”更稳妥
    return norm_echo.find(norm_sent) != std::string::npos;
}

bool is_cs_command_echo(std::string_view echoed_line)
{
    return is_command_echo("CS", echoed_line);
}

} // namespace nav_core::dvl_protocol
