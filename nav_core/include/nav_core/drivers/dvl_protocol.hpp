// nav_core/include/nav_core/dvl_protocol.hpp
//
// @file  dvl_protocol.hpp
// @brief Hover H1000 (PD6 / EPD6) 文本协议工具：
//        - 单行报文解析（PD6/EPD6 → 结构化字段 ParsedLine）
//        - 命令打包（ASCII 命令 → 16 字节固定长度帧）
//        - 命令回显检测（DVL 是否返回了相同命令）
//
// 设计定位：
//   - 完全「无 IO」：不依赖串口 / 线程 / 时间戳，只处理字符串与字节数组；
//   - 可在 nav_core 驱动、离线解析工具、单元测试中复用；
//   - 所有协议细节（帧类型 → 坐标系、底跟踪/水团分类、A/V 语义、+88888 占位等）
//     统一在此处维护，避免复制到 DvlDriver 等其他模块中；
//   - 工程级过滤策略（噪声门限、ZUPT 等）不在此处实现，而交由 estimator / math.hpp。
//

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <string_view>

namespace nav_core::dvl_protocol {

// ============================ 1. 基本枚举 ============================

/// @brief DVL 观测类型：底跟踪 / 水团 / 未知。
///
/// 典型映射（Hover H1000 PD6/EPD6）：
///   - BottomTrack:
///       * BI / BS / BE / BD
///   - WaterTrack:
///       * WI / WS / WE / WD
enum class TrackType : std::uint8_t {
    Unknown     = 0,
    BottomTrack = 1,
    WaterTrack  = 2,
};

/// @brief 报文中速度 / 位移所在的坐标系。
///
/// 典型映射（Hover H1000 PD6/EPD6）：
///   - Instrument frame（仪器坐标系）：
///       * BI / WI
///   - Ship/body frame（船体/机体坐标系）：
///       * BS / WS
///   - Earth/ENU frame（地理/导航坐标系）：
///       * BE / WE / BD / WD
enum class CoordFrame : std::uint8_t {
    Unknown    = 0,
    Instrument = 1,   ///< 仪器（探头）坐标系
    Ship       = 2,   ///< 船体 / 机体坐标系
    Earth      = 3,   ///< 地理 / 导航 ENU 坐标系
};


// ============================ 2. 单行解析结果 ============================

/**
 * @brief 单行 PD6 / EPD6 文本报文的解析结果（协议语义层）。
 *
 * 说明：
 *   - 不带时间戳（时间信息由上层 DvlDriver 在读取串口时补充）；
 *   - 不做任何“工程过滤”（例如速度阈值、噪声地板），只负责还原协议语义；
 *   - +88888 / 空字段等占位值，由实现约定转换为具体数值 / 标志。
 */
struct ParsedLine {
    // ---- 报文元信息 ----
    std::string src;             ///< 帧类型字符串，如 "BI" / "BS" / "BE" / "BD" / "WI" 等

    TrackType   track_type{TrackType::Unknown};
    CoordFrame  coord_frame{CoordFrame::Unknown};

    // DVL 内部时间戳（若报文中带有时间字段；没有则为 0）
    double timestamp_s{0.0};

    // ---- 速度（当前帧坐标系下的三分量）----
    //
    // 对 CoordFrame::Earth：
    //   - ve_mm/vn_mm/vu_mm 对应 ENU 三轴速度 [vE, vN, vU]，单位 mm/s；
    //
    // 对 Instrument / Ship：
    //   - ve_mm/vn_mm/vu_mm 对应该帧坐标系下的三轴速度分量（Body/Ship XYZ），
    //     上层若需要 ENU 速度，应结合 IMU 姿态单独旋转。
    //
    double ve_mm{0.0};
    double vn_mm{0.0};
    double vu_mm{0.0};

    // ---- 深度与累计位移（若该帧类型包含）----
    //
    // 典型映射：
    //   - BD / WD： e_m, n_m, u_m 为 ENU 下的累计位移 [E, N, U] (m)；
    //   - 某些帧包含深度（下为正），由 depth_m 提供。
    //
    double depth_m{0.0};
    double e_m{0.0};
    double n_m{0.0};
    double u_m{0.0};

    // ---- 有效性与质量 ----
    //
    // valid:
    //   - 1 : 'A'（valid，可用）
    //   - 0 : 'V'（invalid，不可信）
    //   - -1: 未设置 / 报文中不存在该字段
    //
     // ==== 新增：各分量是否有效 ====
    bool has_e     = false;
    bool has_n     = false;
    bool has_u     = false;
    bool has_depth = false;
    int   valid{-1};

    // 质量指标（例如原始 FOM 或其转换结果，具体含义由实现约定）
    int   quality{0};

    // 是否锁底成功（若报文中可推断。对纯水团帧通常为 false）
    bool  bottom_lock{false};


    // ---- 工具函数（便捷访问）----

    [[nodiscard]] double ve_mps() const noexcept { return ve_mm * 1e-3; }
    [[nodiscard]] double vn_mps() const noexcept { return vn_mm * 1e-3; }
    [[nodiscard]] double vu_mps() const noexcept { return vu_mm * 1e-3; }

    [[nodiscard]] bool is_bottom_track() const noexcept {
        return track_type == TrackType::BottomTrack;
    }

    [[nodiscard]] bool is_water_track() const noexcept {
        return track_type == TrackType::WaterTrack;
    }
};

// ============================ 3. 文本解析接口 ============================

/**
 * @brief 解析一行 PD6 / EPD6 文本报文。
 *
 * @param line   输入文本行（不含结尾 '\n'，是否含 '\r' 由实现内部容忍处理）
 * @param out    输出：解析后的协议字段（src / track_type / 坐标系 / 速度 / 位移等）
 *
 * 约定：
 *   - 本函数只做「协议语义还原」，不做工程级过滤；
 *   - 若报文格式不符合 PD6 / EPD6 规范，返回 false，out 内容未定义；
 *   - 对于占位值（如 +88888），实现可按约定转换为数值或 NaN 等；
 *   - 时间戳（mono_ns / est_ns）不在此处处理，由 DvlDriver 在读取串口时补充。
 *
 * @return true  解析成功
 * @return false 解析失败（非法格式 / 不支持的帧类型等）
 */
[[nodiscard]] bool parse_pd6_line(std::string_view line, ParsedLine& out);


// ============================ 4. 命令打包接口 ============================

/// @brief 固定长度命令字节数组类型（Hover H1000 要求 16 字节命令）。
using CommandBytes = std::array<std::uint8_t, 16>;

/**
 * @brief DH1000 风格的 16 字节命令帧构造（与 Python 版 build_cmd16 对齐）。
 *
 * 约定：
 *   - cmd2：2 字符命令缩写（"CS" / "CZ" / "PR" / "PM" 等），不区分大小写；
 *   - arg ：可选参数字符串，未做范围检查：
 *       * 若为空（std::string_view{} 或 只含空白），则构造无参数命令：
 *           s = cmd2;
 *       * 否则构造有参数命令：
 *           s = cmd2 + " " + arg;
 *   - s 末尾追加 '\r'：
 *           s += '\r';
 *   - 若此时 s 长度 > 16，视为错误（抛异常或在实现中返回全 0 并记录错误）；
 *   - 若 s 长度 <= 16，则用字符 '0' 右侧填充至 16 字节。
 *
 * 最终：
 *   - 返回长度严格为 16 的字节数组，可直接写入串口。
 *
 * 典型示例：
 *   - build_command16("CS", {})      -> "CS\\r0000000000000"
 *   - build_command16("CZ", {})      -> "CZ\\r0000000000000"
 *   - build_command16("PR", " 1")    -> "PR 1\\r000000000000"
 *   - build_command16("PR", "10")    -> "PR 10\\r0000000000"
 */
[[nodiscard]] CommandBytes build_command16(std::string_view cmd2,
                                           std::string_view arg);

/**
 * @brief 构造 PR 命令参数字符串（两位宽整数，前导空格），例如 " 1"、"10"。
 *
 * 约定：
 *   - ping_rate 必须在 [0, 99] 区间内；
 *   - 返回格式固定为 2 字符宽度：
 *       1  -> " 1"
 *       10 -> "10"
 *
 * 说明：
 *   - 这是 Python 版 fmt_int_2d 的 C++ 等价物；
 *   - 仅负责格式化整数，不负责追加命令/回车/填充。
 */
[[nodiscard]] std::string format_pr_pm_arg_2d(int value);

/**
 * @brief 生成 CZ 命令（停机），语义等价于 Python safe_start 里的 "CZ" 步骤。
 *
 * 实际发送内容：
 *   - build_command16("CZ", {})
 */
[[nodiscard]] CommandBytes make_command_cz();

/**
 * @brief 生成 CS 命令（启动 ping），语义等价于 Python safe_start 里的 "CS" 步骤。
 *
 * 实际发送内容：
 *   - build_command16("CS", {})
 */
[[nodiscard]] CommandBytes make_command_cs();

/**
 * @brief 生成 PR 命令（设置发射呼率）。
 *
 * 实际发送内容：
 *   - build_command16("PR", format_pr_pm_arg_2d(ping_rate));
 *
 * @param ping_rate   呼率值（0..99），具体物理含义按说明书定义
 */
[[nodiscard]] CommandBytes make_command_pr(int ping_rate);

/**
 * @brief 生成 PM 命令（设置平均次数）。
 *
 * 实际发送内容：
 *   - build_command16("PM", format_pr_pm_arg_2d(avg_count));
 *
 * @param avg_count   平均次数（0..99）
 */
[[nodiscard]] CommandBytes make_command_pm(int avg_count);


/**
 * @brief 返回“人类可读”的命令 ASCII 表示（不含 '\r'），用于日志和 echo 比较。
 *
 * 约定：
 *   - 无参数命令："CS" / "CZ"
 *   - 带参数命令："PR 10" / "PM  5"（包含空格）
 *
 * 说明：
 *   - 这只是日志/比较用字符串，不参与 16 字节填充；
 *   - 实际串口发送仍使用 build_command16 生成的 CommandBytes。
 */
[[nodiscard]] std::string make_ascii_cmd(std::string_view cmd2,
                                         std::string_view arg);

// ============================ 5. 命令回显 / 确认检测 ============================

/**
 * @brief 规范化命令 ASCII 字符串（用于比较），与 Python normalize 行为对齐。
 *
 * 建议实现规则：
 *   - 去掉首尾空白（空格 / '\t' / '\r' / '\n' 等）；
 *   - 将连续空格折叠为单个空格（可选）；
 *   - 字母统一转为大写；
 *   - 不处理 16 字节填充，仅针对“人类可读”的命令部分。
 *
 * 典型用法：
 *   - normalize_ascii("PR 10")        -> "PR 10"
 *   - normalize_ascii("  pr  10\r")   -> "PR 10"
 */
[[nodiscard]] std::string normalize_command_ascii(std::string_view cmd);

/**
 * @brief 检查回传行是否“看起来是”指定命令的 echo。
 *
 * 典型实现思路：
 *   - 对 sent_ascii_cmd 调用 normalize_command_ascii，得到 norm_sent；
 *   - 对 echoed_line 内部尝试提取类似命令片段（例如去掉前后杂字符），
 *     调用 normalize_command_ascii 得到 norm_echo；
 *   - 若 norm_echo 中包含 norm_sent 子串，或者相等，则认为“通过 echo 检查”。
 *
 * 典型用法（在 DvlDriver 中）：
 *   - 发送 PR 命令时记录 ascii："PR 10"；
 *   - 收到来自 DVL 的 RESP 行时，调用 is_command_echo("PR 10", line)；
 *   - 若返回 true，即认为 DVL 已接受并回显该命令。
 */
[[nodiscard]] bool is_command_echo(std::string_view sent_ascii_cmd,
                                   std::string_view echoed_line);

/**
 * @brief CS 命令的专用 echo 检查。
 *
 * 说明：
 *   - 等价于 is_command_echo("CS", echoed_line)；
 *   - 由于 CS 不带数值参数，因此不涉及格式化/空格。
 */
[[nodiscard]] bool is_cs_command_echo(std::string_view echoed_line);


} // namespace nav_core::dvl_protocol
