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
 * @brief 将 ASCII 命令打包为 Hover H1000 所需的 16 字节命令。
 *
 * 规则（与厂家要求一致的工程约定）：
 *   - 输入 ascii_cmd 为“人类可读命令本体”，例如 "CS"、"CZ"、"PR10"、"PM10"；
 *   - 实际发送内容为：
 *       [ ascii_cmd 字符... ][ '\r' ][ 0x00 填充到 16 字节 ];
 *   - 若 ascii_cmd 本身包含空白或 '\r' / '\n'，实现会先做 trim，再追加 '\r'。
 *
 * 说明：
 *   - 调用方只需关心“逻辑命令”字符串；
 *   - 本函数保证返回的数组长度始终为 16，可直接写入串口。
 *
 * @param ascii_cmd  人类可读命令字符串（不带结尾 '\r'）
 * @return           固定长度 16 字节命令缓冲区，可直接写入串口。
 */
[[nodiscard]] CommandBytes make_command_from_ascii(std::string_view ascii_cmd);

/**
 * @brief 生成 CZ 命令（强制停机）。
 *
 * 典型用途：
 *   - 上电后先发送 CZ，确保 DVL 不会在空气中发 ping；
 *   - 程序退出 / 机器人上岸前再次发送 CZ，保证设备安全。
 *
 * 约定：
 *   - 实际发送内容等价于 "CZ" + '\r' + 0 填充。
 */
[[nodiscard]] CommandBytes make_command_cz();

/**
 * @brief 生成 CS 命令（启动 ping）。
 *
 * 说明：
 *   - 实际协议中 CS 不带数值参数，仅表示“开始工作”；
 *   - 呼率与平均次数通过 PR/PM 命令单独设置。
 *
 * 约定：
 *   - 实际发送内容等价于 "CS" + '\r' + 0 填充。
 */
[[nodiscard]] CommandBytes make_command_cs();

/**
 * @brief 构造 PR 命令的 ASCII 形式，例如 "PR10"。
 *
 * 说明：
 *   - 协议要求格式为指令 + 数字，无逗号，即 "PR10"；
 *   - 本函数仅返回 ASCII 部分（不含 '\r'），方便日志 / 调试或进一步打包。
 *
 * @param ping_rate   呼率参数（例如 10）
 * @return            ASCII 命令字符串（如 "PR10"）
 */
[[nodiscard]] std::string make_pr_ascii(int ping_rate);

/**
 * @brief 构造 PM 命令的 ASCII 形式，例如 "PM10"。
 *
 * 说明与 make_pr_ascii 相同。
 *
 * @param avg_count   平均次数（例如 10）
 * @return            ASCII 命令字符串（如 "PM10"）
 */
[[nodiscard]] std::string make_pm_ascii(int avg_count);

/**
 * @brief 生成 PR 命令（设置发射呼率）。
 *
 * 实现约定：
 *   - 等价于 make_command_from_ascii(make_pr_ascii(ping_rate))。
 */
[[nodiscard]] CommandBytes make_command_pr(int ping_rate);

/**
 * @brief 生成 PM 命令（设置平均次数）。
 *
 * 实现约定：
 *   - 等价于 make_command_from_ascii(make_pm_ascii(avg_count))。
 */
[[nodiscard]] CommandBytes make_command_pm(int avg_count);


// ============================ 5. 命令回显 / 确认检测 ============================

/**
 * @brief 规范化命令 ASCII 字符串（用于比较）。
 *
 * 建议处理规则（在 .cpp 中实现）：
 *   - 去掉首尾空白字符（空格、'\r'、'\n'、'\t' 等）；
 *   - 将连续空格折叠为单个空格（如果需要支持带空格命令）；
///   - 可选：将字母统一转为大写，避免大小写差异；
///   - 不做 16 字节填充，仅针对“人类可读”的命令行。
 *
 * 用途：
///   - 用于比较“发送的 ASCII 命令”和“DVL 回显的命令行”是否等价；
///   - 也可单独用来打印或记录规范化格式。
 *
 * @param cmd   原始命令字符串（发送时或回显行中的片段）
 * @return      规范化后的命令字符串
 */
[[nodiscard]] std::string normalize_command_ascii(std::string_view cmd);

/**
 * @brief 检查回传行是否与给定命令等价（通用版本）。
 *
 * 典型实现思路：
 *   - 对 sent_ascii_cmd 和 echoed_line 分别调用 normalize_command_ascii；
 *   - 比较规范化后字符串是否完全相等；
 *   - 如需忽略 echoed_line 中前缀/后缀，可在实现中先截取命令部分再比较。
 *
 * 典型用法（在 DvlDriver 中）：
 *   - 发送命令前记录 sent = "CS" / "CZ" / "PR10" / "PM10"；
 *   - 后台线程解析到一行非 PD6/EPD6 文本时，调用 is_command_echo(sent, line)，
 *     若返回 true，则认为该命令已被 DVL 接收并回显。
 *
 * @param sent_ascii_cmd   本地构造的命令 ASCII（如 "CS"、"PR10"）
 * @param echoed_line      从 DVL 串口读回的一整行文本
 * @return true            若认为是同一命令的回显
 * @return false           否则
 */
[[nodiscard]] bool is_command_echo(std::string_view sent_ascii_cmd,
                                   std::string_view echoed_line);

/**
 * @brief CS 命令专用的回显检查工具。
 *
 * 说明：
 *   - 内部可直接实现为：
 *       return is_command_echo("CS", echoed_line);
 *   - 注意：CS 命令不带数值参数，因此无需 ping_rate / avg_count。
 *
 * @param echoed_line DVL 回传的一行文本
 * @return true       若认为 DVL 回显了 CS 命令
 * @return false      否则
 */
[[nodiscard]] bool is_cs_command_echo(std::string_view echoed_line);

} // namespace nav_core::dvl_protocol
