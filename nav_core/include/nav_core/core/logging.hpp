// nav_core/include/nav_core/logging.hpp
#pragma once

#include <cstdint>
#include <string>
#include <string_view>

namespace nav_core::logging {

/**
 * @brief 日志级别枚举
 *
 * 设计为“从低到高”的严重程度：
 *   Trace < Debug < Info < Warn < Error
 */
enum class LogLevel : std::uint8_t {
    Trace = 0,
    Debug = 1,
    Info  = 2,
    Warn  = 3,
    Error = 4,
};

/**
 * @brief 配置当前全局日志级别
 *
 * 说明：
 *   - 默认值在 logging.cpp 中给出（建议默认为 Info）；
 *   - 若设置为 Warn，则 Debug/Info 日志都会被抑制。
 */
void set_log_level(LogLevel lvl) noexcept;

/**
 * @brief 获取当前全局日志级别
 */
LogLevel log_level() noexcept;

/**
 * @brief 核心日志输出接口（线程安全）
 *
 * @param lvl  日志级别
 * @param msg  已经格式化好的字符串
 *
 * 说明：
 *   - 若 lvl < 当前全局 log_level，则本调用会被忽略；
 *   - 当前实现将输出到 stderr/std::clog，将来可以扩展为文件等。
 */
void log(LogLevel lvl, std::string_view msg);

/**
 * @brief 将日志级别转为字符串（如 "INFO"）
 *
 * 方便打印时使用，或者在工具里做文本化输出。
 */
const char* to_string(LogLevel lvl) noexcept;

} // namespace nav_core::logging

// ====================== 便捷宏（可选） ======================
//
// 说明：
//   - 这些宏只是为了少写一点枚举名；
//   - msg 建议传 std::string 或 std::string_view；
//   - 如需格式化，可以先用 fmt::format / std::ostringstream 拼出字符串。

#define NAV_LOG_TRACE(msg) ::nav_core::logging::log(::nav_core::logging::LogLevel::Trace, (msg))
#define NAV_LOG_DEBUG(msg) ::nav_core::logging::log(::nav_core::logging::LogLevel::Debug, (msg))
#define NAV_LOG_INFO(msg)  ::nav_core::logging::log(::nav_core::logging::LogLevel::Info,  (msg))
#define NAV_LOG_WARN(msg)  ::nav_core::logging::log(::nav_core::logging::LogLevel::Warn,  (msg))
#define NAV_LOG_ERROR(msg) ::nav_core::logging::log(::nav_core::logging::LogLevel::Error, (msg))
