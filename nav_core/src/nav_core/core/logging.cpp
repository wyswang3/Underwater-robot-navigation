// nav_core/src/logging.cpp
#include "nav_core/core/logging.hpp"

#include <atomic>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

namespace nav_core::logging {

namespace {

// 全局日志级别（默认 Info）
std::atomic<LogLevel> g_log_level{LogLevel::Info};

// 输出时的互斥锁，确保多线程下行不穿插
std::mutex g_log_mutex;

// 简单的本地时间戳字符串，如 "2026-01-08 15:23:01"
std::string make_timestamp()
{
    using namespace std::chrono;
    auto now = system_clock::now();
    auto t   = system_clock::to_time_t(now);

    std::tm tm_buf{};
#if defined(_WIN32)
    localtime_s(&tm_buf, &t);
#else
    localtime_r(&t, &tm_buf);
#endif

    std::ostringstream oss;
    oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S");
    return oss.str();
}

} // anonymous namespace

void set_log_level(LogLevel lvl) noexcept
{
    g_log_level.store(lvl, std::memory_order_relaxed);
}

LogLevel log_level() noexcept
{
    return g_log_level.load(std::memory_order_relaxed);
}

const char* to_string(LogLevel lvl) noexcept
{
    switch (lvl) {
    case LogLevel::Trace: return "TRACE";
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info:  return "INFO";
    case LogLevel::Warn:  return "WARN";
    case LogLevel::Error: return "ERROR";
    default:              return "UNKNOWN";
    }
}

void log(LogLevel lvl, std::string_view msg)
{
    // 1) 级别过滤
    if (static_cast<std::uint8_t>(lvl) < static_cast<std::uint8_t>(log_level())) {
        return;
    }

    // 2) 构造一行日志并输出
    std::lock_guard<std::mutex> lock(g_log_mutex);

    std::cerr << "[" << make_timestamp() << "]"
              << " [" << to_string(lvl) << "] "
              << msg << std::endl;
}

} // namespace nav_core::logging
