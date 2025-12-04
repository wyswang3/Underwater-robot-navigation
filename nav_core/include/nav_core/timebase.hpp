// include/nav_core/timebase.hpp
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "nav_core/types.hpp"

// 补一个别名：把 nav_core::MonoTimeNs 映射到 uwnav::core::MonoTimeNs
namespace uwnav::core {
using MonoTimeNs = nav_core::MonoTimeNs;
}

namespace uwnav::core::timebase {

// ============= 1. 基础时间接口 =============

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = Clock::duration;

// 单调时间（纳秒）
MonoTimeNs now_ns();

// 单调时间（time_point）
TimePoint now();

// ============= 2. 传感器类型 =============

enum class SensorKind {
    IMU,
    DVL,
    USBL,
    OTHER
};

// ============= 3. 统一时间戳结构 =============

struct Stamp {
    std::string   sensor_id;    // 如 "imu0", "dvl0", "usbl0"
    SensorKind    kind;

    // 数据到达/解析完成时的主机单调时间
    MonoTimeNs host_time_ns{0};

    // 传感器内部时间戳（如果有的话，可以是本地 tick 转换过来的纳秒）
    std::optional<MonoTimeNs> sensor_time_ns;

    // 估计的延迟（ns），正值表示“数据比 host_time 滞后”
    std::int64_t latency_ns{0};

    // 对外统一使用的时间戳（给 ESKF / Logger）：
    // 一般为 sensor_time_ns 校正后的结果；没有 sensor_time_ns 时，
    // 通常是 host_time_ns - latency_ns。
    MonoTimeNs corrected_time_ns{0};
};

// ============= 4. 默认延迟参数 =============

struct LatencyDefaults {
    std::int64_t imu_ns   = 2'000'000;      // 2 ms
    std::int64_t dvl_ns   = 50'000'000;     // 50 ms
    std::int64_t usbl_ns  = 150'000'000;    // 150 ms
    std::int64_t other_ns = 0;
};

// 全局默认值，可读可写（线程不安全版本，配置阶段使用即可）
LatencyDefaults& latency_defaults();

// ============= 5. 统一打时间戳工具函数 =============
//
// 调用方式：
//   auto st  = stamp("imu0", SensorKind::IMU);
//   auto st2 = stamp("dvl0", SensorKind::DVL, sensor_time_ns);
Stamp stamp(
    const std::string& sensor_id,
    SensorKind kind,
    std::optional<MonoTimeNs> sensor_time_ns = std::nullopt,
    std::optional<std::int64_t>             latency_ns     = std::nullopt
);

} // namespace uwnav::core::timebase

// ============= 6. 兼容旧命名空间 =============
//
// 原有代码如果用了：uwnav::timebase::stamp(...)
// 在这里做一个别名导出，保证不需要一次性大改历史代码。
namespace uwnav::timebase {
    using namespace uwnav::core::timebase;
}
