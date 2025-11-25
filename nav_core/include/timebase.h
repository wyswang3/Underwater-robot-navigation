// timebase.h
#pragma once
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

namespace uwnav::timebase {

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = Clock::duration;

// 1. 基础时间接口（保持不变）
int64_t  now_ns();
TimePoint now();

// 2. 传感器类型（精简版）
enum class SensorKind {
    IMU,
    DVL,
    USBL,
    OTHER
};

// 3. 统一的时间戳结构
struct Stamp {
    std::string   sensor_id;        // "imu0", "dvl0", "usbl0"
    SensorKind    kind;

    int64_t       host_time_ns;     // 主机单调时间（数据到达/解析完成瞬间）
    std::optional<int64_t> sensor_time_ns; // 传感器内部时间，可无

    int64_t       latency_ns;       // 估计延迟
    int64_t       corrected_time_ns;// 统一后的时间戳（给 ESKF/Logger 用）
};

// 4. 默认延迟参数（可以写死，也可以将来从配置加载）
struct LatencyDefaults {
    int64_t imu_ns  = 2'000'000;     // 2 ms
    int64_t dvl_ns  = 50'000'000;    // 50 ms
    int64_t usbl_ns = 150'000'000;   // 150 ms
    int64_t other_ns= 0;
};

// 5. 全局配置的 getter/setter（先用简单静态变量）
LatencyDefaults& latency_defaults();

// 6. 统一打时间戳的工具函数（核心）
Stamp stamp(
    const std::string& sensor_id,
    SensorKind kind,
    std::optional<int64_t> sensor_time_ns = std::nullopt,
    std::optional<int64_t> latency_ns     = std::nullopt
);

} // namespace uwnav::timebase
