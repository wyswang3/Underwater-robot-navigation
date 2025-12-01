#include "nav_core/timebase.h"

namespace uwnav::timebase {

int64_t now_ns() {
    auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

TimePoint now() {
    return Clock::now();
}

// =============================
// 默认延迟（全局静态）
// =============================
static LatencyDefaults g_latency_defaults{};

LatencyDefaults& latency_defaults() {
    return g_latency_defaults;
}

// =============================
// 根据传感器类型返回默认延迟
// =============================
static int64_t default_latency_ns(SensorKind kind) {
    const auto& d = g_latency_defaults;
    switch (kind) {
    case SensorKind::IMU:  return d.imu_ns;
    case SensorKind::DVL:  return d.dvl_ns;
    case SensorKind::USBL: return d.usbl_ns;
    default:               return d.other_ns;
    }
}

// =============================
// stamp(): 统一时间戳核心方法
// =============================
Stamp stamp(
    const std::string& sensor_id,
    SensorKind kind,
    std::optional<int64_t> sensor_time_ns,
    std::optional<int64_t> latency_ns
) {
    Stamp s;
    s.sensor_id      = sensor_id;
    s.kind           = kind;
    s.host_time_ns   = now_ns();
    s.sensor_time_ns = sensor_time_ns;

    // 1) 选择延迟：参数优先，否则使用默认值
    s.latency_ns = latency_ns.has_value()
        ? *latency_ns
        : default_latency_ns(kind);

    // 2) 统一后的 corrected 时间戳
    s.corrected_time_ns = s.host_time_ns - s.latency_ns;

    // 未来你如果想加入 sensor_time_ns 对齐逻辑，可以在这里扩展：
    // if (sensor_time_ns.has_value()) {
    //     TODO：添加 offset 估计 + 时钟对齐
    // }

    return s;
}

} // namespace uwnav::timebase
