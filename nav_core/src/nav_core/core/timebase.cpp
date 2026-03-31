// nav_core/src/timebase.cpp

#include "nav_core/core/timebase.hpp"

namespace nav_core::core::timebase {  // ★ 这里改成 nav_core

MonoTimeNs now_ns()
{
    const auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

TimePoint now()
{
    return Clock::now();
}

// 全局默认延迟
static LatencyDefaults g_latency_defaults{};

LatencyDefaults& latency_defaults()
{
    return g_latency_defaults;
}

static std::int64_t default_latency_ns(SensorKind kind)
{
    const auto& d = g_latency_defaults;
    switch (kind) {
    case SensorKind::IMU:  return d.imu_ns;
    case SensorKind::DVL:  return d.dvl_ns;
    case SensorKind::USBL: return d.usbl_ns;
    default:               return d.other_ns;
    }
}

Stamp stamp(const std::string&          sensor_id,
            SensorKind                  kind,
            std::optional<MonoTimeNs>   sensor_time_ns,
            std::optional<std::int64_t> latency_ns)
{
    Stamp s;
    s.sensor_id      = sensor_id;
    s.kind           = kind;
    s.host_time_ns   = now_ns();
    s.sensor_time_ns = sensor_time_ns;

    s.latency_ns = latency_ns.has_value()
        ? *latency_ns
        : default_latency_ns(kind);

    if (s.sensor_time_ns.has_value()) {
        s.corrected_time_ns = *s.sensor_time_ns;
    } else {
        s.corrected_time_ns = s.host_time_ns - s.latency_ns;
    }
    return s;
}

} // namespace nav_core::core::timebase
