// nav_core/src/timebase.cpp
//
// 时间基准与统一时间戳工具实现：uwnav::core::timebase
//
// 设计目标：
//   - 所有传感器（IMU / DVL / USBL / 其他）统一走 steady_clock 时间基；
//   - 通过一个简单的 Stamp 结构，把“主机时间 + 估计延迟”封装为 corrected_time_ns；
//   - 后端 ESKF / Logger / 跨进程 NavStatePublisher 都可以只认一个时间轴；
//   - 将来可以在这里扩展“传感器自带时间戳”的校准逻辑，而不影响上层代码。

#include "nav_core/timebase.hpp"

namespace uwnav::core::timebase {

// ============================================================================
// 基础时间接口
// ============================================================================

MonoTimeNs now_ns()
{
    const auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

TimePoint now()
{
    return Clock::now();
}

// ============================================================================
// 默认延迟（全局静态配置）
// ============================================================================
//
// 说明：
//   - 这里的延迟是“从物理世界发生”到“主机收到并解析完成”的估计值；
//   - 当前版本先写死一个 LatencyDefaults 全局实例，后续可以从 YAML / config 加载；
//   - 导航/融合模块只看 corrected_time_ns，不关心内部怎么算。
// ============================================================================

static LatencyDefaults g_latency_defaults{};

LatencyDefaults& latency_defaults()
{
    return g_latency_defaults;
}

// 根据传感器类型，返回默认延迟（单位：ns）
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

// ============================================================================
// stamp(): 统一时间戳核心方法
// ============================================================================
//
// 使用方式（以 IMU 为例）：
//   auto ts = uwnav::timebase::stamp("imu0", SensorKind::IMU);
//   frame.mono_ns = ts.host_time_ns;
//   frame.est_ns  = ts.corrected_time_ns;
//
// 说明：
//   - host_time_ns        ：调用时刻的 steady_clock 纳秒值（主机时间）
//   - sensor_time_ns(opt) ：可选的“传感器内部时间”（目前仅存放，不做对齐）
//   - latency_ns          ：估计延迟；不传则用 default_latency_ns(kind)
//   - corrected_time_ns   ：host_time_ns - latency_ns
//
// 将来如果需要做“传感器时钟对齐”（例如 DVL/USBL 自带时间），可以在这里扩展：
//   - 利用 sensor_time_ns 做 offset 估计；
//   - 形成 host_time_ns <-> sensor_time_ns 的映射；
//   - 再统一写入 corrected_time_ns。
//
Stamp stamp(const std::string&              sensor_id,
            SensorKind                      kind,
            std::optional<MonoTimeNs>       sensor_time_ns,
            std::optional<std::int64_t>     latency_ns)
{
    Stamp s;
    s.sensor_id      = sensor_id;
    s.kind           = kind;
    s.host_time_ns   = now_ns();
    s.sensor_time_ns = sensor_time_ns;

    // 1) 选择延迟：调用方显式传入优先，否则使用默认值
    s.latency_ns = latency_ns.has_value()
        ? *latency_ns
        : default_latency_ns(kind);

    // 2) 统一后的 corrected 时间戳（当前版本 = host_time_ns - latency）
    s.corrected_time_ns = s.host_time_ns - s.latency_ns;

    // TODO（将来）：如需利用 sensor_time_ns 做时钟对齐，在这里扩展逻辑
    // if (sensor_time_ns.has_value()) {
    //     // 估计传感器时钟与 host_time_ns 的偏移，修正 corrected_time_ns
    // }

    return s;
}

} // namespace uwnav::core::timebase
