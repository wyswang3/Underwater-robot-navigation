// nav_core/include/nav_core/core/timebase.hpp
#pragma once

#include <chrono>
#include <cstdint>
#include <optional>
#include <string>

#include "nav_core/core/types.hpp"  // MonoTimeNs / SysTimeNs

namespace nav_core::core::timebase {

// ============= 1. 基础时间接口 =============

using Clock     = std::chrono::steady_clock;
using TimePoint = Clock::time_point;
using Duration  = Clock::duration;

/// @brief 当前单调时钟时间（纳秒）
MonoTimeNs now_ns();

/// @brief 当前单调时钟 time_point
TimePoint now();

// ============= 2. 传感器类型 =============

/// @brief 传感器种类，用于选择默认延迟等策略
enum class SensorKind : std::uint8_t {
    IMU,
    DVL,
    USBL,
    OTHER
};

// ============= 3. 统一时间戳结构 =============

/// @brief 统一时间戳：把 host 时间 / 传感器时间 / 延迟统一封装
struct Stamp {
    std::string   sensor_id;        ///< 如 "imu0", "dvl0", "usbl0"
    SensorKind    kind{SensorKind::OTHER};

    // 数据到达/解析完成时的主机单调时间（steady_clock）。
    // 在驱动链路中对应 recv_mono_ns。
    MonoTimeNs host_time_ns{0};

    // 样本对应的采样时刻（steady/mono 时间轴）。
    // 若设备本身没有时间戳，可退化为基于固定延迟回推的采样时刻。
    std::optional<MonoTimeNs> sensor_time_ns;

    // 估计的延迟（ns），正值表示“数据比 host_time 滞后”
    std::int64_t latency_ns{0};

    // 对外统一使用的“样本时间”（steady/mono 时间轴）：
    //   - 若 sensor_time_ns 可用，则优先直接使用；
    //   - 否则退化为 host_time_ns - latency_ns。
    // 该字段用于 freshness / stale / 日志重放，不是 wall/epoch 时间。
    MonoTimeNs corrected_time_ns{0};
};

// ============= 4. 默认延迟参数 =============

/// @brief 导航进程中各类传感器的默认延迟（工程经验值）
struct LatencyDefaults {
    std::int64_t imu_ns   = 2'000'000;      ///< IMU 默认延迟：2 ms
    std::int64_t dvl_ns   = 50'000'000;     ///< DVL 默认延迟：50 ms
    std::int64_t usbl_ns  = 150'000'000;    ///< USBL 默认延迟：150 ms
    std::int64_t other_ns = 0;
};

/// @brief 获取全局延迟默认值（配置阶段读写即可，线程安全要求不高）
LatencyDefaults& latency_defaults();

// ============= 5. 统一打时间戳工具函数 =============
//
// 典型用法：
//   auto st = stamp("imu0", SensorKind::IMU);
//   frame.recv_mono_ns   = st.host_time_ns;
//   frame.sensor_time_ns = st.corrected_time_ns;
//   frame.mono_ns        = st.corrected_time_ns;
//   frame.est_ns         = frame.mono_ns;   // 兼容字段
//
Stamp stamp(const std::string&          sensor_id,
            SensorKind                  kind,
            std::optional<MonoTimeNs>   sensor_time_ns = std::nullopt,
            std::optional<std::int64_t> latency_ns     = std::nullopt);

} // namespace nav_core::core::timebase

// ============================================================================
// 6. 兼容旧命名空间（uwnav::*）
// ============================================================================
//
// 说明：
//   - 老代码大量使用 uwnav::core::timebase / uwnav::timebase；
//   - 为避免一次性大面积改动，这里通过 using 做别名；
//   - 新代码请统一使用 nav_core::core::timebase。

// 把 nav_core::MonoTimeNs 映射到 uwnav::core::MonoTimeNs
namespace uwnav::core {
    using MonoTimeNs = nav_core::MonoTimeNs;
}

// uwnav::core::timebase -> 直接引用 nav_core::core::timebase 下所有符号
namespace uwnav::core::timebase {
    using namespace nav_core::core::timebase;
}

// uwnav::timebase -> 旧的顶层别名，同样导出所有符号
namespace uwnav::timebase {
    using namespace nav_core::core::timebase;
}
