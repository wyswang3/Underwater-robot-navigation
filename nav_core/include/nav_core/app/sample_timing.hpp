#pragma once

#include <cstdint>

#include "nav_core/core/types.hpp"

namespace nav_core::app {

constexpr std::uint32_t kAgeUnknownMs = 0xFFFFFFFFu;

/**
 * @brief 统一的样本时间元数据。
 *
 * 约定：
 *  - sensor_time_ns: 样本真正对应的采样时刻（steady/mono 时间轴）；
 *  - recv_mono_ns: 驱动线程收到/解码样本的时刻；
 *  - consume_mono_ns: 主线程真正接受并用于估计/发布语义的时刻；
 *  - freshness / stale / out-of-order 一律基于 sensor_time_ns 所代表的样本时刻判断。
 */
struct SampleTiming final {
    MonoTimeNs sensor_time_ns{0};
    MonoTimeNs recv_mono_ns{0};
    MonoTimeNs consume_mono_ns{0};

    [[nodiscard]] MonoTimeNs sample_mono_ns() const noexcept
    {
        return sensor_time_ns;
    }
};

/**
 * @brief 计算某个状态/样本相对当前时间的累计 age。
 *
 * 输入和输出都在 steady/mono 时间轴上定义；若时间无效则返回 kAgeUnknownMs。
 */
std::uint32_t compute_age_ms(MonoTimeNs now_mono_ns, MonoTimeNs stamp_ns) noexcept;

/**
 * @brief 判断样本是否仍在允许的最大年龄窗口内。
 *
 * 若 max_age_s<=0 则视为不限制年龄；若样本时间无效或位于未来则返回 false。
 */
bool is_sample_fresh(MonoTimeNs now_mono_ns,
                     MonoTimeNs sample_mono_ns,
                     double     max_age_s) noexcept;

/**
 * @brief 判断带完整时间元数据的样本是否 fresh。
 */
bool is_sample_fresh(MonoTimeNs now_mono_ns,
                     const SampleTiming& timing,
                     double              max_age_s) noexcept;

/**
 * @brief 只接受严格更新的样本时间，避免重复或乱序样本被重复消费。
 */
bool should_consume_sample(MonoTimeNs candidate_sample_ns,
                           MonoTimeNs last_consumed_sample_ns) noexcept;

} // namespace nav_core::app
