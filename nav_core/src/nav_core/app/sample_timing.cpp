#include "nav_core/app/sample_timing.hpp"

#include <limits>

namespace nav_core::app {

std::uint32_t compute_age_ms(MonoTimeNs now_mono_ns, MonoTimeNs stamp_ns) noexcept
{
    if (stamp_ns <= 0 || now_mono_ns < stamp_ns) {
        return kAgeUnknownMs;
    }

    const std::uint64_t age_ms64 =
        static_cast<std::uint64_t>(now_mono_ns - stamp_ns) / 1000000ull;
    return (age_ms64 > std::numeric_limits<std::uint32_t>::max())
        ? std::numeric_limits<std::uint32_t>::max()
        : static_cast<std::uint32_t>(age_ms64);
}

bool is_sample_fresh(MonoTimeNs now_mono_ns,
                     MonoTimeNs sample_mono_ns,
                     double     max_age_s) noexcept
{
    if (sample_mono_ns <= 0) {
        return false;
    }
    if (now_mono_ns < sample_mono_ns) {
        return false;
    }
    if (max_age_s <= 0.0) {
        return true;
    }

    const double age_s = static_cast<double>(now_mono_ns - sample_mono_ns) * 1e-9;
    return age_s <= max_age_s;
}

bool is_sample_fresh(MonoTimeNs now_mono_ns,
                     const SampleTiming& timing,
                     double              max_age_s) noexcept
{
    return is_sample_fresh(now_mono_ns, timing.sample_mono_ns(), max_age_s);
}

bool should_consume_sample(MonoTimeNs candidate_sample_ns,
                           MonoTimeNs last_consumed_sample_ns) noexcept
{
    if (candidate_sample_ns <= 0) {
        return false;
    }
    if (last_consumed_sample_ns <= 0) {
        return true;
    }
    return candidate_sample_ns > last_consumed_sample_ns;
}

} // namespace nav_core::app
