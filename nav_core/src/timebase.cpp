#include "timebase.h"

#include <chrono>

namespace nav_core {

namespace {

// C++ 版的 TimeBase：
//  - mono0: steady_clock 起点
//  - real0: system_clock 对应的真实时间起点
struct TimeBaseImpl {
    std::chrono::steady_clock::time_point mono0;
    std::chrono::system_clock::time_point real0;

    TimeBaseImpl()
        : mono0(std::chrono::steady_clock::now())
        , real0(std::chrono::system_clock::now()) {}
};

inline TimeBaseImpl& instance() {
    static TimeBaseImpl tb;
    return tb;
}

} // anonymous namespace

void stamp(int64_t& mono_ns, int64_t& est_ns) {
    auto& tb = instance();

    // 1) 单调时间增量
    auto now_mono = std::chrono::steady_clock::now();
    auto dt       = now_mono - tb.mono0;
    auto dt_ns    = std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count();

    mono_ns = dt_ns;

    // 2) 真实时间起点 + 单调增量 = 估计的“真实时间”
    auto real0_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        tb.real0.time_since_epoch()
    ).count();
    est_ns = real0_ns + dt_ns;
}

} // namespace nav_core
