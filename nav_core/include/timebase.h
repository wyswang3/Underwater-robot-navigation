#ifndef NAV_CORE_TIMEBASE_H
#define NAV_CORE_TIMEBASE_H

#include <cstdint>

namespace nav_core {

/// 生成一对时间戳：
///  - mono_ns: 单调时钟时间，起点为程序启动时刻（纳秒）
///  - est_ns : 估计的“真实”时间（Unix epoch 纳秒）
void stamp(int64_t& mono_ns, int64_t& est_ns);

/// 辅助：把纳秒转成秒（double）
inline double to_sec(int64_t ns) {
    return static_cast<double>(ns) * 1e-9;
}

} // namespace nav_core

#endif // NAV_CORE_TIMEBASE_H
