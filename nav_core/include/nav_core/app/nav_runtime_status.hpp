#pragma once

#include "nav_core/app/device_binding.hpp"
#include "nav_core/app/sample_timing.hpp"
#include "nav_core/core/types.hpp"
#include "nav_core/estimator/eskf.hpp"
#include "shared/msg/nav_state.hpp"

namespace nav_core::app {

/**
 * @brief 发布 NavState 前所需的运行时上下文。
 *
 * 该结构把“状态机门控所需的信息”集中在一个地方，避免 nav_daemon_runner
 * 在多个分支中散落地拼凑 valid/stale/fault 语义。
 */
struct NavPublishContext {
    MonoTimeNs publish_mono_ns{0};
    MonoTimeNs state_stamp_ns{0};
    SampleTiming imu_timing{};
    SampleTiming dvl_timing{};
    DeviceConnectionState imu_device_state{DeviceConnectionState::DISCONNECTED};
    DeviceConnectionState dvl_device_state{DeviceConnectionState::DISCONNECTED};

    double max_imu_age_s{0.0};
    double max_dvl_age_s{0.0};

    bool imu_enabled{true};
    bool dvl_enabled{true};
    bool imu_bias_ready{false};
};

/**
 * @brief 检查 ESKF 导出的数值状态是否仍然有限。
 *
 * 一旦名义状态出现 NaN/Inf，就不能再把它包装成“合法导航数值”发布出去。
 */
bool eskf_state_is_finite(const estimator::EskfFilter& eskf) noexcept;

/**
 * @brief 将 ESKF 名义状态复制到 NavState 的运动学字段。
 *
 * 约定：
 *   - 位姿/速度来自 ESKF 名义状态；
 *   - omega_b / acc_b 必须表达“当前最新的机体系角速度/线加速度量测”，
 *     不能错误复用 ESKF 内部 bias 状态；
 *   - 当当前循环没有可用 IMU 量测时，omega_b / acc_b 退化为 0，由
 *     apply_nav_publish_semantics() 再决定该帧是否可信。
 *
 * 本函数只做数值映射，不填充 valid/stale/fault 等工程语义；
 * 这些语义统一由 apply_nav_publish_semantics() 决定。
 */
void fill_nav_state_kinematics(const estimator::EskfFilter& eskf,
                               shared::msg::NavState&       nav,
                               MonoTimeNs                   state_stamp_ns,
                               const ImuSample*             latest_imu_sample = nullptr) noexcept;

/**
 * @brief 根据运行时上下文显式填充 NavState 的可信度/新鲜度/故障语义。
 *
 * 约束：
 *   - 未初始化、对准未完成、IMU 缺失/超时、状态数值失效时，必须输出 valid=0；
 *   - DVL 缺失只允许进入 DEGRADED，不能伪装成 OK；
 *   - status_flags 不允许靠默认值伪造正常状态。
 */
void apply_nav_publish_semantics(const NavPublishContext& ctx,
                                 bool                     state_finite,
                                 shared::msg::NavState&   nav) noexcept;

} // namespace nav_core::app
