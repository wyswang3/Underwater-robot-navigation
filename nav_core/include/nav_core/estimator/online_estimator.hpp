// nav_core/include/nav_core/estimator/online_estimator.hpp
//
// @file  online_estimator.hpp
// @brief 在线导航解算器：IMU + DVL 的轻量级 2D/2.5D 组合导航。
// 
// 职责：
//   - 以 IMU 采样作为主时基（高频），DVL 速度作为慢速约束：
//       * IMU yaw 积分 + unwrap（使用 core::math 工具）；
//       * IMU 体速积分为 Body 速度（可选，仅内部使用）；
//       * DVL 速度（ENU）做噪声地板 + ZUPT 判定 + 约束修正；
//       * 基于 ENU 速度做位置梯形积分（Kahan 补偿可在 .cpp 中使用）；
//   - 维护导航健康状态 / 标志位：
//       * DVL 掉线超时 → 降级为惯导（NavHealth = DEGRADED）；
//       * IMU 或时间轴异常 → NavHealth = INVALID；
//   - 输出统一的 NavState（shared/msg/nav_state.hpp），
//     供 nav_daemon 或控制模块通过 shm/UDP 发布使用。
// 
// 非目标：
//   - 不负责 ESKF / 高级滤波（留在 filters/eskf.hpp）；
//   - 不负责线程、串口、日志 IO，这些由 nav_daemon / drivers 层承担；
//   - 不直接做因子图平滑（留给 graph_smoother_2d.hpp）。
//

#pragma once

#include <cstdint>
#include <mutex>

#include "nav_core/core/types.hpp"          // Vec3d / ImuSample / DvlSample / MonoTimeNs
#include "nav_core/core/math.hpp"      // kDvlNoiseFloorMps 等工具常量
#include "shared/msg/nav_state.hpp"    // shared::msg::NavState / NavHealth

namespace nav_core::estimator {

// ============================ 1. 配置结构 ============================

/**
 * @brief 在线导航解算配置。
 *
 * 说明：
 *   - 所有阈值都是“工程参数”，后续可以通过配置文件/YAML 暴露；
///   - DVL 相关阈值与 core::math 中的噪声地板函数配合使用；
///   - IMU 相关阈值假定 imu_rt_filter 已经做了静止 bias 估计 + 基本去噪，
 *     这里更多是“残余极小值”的 deadzone 判定。
 */
struct OnlineEstimatorConfig {
    // ---- DVL 相关（速度噪声地板 + ZUPT + 掉线超时）----

    /// DVL 速度噪声地板（总速度 |v| < 此值时认为主要是噪声，倾向于置零）。
    /// 默认采用 math::kDvlNoiseFloorMps ≈ 0.08 m/s。
    double dvl_noise_floor_mps = core::math::kDvlNoiseFloorMps;

    /// ZUPT 判定阈值（基于 ENU 速度模长），通常略小于噪声地板。
    /// 若 |v_dvl| < zupt_threshold_mps，则认为“站立近似静止”，
    /// 可在 .cpp 中触发 ZUPT 策略（例如速度强制衰减到 0）。
    double zupt_threshold_mps  = 0.03;

    /// DVL 掉线超时时间（秒）。
    /// 若 now_mono_ns - last_dvl_mono_ns > dvl_timeout_s，则视为 DVL 不可用，
    /// NavHealth 至少降级为 DEGRADED。
    double dvl_timeout_s       = 2.0;

    /// 是否使用 DVL 垂直速度分量更新 z（启用前需确认水池数据可靠性）。
    bool use_dvl_vertical      = false;

    // ---- IMU 相关（残余 deadzone）----

    /// IMU 角速度 residual 的 deadzone（rad/s），
    /// 通常比 imu_rt_filter 中的阈值略大一点，用于抑制长期漂移。
    double imu_gyro_deadzone_rad_s   = 0.001;

    /// IMU 线加速度 residual 的 deadzone（m/s^2），
    /// 用于抑制近似静止时的微小噪声积分。
    double imu_accel_deadzone_mps2   = 0.01;

    // ---- 积分 / 数值稳定 ----

    /// IMU 连续两帧最大允许时间间隔（秒），超过则认为时间轴异常，
    /// 可以在 .cpp 中选择“跳变重置”或直接标记 INVALID。
    double max_imu_dt_s       = 0.05;   // 50 Hz 左右

    /// 位置积分时允许的最大单步位移（m），防止异常 dt 或速度尖刺导致
    /// 一次积分跳远。
    double max_step_displacement_m = 1.0;

    // ---- 输出频率 / 采样策略（可选扩展）----
    //
    // 当前设计中，OnlineEstimator 每收到一帧 IMU 就更新一次内部 NavState。
    // 若你希望“插值到固定发布频率”，可以未来在 .cpp 或 nav_daemon 层扩展。
};


// ============================ 2. 在线估计器类接口 ============================

/**
 * @brief 在线导航解算器：IMU 驱动的高频积分 + DVL 低频约束。
 *
 * 调用模型（推荐）：IMU 线程驱动，DVL 异步注入：
 *
 *   OnlineEstimator est(cfg);
 *   est.reset(init_pos, init_vel, init_yaw, init_depth);
 *
 *   // IMU 回调线程：
 *   void on_imu(const ImuSample& imu) {
 *       est.handleImuSample(imu);
 *       shared::msg::NavState nav;
 *       if (est.lastNavState(nav)) {
 *           // nav_daemon: 发布 nav 到 shm / UDP
 *       }
 *   }
 *
 *   // DVL 回调线程：
 *   void on_dvl(const DvlSample& dvl) {
 *       est.handleDvlSample(dvl);
 *   }
 *
 * 注意：
 *   - OnlineEstimator 自身不持有线程；
///   - 内部对 NavState 使用互斥锁保护，可多线程读/写；
 *   - 对外接口不抛异常，通过 NavHealth / status_flags 反映状态。
 */
class OnlineEstimator {
public:
    using ImuSample = nav_core::ImuSample;
    using DvlSample = nav_core::DvlSample;

    /// 使用配置构造在线解算器。
    explicit OnlineEstimator(const OnlineEstimatorConfig& cfg = OnlineEstimatorConfig{});

    /// 更新配置（不会自动重置状态）。
    void setConfig(const OnlineEstimatorConfig& cfg);
    [[nodiscard]] OnlineEstimatorConfig config() const { return cfg_; }

    /**
     * @brief 重置在线导航内部状态。
     *
     * @param init_pos_enu 初始位置（ENU，单位 m）
     * @param init_vel_enu 初始速度（ENU，单位 m/s，通常为 0）
     * @param init_yaw_rad 初始航向角（rad）
     * @param init_depth_m 初始深度（m，下为正）
     */
    void reset(const Vec3d& init_pos_enu,
               const Vec3d& init_vel_enu,
               double       init_yaw_rad,
               double       init_depth_m);

    /// 当前是否已经完成初始化。
    [[nodiscard]] bool initialized() const noexcept { return initialized_; }

    // ==================== 2.1 传感器输入接口 ====================

    /**
     * @brief 处理一帧 IMU 采样（主驱动入口）。
     *
     * 行为（在 .cpp 中实现）：
///   - 若尚未初始化，则根据第一帧 IMU 时间戳建立 timebase；
///   - 根据上一帧 IMU 时间戳计算 dt，做上限检查（cfg_.max_imu_dt_s）；
///   - 积分 gyro yaw（unwrap），得到当前连续航向角；
///   - 将 IMU 体速（由上一层滤波器提供或内部估计）转换到 ENU；
///   - 结合最近一次有效 DVL 速度做约束（ZUPT / 噪声地板 / 融合策略）；
///   - 对 ENU 速度做梯形积分，更新位置 pos_enu_；
///   - 根据 DVL 掉线时间、IMU 正常性更新 NavHealth / status_flags；
///   - 更新内部 NavState 缓存。
     */
    void handleImuSample(const ImuSample& imu);

    /**
     * @brief 处理一帧 DVL 采样（慢速约束输入）。
     *
     * 行为（在 .cpp 中实现）：
///   - 更新 last_dvl_ / last_dvl_mono_ns_；
///   - 根据 cfg_.dvl_noise_floor_mps 对速度做噪声地板处理；
///   - 标记 DVL 状态（OK / 无效），用于 NavStatusFlags；
///   - 不直接改变位置，只在下一次 handleImuSample 中参与融合。
     */
    void handleDvlSample(const DvlSample& dvl);

    // ==================== 2.2 状态查询接口 ====================

    /**
     * @brief 获取最近一次计算出的 NavState 快照。
     *
     * @param out 输出：NavState（按值拷贝）
     * @return true 若 OnlineEstimator 已初始化且 out 有效；
     * @return false 若尚未初始化（此时 out 未定义）。
     *
     * 线程安全：内部使用互斥锁保护 nav_state_。
     */
    bool lastNavState(shared::msg::NavState& out) const;

    /// 当前导航健康状态（粗粒度）。
    [[nodiscard]] shared::msg::NavHealth navHealth() const noexcept {
        return health_;
    }

    /// 当前导航状态标志（细粒度 bitmask，见 nav_state.hpp 中 NavStatusFlags）。
    [[nodiscard]] std::uint16_t navStatusFlags() const noexcept {
        return status_flags_;
    }

private:
    // ==================== 3. 内部辅助函数（仅在 .cpp 中实现） ====================

    /// 使用 IMU 更新 yaw_unwrapped_，并根据 dt 检查数值稳定性。
    void integrateYawFromImu(const ImuSample& imu, double dt_s);

    /// 基于当前 yaw_unwrapped_ & IMU / DVL 更新 ENU 速度 vel_enu_。
    void updateVelocityEnu(const ImuSample& imu, double dt_s);

    /// 基于 ENU 速度做位置积分（带步长检查）。
    void integratePosition(double dt_s);

    /// 使用最新 DVL 信息更新健康标志（DVL 掉线、质量等）。
    void updateDvlHealth(MonoTimeNs now_mono_ns);

    /// 综合 IMU / DVL 状态，更新 NavHealth / status_flags。
    void updateHealthFlags();

    /// 将内部状态打包到 shared::msg::NavState。
    void packNavStateNoLock();

private:
    // 配置
    OnlineEstimatorConfig cfg_{};

    // 初始化 & 时间轴状态
    bool       initialized_{false};
    MonoTimeNs last_imu_mono_ns_{0};

    // 最近一帧 IMU / DVL
    ImuSample  last_imu_{};
    DvlSample  last_dvl_{};
    MonoTimeNs last_dvl_mono_ns_{0};
    bool       has_dvl_{false};

    // 连续 yaw 与 ENU 位置/速度
    double yaw_unwrapped_rad_{0.0};
    Vec3d  pos_enu_{0.0, 0.0, 0.0};
    Vec3d  vel_enu_{0.0, 0.0, 0.0};
    double depth_m_{0.0};

    // 健康/标志
    shared::msg::NavHealth health_{shared::msg::NavHealth::UNINITIALIZED};
    std::uint16_t          status_flags_{0};

    // 对外可见的导航状态快照（加锁保护）
    mutable std::mutex     mutex_;
    shared::msg::NavState  nav_state_{};
};

} // namespace nav_core::estimator
