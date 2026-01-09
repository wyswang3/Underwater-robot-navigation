// nav_core/include/nav_core/filters/imu_rt_filter.hpp
//
// @file  imu_rt_filter.hpp
// @brief IMU 实时预处理与静止窗 bias 估计：
//        - 启动阶段静止窗内估计 gyro/accel 零偏；
//        - 运行阶段对原始 ImuSample 做有效性检查 + 去偏 + 简单平滑；
//        - 输出“导航可用”的 ImuSample，并给出当前是否静止的判定结果。
//
// 设计定位：
//   - 位于「驱动层 ImuDriverWit」与「estimator（在线导航 / ESKF）」之间；
//   - 只负责单传感器“信号质量提升”：静止窗 bias + 基本滤波 + deadband；
//   - 不做坐标变换 / ESKF / ZUPT 等高层逻辑，这些由 estimator / math.hpp 处理；
//   - 典型使用方式：
//       ImuRtFilter filter(cfg);
//       ImuSample raw, filt;
//       while (get_raw_from_driver(raw)) {
//           if (filter.process(raw, filt)) { feed_to_estimator(filt); }
//       }
//
// 线程模型：
//   - 默认假定在单线程中调用 process()，内部不做锁保护；
//   - 若多线程访问，请由上层进行同步控制。
// 

#pragma once

#include <cstdint>

#include "nav_core/core/types.hpp"   // MonoTimeNs / ImuSample / ImuFilterConfig

namespace nav_core::filters {

// ============================ 1. 配置与状态 ============================

/**
 * @brief IMU 实时滤波配置。
 *
 * basic         : 复用 nav_core::ImuFilterConfig，对明显异常值做硬性阈值检查； 
 * static_window : 启动阶段用于估计零偏的“静止窗”相关参数；
 * lpf           : 简单一阶低通滤波配置（可选，若 cutoff<=0 则关闭）；
 * deadband      : 对很小的角速度 / 加速度做硬截断，抑制随机游走。
 */
struct ImuRtFilterConfig {
    // ---- 1) 基本有效性检查（来自 types.hpp）----
    ImuFilterConfig basic{};   ///< 尖刺 / NaN / 过大数值过滤配置

    // ---- 2) 静止窗 bias 估计参数 ----
    //
    // static_window_s:
    //   - 上电后，若 |gyro| 和 |accel| 都小于静止阈值，则认为处于静止窗内；
    //   - 当持续静止时间累计 >= static_window_s 时，估计 gyro_bias / accel_bias；
    //   - 估计完成后 bias_ready=true，后续样本将减去该 bias。
    //
    double static_window_s         = 1.0;   ///< 静止窗持续时间 [s]

    float static_max_abs_gyro      = 0.03f; ///< 静止判定阈值：|ω| < 0.03 rad/s
    float static_max_abs_accel_dev = 0.2f;  ///< 静止判定阈值：|a - g_ref| < 0.2 m/s^2

    float g_ref_m_s2               = 9.81f; ///< 静止时加速度模长参考（重力）[m/s^2]

    // ---- 3) 一阶低通滤波（可选）----
    //
    // 若 cutoff_hz <= 0，则关闭对应通道的低通滤波，仅做去偏 + deadband。
    //
    float gyro_lpf_cutoff_hz  = 5.0f;      ///< 角速度低通截止频率 [Hz]
    float accel_lpf_cutoff_hz = 5.0f;      ///< 加速度低通截止频率 [Hz]

    // ---- 4) deadband（随机游走截断）----
    //
    // 对绝对值小于 deadband 的分量直接置零，用于抑制长期积分误差：
    float gyro_deadband_rad_s = 0.002f;    ///< 角速度 deadband
    float accel_deadband_m_s2 = 0.01f;     ///< 加速度 deadband
};

/**
 * @brief ImuRtFilter 的内部状态快照（可用于诊断 / 调试）。
 *
 * 注意：
 *   - 该结构主要用于只读观察，不推荐上层直接修改；
 *   - 某些字段（如 lpf_state_*）仅作为调试参考，不作为对外契约。
 */
struct ImuRtFilterState {
    MonoTimeNs last_mono_ns{0}; ///< 最近一次处理样本的时间戳（ns）

    bool bias_ready{false};     ///< 静止窗 bias 是否已经估计完成
    bool currently_static{false};///< 当前样本是否被判定为“静止”

    // 估计出的零偏（供上层诊断参考）
    float gyro_bias[3]{0.f, 0.f, 0.f};   ///< [bx, by, bz] (rad/s)
    float accel_bias[3]{0.f, 0.f, 0.f};  ///< [bx, by, bz] (m/s^2)

    // 静止窗统计（只读，便于日志/观察）
    double static_window_acc_s{0.0};     ///< 当前连续静止累计时长 [s]
    std::uint64_t static_window_samples{0}; ///< 静止窗累计样本数

    // 低通滤波内部状态（可选，用于调试）
    float gyro_lpf_state[3]{0.f, 0.f, 0.f};
    float accel_lpf_state[3]{0.f, 0.f, 0.f};
};


// ============================ 2. 实时滤波类 ============================

/**
 * @brief IMU 实时滤波器（静止窗 bias 估计 + 有效性检查 + 简单平滑）
 *
 * 典型使用流程：
 *   1) 上电后构造并 reset()；
 *   2) 按时间顺序调用 process(raw, filt)：
 *      - 启动阶段：自动检测静止段，累计 static_window_s 后估计 bias；
 *      - 运行阶段：对每个样本执行：
 *          * basic 有效性检查（超量程 / NaN / inf 等直接丢弃）；
 *          * 减去估计 bias；
///          * 一阶低通滤波（若配置启用）；
///          * deadband 截断（去除很小随机游走）。
 *      - 若输出样本有效，则返回 true，并在 out 中给出滤波后 ImuSample。
 *
 * 设计约束：
 *   - 不做坐标变换：仅在 IMU 自身坐标系下处理 ang_vel / lin_acc；
///   - 不直接估计姿态（roll/pitch/yaw），相关逻辑位于 estimator 或 ESKF 内；
///   - 非线程安全：若多线程访问，请由上层加锁。
 */
class ImuRtFilter {
public:
    /// @brief 使用给定配置构造滤波器（不会自动 reset）。
    explicit ImuRtFilter(const ImuRtFilterConfig& cfg = ImuRtFilterConfig{});

    /// @brief 重置内部状态（丢弃已有 bias / LPF 状态，重新进入“静止窗估计”阶段）。
    void reset();

    /// @brief 设置新的滤波配置（不自动 reset，配置即时生效）。
    void setConfig(const ImuRtFilterConfig& cfg);

    /// @brief 获取当前滤波配置（按值返回）。
    [[nodiscard]] ImuRtFilterConfig config() const;

    /// @brief 获取当前内部状态快照（按值返回，仅用于观察）。
    [[nodiscard]] ImuRtFilterState state() const;

    /// @brief 当前是否已经完成静止窗 bias 估计。
    [[nodiscard]] bool biasReady() const noexcept;

    /// @brief 当前样本是否被认为“处于静止状态”（基于最近一次 process 判定）。
    [[nodiscard]] bool currentlyStatic() const noexcept;

    /**
     * @brief 处理一帧原始 IMU 样本，输出滤波后的样本。
     *
     * 步骤（在 .cpp 中实现，接口约定如下）：
     *   1) 按 basic 配置做有效性检查（NaN / 过大值等），失败则返回 false；
     *   2) 若尚未完成 bias 估计：
///        - 根据 static_* 阈值判断当前是否静止；
///        - 若静止则累积时间与样本，窗口满后估计 gyro_bias/accel_bias；
///   3) 对 ang_vel / lin_acc 减去已估计的 bias（若 bias_ready 为 true）；
///   4) 若启用了低通滤波，则按 dt 对 ang_vel / lin_acc 做一阶 LPF；
///   5) 应用 deadband，将绝对值小于阈值的分量置零；
///   6) 构造输出 ImuSample out（时间戳沿用输入，status/valid 按处理结果设置）。
     *
     * @param in   原始 IMU 样本（来自 ImuDriverWit）
     * @param out  输出：滤波后的 IMU 样本（仅在返回 true 时有效）
     * @return true    成功生成有效输出样本
     * @return false   输入无效 / 被 basic 过滤丢弃
     */
    bool process(const ImuSample& in, ImuSample& out);

private:
    ImuRtFilterConfig cfg_{};
    ImuRtFilterState  st_{};

    // 内部辅助函数只在 .cpp 中实现，这里不展开细节（例如：
    //   - 更新静止窗统计；
    //   - 估计 bias；
    //   - 计算 LPF 系数等）。
};

} // namespace nav_core::filters
