// nav_core/include/nav_core/imu_rt_filter.hpp
#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace nav_core {

/**
 * @brief 实时 IMU 滤波输出
 *
 * 单位：
 *  - acc_f_g      : g
 *  - gyro_f_dps   : deg/s
 *  - yaw_deg      : 航向角，deg，经过解缠后连续
 */
struct ImuFiltOutput {
    std::array<double, 3> acc_f_g{};      ///< 滤波后加速度 (g)
    std::array<double, 3> gyro_f_dps{};   ///< 滤波后角速度 (deg/s)
    double                yaw_deg{0.0};   ///< 积分+解缠后的航向角 (deg, 连续)
};

/**
 * @brief 轻量实时 IMU 滤波器（C++ 版）
 *
 * 设计目标：
 *  - 与 Python RealTimeIMUFilter 的接口语义保持一致，便于算法对比
 *  - 输入：
 *      * timestamp_s  : 单调时间戳（秒，建议由 mono_ns * 1e-9 转换）
 *      * acc_g        : 加速度，单位 g
 *      * gyro_dps     : 角速度，单位 deg/s
 *  - 校准阶段：
 *      * 前 calib_seconds 秒进行零偏估计，求均值作为 bias
 *      * 在校准阶段，process() 返回 std::nullopt
 *  - 滤波：
 *      * 一阶低通（可配置“伪二阶”）
 *  - 航向积分：
 *      * 使用 Z 轴角速度（deg/s）按 dt 积分
 *      * 内部用弧度做解缠，输出 yaw_deg 为连续角度
 */
class RealTimeImuFilterCpp {
public:
    RealTimeImuFilterCpp(double fs              = 100.0,
                         double cutoff          = 4.0,
                         bool   calibrate       = true,
                         double calib_seconds   = 1.0,
                         bool   second_order    = false,
                         double clamp_gyro_dps  = 0.0);

    /**
     * @brief 处理一帧 IMU 数据
     *
     * @param timestamp_s   单调时间戳（秒），建议使用 mono_ns * 1e-9
     * @param acc_g         加速度 [gx, gy, gz]，单位：g
     * @param gyro_dps      角速度 [wx, wy, wz]，单位：deg/s
     *
     * @return std::nullopt 表示仍在校准阶段（未输出有效结果）
     * @return ImuFiltOutput 滤波后输出（校准完成后）
     */
    std::optional<ImuFiltOutput> process(
        double                          timestamp_s,
        const std::array<double, 3>&   acc_g,
        const std::array<double, 3>&   gyro_dps
    );

private:
    // 基本参数
    double fs_;                ///< 采样频率 (Hz)
    double cutoff_;            ///< 截止频率 (Hz)
    bool   second_order_;      ///< 是否采用伪二阶
    double clamp_gyro_dps_;    ///< 限幅阈值；<=0 表示不启用

    // 一阶低通时间常数 tau
    double tau_;

    // 校准相关
    bool   do_calib_;                  ///< 是否启用零偏校准
    int    calib_N_;                   ///< 校准样本数（由 calib_seconds * fs 计算）
    int    calib_count_;               ///< 已累计样本数
    std::array<double, 3> acc_sum_{};       ///< 校准期加速度累加 (g)
    std::array<double, 3> gyr_sum_dps_{};   ///< 校准期角速度累加 (deg/s)

    std::array<double, 3> accel_bias_{};    ///< 加速度零偏 (g)
    std::array<double, 3> gyro_bias_dps_{}; ///< 角速度零偏 (deg/s)

    // 低通滤波状态
    bool   lp_inited_;                       ///< 低通是否已经初始化
    std::array<double, 3> acc_lp1_{};        ///< 一阶低通状态1 (g)
    std::array<double, 3> acc_lp2_{};        ///< 二阶时第二级状态 (g)
    std::array<double, 3> gyro_lp1_dps_{};   ///< 一阶低通状态1 (deg/s)
    std::array<double, 3> gyro_lp2_dps_{};   ///< 二阶时第二级状态 (deg/s)

    // 航向积分（内部用弧度）
    double yaw_rad_;              ///< 当前航向（rad，已解缠）
    double yaw_offset_;           ///< 初始偏移（rad）
    double last_ts_;              ///< 上一帧时间戳（秒）；<0 表示尚未初始化
    double default_dt_;           ///< 默认 dt（秒），在 timestamp 抖动/丢失时兜底

    // yaw 解缠时用到的“上一帧解缠后的 yaw”（与 cpp 中 last_yaw_unwrapped_ 对应）
    double last_yaw_unwrapped_{0.0};

    // 一阶低通更新：y = y + alpha * (x - y)
    void lowpass_step(std::array<double, 3>&       y,
                      const std::array<double, 3>& x,
                      double                      dt);

    // 可选的角速度限幅
    void maybe_clamp_gyro(std::array<double, 3>& gyr_dps);
};

} // namespace nav_core
