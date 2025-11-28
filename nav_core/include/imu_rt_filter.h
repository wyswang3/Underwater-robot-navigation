#pragma once

#include <array>
#include <cstdint>
#include <optional>

namespace nav_core {

struct ImuFiltOutput {
    std::array<double, 3> acc_f_g;      // 滤波后加速度，单位：g
    std::array<double, 3> gyro_f_dps;   // 滤波后角速度，单位：deg/s
    double yaw_deg;                     // 積分+解缠后的航向角（deg，连续）
};

/**
 * 轻量实时 IMU 滤波器（C++ 版）
 * - 接口设计与 Python RealTimeIMUFilter 对齐
 * - 输入：timestamp_s（单调秒） + acc_g + gyro_dps
 * - 校准阶段：前 calib_seconds 秒求均值作为零偏
 * - 滤波：一阶低通，可选“伪二阶”
 * - 航向：用 Z 轴角速度（deg/s）按 dt 积分，内部用弧度做解缠
 */
class RealTimeImuFilterCpp {
public:
    RealTimeImuFilterCpp(double fs           = 100.0,
                         double cutoff       = 4.0,
                         bool   calibrate    = true,
                         double calib_seconds= 1.0,
                         bool   second_order = false,
                         double clamp_gyro_dps = 0.0);

    /** 
     * 处理一帧 IMU 数据。
     * @param timestamp_s   单调时间戳（秒），建议使用 mono_ns * 1e-9
     * @param acc_g         加速度，单位：g
     * @param gyro_dps      角速度，单位：deg/s
     * @return std::nullopt 表示仍在校准阶段；否则返回滤波结果
     */
    std::optional<ImuFiltOutput> process(double timestamp_s,
                                         const std::array<double,3>& acc_g,
                                         const std::array<double,3>& gyro_dps);

private:
    double fs_;
    double cutoff_;
    bool   second_order_;
    double clamp_gyro_dps_;

    // 一阶低通时间常数 tau
    double tau_;

    // 校准
    bool   do_calib_;
    int    calib_N_;
    int    calib_count_;
    std::array<double,3> acc_sum_;
    std::array<double,3> gyr_sum_dps_;

    std::array<double,3> accel_bias_;
    std::array<double,3> gyro_bias_dps_;

    // 低通状态
    bool   lp_inited_;
    std::array<double,3> acc_lp1_;
    std::array<double,3> acc_lp2_;
    std::array<double,3> gyro_lp1_dps_;
    std::array<double,3> gyro_lp2_dps_;

    // 航向积分（弧度）
    double yaw_rad_;
    double yaw_offset_;
    double last_ts_;
    double default_dt_;

    void lowpass_step(std::array<double,3>& y,
                      const std::array<double,3>& x,
                      double dt);

    void maybe_clamp_gyro(std::array<double,3>& gyr_dps);
};

} // namespace nav_core
