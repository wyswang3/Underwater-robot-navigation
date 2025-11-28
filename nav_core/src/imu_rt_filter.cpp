#include "nav_core/imu_rt_filter.h"
#include <cmath>
#include <algorithm>

namespace nav_core {

namespace {
constexpr double PI = 3.14159265358979323846;

// 简单的角度解缠（弧度版）
double unwrap_angle(double angle, double& cumulative_offset, double last_unwrapped) {
    // 以 last_unwrapped - cumulative_offset 为参考
    const double ref = last_unwrapped - cumulative_offset;
    double delta = angle - ref;
    if (delta > PI) {
        cumulative_offset -= 2.0 * PI;
    } else if (delta < -PI) {
        cumulative_offset += 2.0 * PI;
    }
    return angle + cumulative_offset;
}

} // namespace

RealTimeImuFilterCpp::RealTimeImuFilterCpp(double fs,
                                           double cutoff,
                                           bool   calibrate,
                                           double calib_seconds,
                                           bool   second_order,
                                           double clamp_gyro_dps)
    : fs_(fs)
    , cutoff_(cutoff)
    , second_order_(second_order)
    , clamp_gyro_dps_(clamp_gyro_dps)
    , yaw_rad_(0.0)
    , yaw_offset_(0.0)
    , last_ts_(-1.0)
{
    if (fs_ <= 0.0) {
        fs_ = 100.0;
    }
    if (cutoff_ <= 0.0 || cutoff_ >= 0.5 * fs_) {
        cutoff_ = std::min(4.0, 0.25 * fs_);
    }

    tau_ = 1.0 / (2.0 * PI * cutoff_);

    do_calib_     = calibrate;
    calib_N_      = std::max(1, static_cast<int>(std::round(calib_seconds * fs_)));
    calib_count_  = 0;

    acc_sum_.fill(0.0);
    gyr_sum_dps_.fill(0.0);

    accel_bias_.fill(0.0);
    gyro_bias_dps_.fill(0.0);

    lp_inited_ = false;
    acc_lp1_.fill(0.0);
    acc_lp2_.fill(0.0);
    gyro_lp1_dps_.fill(0.0);
    gyro_lp2_dps_.fill(0.0);

    default_dt_ = 1.0 / fs_;
}

void RealTimeImuFilterCpp::lowpass_step(std::array<double,3>& y,
                                        const std::array<double,3>& x,
                                        double dt)
{
    const double alpha = dt / (tau_ + dt);
    for (int i = 0; i < 3; ++i) {
        y[i] += alpha * (x[i] - y[i]);
    }
}

void RealTimeImuFilterCpp::maybe_clamp_gyro(std::array<double,3>& gyr_dps)
{
    if (clamp_gyro_dps_ > 0.0) {
        for (int i = 0; i < 3; ++i) {
            if (gyr_dps[i] >  clamp_gyro_dps_) gyr_dps[i] =  clamp_gyro_dps_;
            if (gyr_dps[i] < -clamp_gyro_dps_) gyr_dps[i] = -clamp_gyro_dps_;
        }
    }
}

std::optional<ImuFiltOutput> RealTimeImuFilterCpp::process(
        double timestamp_s,
        const std::array<double,3>& acc_g,
        const std::array<double,3>& gyro_dps)
{
    // ---- 计算 dt ----
    double dt = default_dt_;
    if (last_ts_ >= 0.0) {
        dt = timestamp_s - last_ts_;
        if (!(dt >= 0.2 * default_dt_ && dt <= 5.0 * default_dt_)) {
            dt = default_dt_;
        }
    }
    last_ts_ = timestamp_s;

    // ---- 校准阶段 ----
    if (do_calib_) {
        for (int i = 0; i < 3; ++i) {
            acc_sum_[i]     += acc_g[i];
            gyr_sum_dps_[i] += gyro_dps[i];
        }
        ++calib_count_;

        if (calib_count_ >= calib_N_) {
            for (int i = 0; i < 3; ++i) {
                accel_bias_[i]     = acc_sum_[i]     / calib_count_;
                gyro_bias_dps_[i]  = gyr_sum_dps_[i] / calib_count_;
            }
            // 初始化低通状态
            for (int i = 0; i < 3; ++i) {
                const double acc0 = acc_g[i] - accel_bias_[i];
                const double gyr0 = gyro_dps[i] - gyro_bias_dps_[i];
                acc_lp1_[i]      = acc0;
                gyro_lp1_dps_[i] = gyr0;
                if (second_order_) {
                    acc_lp2_[i]      = acc0;
                    gyro_lp2_dps_[i] = gyr0;
                }
            }
            lp_inited_ = true;
            do_calib_  = false;
        }
        return std::nullopt;  // 校准期不输出
    }

    // ---- 去偏 + 可选夹持 ----
    std::array<double,3> acc_corr{};
    std::array<double,3> gyr_corr_dps{};
    for (int i = 0; i < 3; ++i) {
        acc_corr[i]      = acc_g[i]      - accel_bias_[i];
        gyr_corr_dps[i]  = gyro_dps[i]   - gyro_bias_dps_[i];
    }
    maybe_clamp_gyro(gyr_corr_dps);

    // ---- 低通 ----
    if (!lp_inited_) {
        acc_lp1_      = acc_corr;
        gyro_lp1_dps_ = gyr_corr_dps;
        if (second_order_) {
            acc_lp2_      = acc_corr;
            gyro_lp2_dps_ = gyr_corr_dps;
        }
        lp_inited_ = true;
    }

    lowpass_step(acc_lp1_,      acc_corr,      dt);
    lowpass_step(gyro_lp1_dps_, gyr_corr_dps,  dt);
    std::array<double,3> acc_f{};
    std::array<double,3> gyro_f_dps{};

    if (second_order_) {
        lowpass_step(acc_lp2_,      acc_lp1_,      dt);
        lowpass_step(gyro_lp2_dps_, gyro_lp1_dps_, dt);
        acc_f      = acc_lp2_;
        gyro_f_dps = gyro_lp2_dps_;
    } else {
        acc_f      = acc_lp1_;
        gyro_f_dps = gyro_lp1_dps_;
    }

    // ---- 航向积分（用 Z 轴角速度）----
    const double gz_rad = gyro_f_dps[2] * PI / 180.0;
    yaw_rad_ += gz_rad * dt;

    static double last_unwrapped = 0.0;
    double yaw_unwrapped = unwrap_angle(yaw_rad_, yaw_offset_, last_unwrapped);
    last_unwrapped = yaw_unwrapped;
    const double yaw_deg = yaw_unwrapped * 180.0 / PI;

    ImuFiltOutput out{};
    out.acc_f_g      = acc_f;
    out.gyro_f_dps   = gyro_f_dps;
    out.yaw_deg      = yaw_deg;
    return out;
}

} // namespace nav_core
