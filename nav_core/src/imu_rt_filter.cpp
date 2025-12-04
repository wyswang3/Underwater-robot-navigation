// nav_core/src/imu_rt_filter.cpp
//
// 实时 IMU 滤波器实现：
//  - 输入：单调时间戳（秒） + 加速度(g) + 角速度(deg/s)
//  - 功能：零偏估计 + 一阶/伪二阶低通 + Z 轴角速度积分 → 连续航向角
//  - 输出：滤波后的 acc / gyro + 连续(yaw_deg)
//
// 典型应用：在 IMU 驱动回调中或导航线程中调用，用于生成“平滑 yaw”供 ESKF 低频校正。

#include <nav_core/imu_rt_filter.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>  // for std::size_t

namespace nav_core {

// ============================================================================
// 匿名命名空间：内部小工具
// ============================================================================

namespace {

constexpr double PI = 3.14159265358979323846;

/// 简单的角度解缠（弧度版）
///
/// 参数：
///   angle            : 当前角度（弧度）
///   cumulative_offset: 累计偏移，用于补偿 ±2π 跳变
///   last_unwrapped   : 上一次“已经解缠”的角度（弧度）
/// 返回：
///   当前 angle 对应的“连续角度”（弧度）
inline double unwrap_angle(double angle,
                           double& cumulative_offset,
                           double  last_unwrapped)
{
    const double ref   = last_unwrapped - cumulative_offset;
    double       delta = angle - ref;

    if (delta > PI) {
        cumulative_offset -= 2.0 * PI;
    } else if (delta < -PI) {
        cumulative_offset += 2.0 * PI;
    }

    return angle + cumulative_offset;
}

} // namespace

// ============================================================================
// 构造函数
// ============================================================================
//
// 主要做几件事：
//  - 检查采样率 / 截止频率是否合法，自动修正到合理范围；
//  - 计算一阶低通时间常数 tau；
//  - 根据 calib_seconds 计算“需要多少个样本用于零偏估计”；
//  - 初始化各种状态变量（零偏、低通状态、yaw 积分等）。
//
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
    , tau_(0.0)
    , do_calib_(calibrate)
    , calib_N_(0)
    , calib_count_(0)
    , acc_sum_{}
    , gyr_sum_dps_{}
    , accel_bias_{}
    , gyro_bias_dps_{}
    , lp_inited_(false)
    , acc_lp1_{}
    , acc_lp2_{}
    , gyro_lp1_dps_{}
    , gyro_lp2_dps_{}
    , yaw_rad_(0.0)
    , yaw_offset_(0.0)
    , last_ts_(-1.0)
    , default_dt_(0.0)
    , last_yaw_unwrapped_(0.0)
{
    // 采样率兜底
    if (fs_ <= 0.0) {
        fs_ = 100.0;  // 默认按 100 Hz 处理
    }

    // 截止频率兜底：
    //   - 不能太低（<=0），也不能超过 Nyquist 频率
    //   - 如果超出范围，就选一个相对保守的值（例如 4 Hz 或 fs_/4）
    if (cutoff_ <= 0.0 || cutoff_ >= 0.5 * fs_) {
        const double safe_fc = std::min(4.0, 0.25 * fs_);
        cutoff_ = safe_fc;
    }

    // 一阶低通时间常数 tau = 1 / (2πf_c)
    tau_ = 1.0 / (2.0 * PI * cutoff_);

    // 校准配置
    const double calib_samples_f = calib_seconds * fs_;
    if (calib_samples_f <= 1.0) {
        calib_N_ = 1;
    } else {
        calib_N_ = static_cast<int>(std::lround(calib_samples_f));
    }
    calib_count_ = 0;

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

// ============================================================================
// 内部工具：一阶低通滤波步进
// ============================================================================
//
// 离散一阶低通：
//   y[n] = y[n-1] + α * (x[n] - y[n-1])
//   α    = dt / (τ + dt)
//
void RealTimeImuFilterCpp::lowpass_step(std::array<double,3>&       y,
                                        const std::array<double,3>& x,
                                        double                      dt)
{
    const double alpha = dt / (tau_ + dt);
    const std::size_t N = y.size();
    for (std::size_t i = 0; i < N; ++i) {
        y[i] += alpha * (x[i] - y[i]);
    }
}

// ============================================================================
// 内部工具：可选角速度夹持
// ============================================================================
//
// clamp_gyro_dps_ > 0 时，将三轴角速度限制在 ±clamp_gyro_dps_ 范围内，
// 防止异常尖峰让 yaw 积分“炸飞”。
// 
void RealTimeImuFilterCpp::maybe_clamp_gyro(std::array<double,3>& gyr_dps)
{
    if (clamp_gyro_dps_ <= 0.0) {
        return;
    }

    const std::size_t N = gyr_dps.size();
    for (std::size_t i = 0; i < N; ++i) {
        if (gyr_dps[i] >  clamp_gyro_dps_) {
            gyr_dps[i] =  clamp_gyro_dps_;
        } else if (gyr_dps[i] < -clamp_gyro_dps_) {
            gyr_dps[i] = -clamp_gyro_dps_;
        }
    }
}

// ============================================================================
// 主接口：处理一帧 IMU 数据
// ============================================================================
//
// 输入：
//   - timestamp_s : 单调时间戳（秒），建议用 mono_ns * 1e-9
//   - acc_g       : 加速度（g）
//   - gyro_dps    : 角速度（deg/s）
//
// 输出：
//   - std::nullopt : 仍在校准阶段（零偏估计未完成）
//   - ImuFiltOutput：滤波后的 acc/gyro + 解缠后的 yaw（单位：deg）
//
std::optional<ImuFiltOutput> RealTimeImuFilterCpp::process(
        double                        timestamp_s,
        const std::array<double,3>&   acc_g,
        const std::array<double,3>&   gyro_dps)
{
    // -------- 1. 估算 dt --------
    double dt = default_dt_;
    if (last_ts_ >= 0.0) {
        const double raw_dt = timestamp_s - last_ts_;
        // 如果时间间隔异常（太小/太大），退回默认值，避免滤波发散
        const double dt_min = 0.2 * default_dt_;
        const double dt_max = 5.0 * default_dt_;
        if (raw_dt >= dt_min && raw_dt <= dt_max) {
            dt = raw_dt;
        }
    }
    last_ts_ = timestamp_s;

    // -------- 2. 校准阶段：估计零偏 --------
    //
    // 思路：
    //   - 前 calib_seconds 秒认为平台大致静止；
    //   - 对 acc_g / gyro_dps 做均值，得到零偏 accel_bias_ / gyro_bias_dps_;
    //   - 同时用这段数据初始化低通状态，以避免滤波起步时大阶跃。
    //
    if (do_calib_) {
        const std::size_t N = acc_sum_.size();
        for (std::size_t i = 0; i < N; ++i) {
            acc_sum_[i]     += acc_g[i];
            gyr_sum_dps_[i] += gyro_dps[i];
        }
        ++calib_count_;

        if (calib_count_ >= calib_N_) {
            // 计算零偏
            for (std::size_t i = 0; i < N; ++i) {
                accel_bias_[i]    = acc_sum_[i]     / static_cast<double>(calib_count_);
                gyro_bias_dps_[i] = gyr_sum_dps_[i] / static_cast<double>(calib_count_);
            }
            // 初始化低通状态
            for (std::size_t i = 0; i < N; ++i) {
                const double acc0 = acc_g[i]    - accel_bias_[i];
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

        // 校准期不输出任何结果，调用方看到 nullopt 就知道“再等等”
        return std::nullopt;
    }

    // -------- 3. 去偏 + 可选夹持 --------
    std::array<double,3> acc_corr{};
    std::array<double,3> gyr_corr_dps{};

    const std::size_t N = acc_corr.size();
    for (std::size_t i = 0; i < N; ++i) {
        acc_corr[i]     = acc_g[i]    - accel_bias_[i];
        gyr_corr_dps[i] = gyro_dps[i] - gyro_bias_dps_[i];
    }

    maybe_clamp_gyro(gyr_corr_dps);

    // -------- 4. 低通滤波 --------
    //
    // first order:
    //   y1 = LP(x)
    // pseudo 2nd order:
    //   y1 = LP(x)
    //   y2 = LP(y1)
    //
    if (!lp_inited_) {
        // 如果因为某些原因没初始化（例如禁用校准），这里兜底初始化一次
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

    // -------- 5. 航向积分（用 Z 轴角速度）--------
    //
    // 注意：
    //   - 输入 gyro_f_dps[2] 是 deg/s，需要转为 rad/s；
    //   - yaw_rad_ 直接积分；
    //   - unwrap_angle() 通过 yaw_offset_ 与 last_yaw_unwrapped_
    //     保证航向是连续的（不会卡在 [-π, π]）。
    //
    const double gz_rad = gyro_f_dps[2] * PI / 180.0;
    yaw_rad_ += gz_rad * dt;

    const double yaw_unwrapped = unwrap_angle(
        yaw_rad_,
        yaw_offset_,
        last_yaw_unwrapped_);

    last_yaw_unwrapped_ = yaw_unwrapped;
    const double yaw_deg = yaw_unwrapped * 180.0 / PI;

    // -------- 6. 组织输出 --------
    ImuFiltOutput out{};
    out.acc_f_g    = acc_f;       // 仍然是 g 单位
    out.gyro_f_dps = gyro_f_dps;  // deg/s
    out.yaw_deg    = yaw_deg;     // deg（连续）

    return out;
}

} // namespace nav_core
