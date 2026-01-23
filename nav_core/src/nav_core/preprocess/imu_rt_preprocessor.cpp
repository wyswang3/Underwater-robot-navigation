// nav_core/src/preprocess/imu_rt_preprocessor.cpp

#include "nav_core/preprocess/imu_rt_preprocessor.hpp"
#include "nav_core/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace nav_core::preprocess {

namespace {

using nav_core::Vec3d;
namespace math = nav_core::core::math;

// -------------------------- 小工具函数 --------------------------

// 一阶低通滤波系数：RC = 1/(2πfc), α = dt / (RC + dt)
inline double lpf_alpha(double dt, double cutoff_hz) noexcept
{
    if (cutoff_hz <= 0.0 || dt <= 0.0) {
        // 不启用 / 无有效 dt 时不做滤波
        return 1.0;
    }
    const double rc = 1.0 / (2.0 * math::kPi * cutoff_hz);
    return dt / (rc + dt);
}

// float[3] → Vec3d
inline Vec3d to_vec3d(const float v[3]) noexcept
{
    return Vec3d{
        static_cast<double>(v[0]),
        static_cast<double>(v[1]),
        static_cast<double>(v[2])
    };
}

// Vec3d → float[3]
inline void from_vec3d(const Vec3d& src, float dst[3]) noexcept
{
    dst[0] = static_cast<float>(src.x);
    dst[1] = static_cast<float>(src.y);
    dst[2] = static_cast<float>(src.z);
}

// 基本有效性检查：NaN / inf / 过大值过滤。
// 对应 hpp 中 ImuFilterConfig 的语义。
inline bool basic_valid(const ImuFilterConfig& cfg,
                        const ImuSample&       s) noexcept
{
    const float* g = s.ang_vel;
    const float* a = s.lin_acc;
    const float* e = s.euler;

    // 角速度检查
    if (cfg.enable_gyro) {
        for (int i = 0; i < 3; ++i) {
            const float v = g[i];
            if (!std::isfinite(v)) {
                return false;
            }
            if (cfg.max_abs_gyro > 0.0 &&
                std::fabs(static_cast<double>(v)) > cfg.max_abs_gyro)
            {
                return false;
            }
        }
    } else {
        // 即便关闭 gyro 检查，也至少做 NaN/inf 过滤
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(g[i])) {
                return false;
            }
        }
    }

    // 加速度检查
    if (cfg.enable_accel) {
        for (int i = 0; i < 3; ++i) {
            const float v = a[i];
            if (!std::isfinite(v)) {
                return false;
            }
            if (cfg.max_abs_accel > 0.0 &&
                std::fabs(static_cast<double>(v)) > cfg.max_abs_accel)
            {
                return false;
            }
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            if (!std::isfinite(a[i])) {
                return false;
            }
        }
    }

    // 欧拉角检查（可选）
    if (cfg.enable_euler) {
        for (int i = 0; i < 3; ++i) {
            const float v = e[i];
            if (!std::isfinite(v)) {
                return false;
            }
            if (cfg.max_euler_abs > 0.0 &&
                std::fabs(static_cast<double>(v)) > cfg.max_euler_abs)
            {
                return false;
            }
        }
    }

    return true;
}

// 传感器坐标系 S(RFU) -> 体坐标系 B(FRD) 线性变换：
//   X_b =  Y_s
//   Y_b =  X_s
//   Z_b = -Z_s
inline void sensor_to_body_vec3(const float s[3], float b[3]) noexcept
{
    const float xs = s[0];
    const float ys = s[1];
    const float zs = s[2];

    b[0] = ys;      // X_b
    b[1] = xs;      // Y_b
    b[2] = -zs;     // Z_b
}

// 同上变换应用到 bias：S → B（用于 state 里的 *_bias_body）
inline void sensor_bias_to_body(const float bias_s[3], float bias_b[3]) noexcept
{
    sensor_to_body_vec3(bias_s, bias_b);
}

// 根据 roll / pitch 计算体坐标系下的重力向量 g_b。
// 约定：
//   - 体坐标系 B = FRD：X 前, Y 右, Z 下；
//   - roll = 绕 X_b, pitch = 绕 Y_b；
//   - 采用常见航空 3-2-1 角定义下的近似表达式：
//       g_bx = -g * sin(pitch)
//       g_by =  g * sin(roll) * cos(pitch)
//       g_bz =  g * cos(roll) * cos(pitch)
inline void gravity_in_body(double roll, double pitch,
                            double g_ref,
                            float g_b[3]) noexcept
{
    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);

    const double gx = -g_ref * sp;
    const double gy =  g_ref * sr * cp;
    const double gz =  g_ref * cr * cp;

    g_b[0] = static_cast<float>(gx);
    g_b[1] = static_cast<float>(gy);
    g_b[2] = static_cast<float>(gz);
}

} // namespace

// ============================ ImuRtPreprocessor 实现 ============================

ImuRtPreprocessor::ImuRtPreprocessor(const ImuRtPreprocessConfig& cfg)
{
    cfg_ = cfg;
    reset();
}

void ImuRtPreprocessor::reset()
{
    st_ = ImuRtPreprocessState{};
    // 所有 bias / std / yaw_lpf_state_body 等都被清零
}

void ImuRtPreprocessor::setConfig(const ImuRtPreprocessConfig& cfg)
{
    cfg_ = cfg;
    // 更新配置不强制 reset，保留已有 bias 和状态。
}

ImuRtPreprocessConfig ImuRtPreprocessor::config() const
{
    return cfg_;
}

ImuRtPreprocessState ImuRtPreprocessor::state() const
{
    return st_;
}


bool ImuRtPreprocessor::biasReady() const noexcept
{
    return st_.bias_ready;
}

bool ImuRtPreprocessor::currentlyStatic() const noexcept
{
    return st_.currently_static;
}

bool ImuRtPreprocessor::process(const ImuSample& in, ImuSample& out)
{
    // 0) basic 有效性检查：失败直接丢弃该样本
    if (!basic_valid(cfg_.basic, in)) {
        return false;
    }

    const MonoTimeNs t_ns = in.mono_ns;
    double dt = 0.0;

    if (st_.last_mono_ns > 0 && t_ns > st_.last_mono_ns) {
        dt = static_cast<double>(t_ns - st_.last_mono_ns) * 1e-9; // ns → s
    }
    st_.last_mono_ns = t_ns;

    const float* gyro_s   = in.ang_vel;  // 传感器坐标系 S
    const float* accel_s  = in.lin_acc;  // 传感器坐标系 S

    // ----------------------------------------------------------
    // 1) 静止窗检测 + bias 估计（在传感器坐标系 S 下完成）
    // ----------------------------------------------------------
    bool is_static_now = false;

    {
        const Vec3d gyro_v  = to_vec3d(gyro_s);
        const Vec3d accel_v = to_vec3d(accel_s);

        const double gyro_norm  = math::norm(gyro_v);
        const double accel_norm = math::norm(accel_v);
        const double accel_dev  =
            std::fabs(accel_norm - static_cast<double>(cfg_.g_ref_m_s2));

        if (gyro_norm <= static_cast<double>(cfg_.static_max_abs_gyro) &&
            accel_dev <= static_cast<double>(cfg_.static_max_abs_accel_dev))
        {
            is_static_now = true;
        }
    }

    if (!st_.bias_ready) {
        if (is_static_now && dt > 0.0) {
            st_.currently_static      = true;
            st_.static_window_acc_s  += dt;

            const std::uint64_t max_samples =
                (cfg_.static_max_samples > 0)
                    ? cfg_.static_max_samples
                    : static_cast<std::uint64_t>(-1);

            if (st_.static_window_samples < max_samples) {
                ++st_.static_window_samples;
            }
            const double n = static_cast<double>(st_.static_window_samples);

            // 递推均值：mean_new = mean_old + (x - mean_old) / n
            for (int i = 0; i < 3; ++i) {
                const double old_g = static_cast<double>(st_.gyro_bias_sensor[i]);
                const double old_a = static_cast<double>(st_.accel_bias_sensor[i]);

                const double new_g = old_g + (static_cast<double>(gyro_s[i]) - old_g) / n;
                const double new_a = old_a + (static_cast<double>(accel_s[i]) - old_a) / n;

                st_.gyro_bias_sensor[i]  = static_cast<float>(new_g);
                st_.accel_bias_sensor[i] = static_cast<float>(new_a);

                // std 精确估计需要额外统计量，这里先置 0 保留接口
                st_.gyro_std_sensor[i]   = 0.0f;
                st_.accel_std_sensor[i]  = 0.0f;
            }

            // bias 估计时间累计达到阈值 → 确认 bias_ready
            if (st_.static_window_acc_s >= cfg_.static_window_s) {
                st_.bias_ready = true;

                // 同步更新“体坐标系下”的 bias（只用于诊断）
                sensor_bias_to_body(st_.gyro_bias_sensor,  st_.gyro_bias_body);
                sensor_bias_to_body(st_.accel_bias_sensor, st_.accel_bias_body);
            }
        } else {
            // 静止条件中断：清空静止窗统计，下次重新累计
            st_.currently_static      = false;
            st_.static_window_acc_s   = 0.0;
            st_.static_window_samples = 0;

            // 重置 bias / std
            for (int i = 0; i < 3; ++i) {
                st_.gyro_bias_sensor[i]  = 0.0f;
                st_.accel_bias_sensor[i] = 0.0f;
                st_.gyro_bias_body[i]    = 0.0f;
                st_.accel_bias_body[i]   = 0.0f;
                st_.gyro_std_sensor[i]   = 0.0f;
                st_.accel_std_sensor[i]  = 0.0f;
            }
        }
    } else {
        // bias 已经估计完成，静止窗只做“当前是否静止”的标记
        st_.currently_static = is_static_now;
    }

    // ----------------------------------------------------------
    // 2) 去 bias（在传感器坐标系 S 下）
    // ----------------------------------------------------------
    float gyro_corr_s[3];
    float accel_corr_s[3];

    for (int i = 0; i < 3; ++i) {
        if (st_.bias_ready) {
            gyro_corr_s[i]  = gyro_s[i] - st_.gyro_bias_sensor[i];
            accel_corr_s[i] = accel_s[i] - st_.accel_bias_sensor[i];
        } else {
            gyro_corr_s[i]  = gyro_s[i];
            accel_corr_s[i] = accel_s[i];
        }
    }

    // ----------------------------------------------------------
    // 3) 传感器坐标系 S → 体坐标系 B(FRD)
    // ----------------------------------------------------------
    float gyro_b[3];
    float accel_b[3];
    sensor_to_body_vec3(gyro_corr_s,  gyro_b);
    sensor_to_body_vec3(accel_corr_s, accel_b);

    // 同步更新体坐标系 bias（仅用于诊断）
    if (st_.bias_ready) {
        sensor_bias_to_body(st_.gyro_bias_sensor,  st_.gyro_bias_body);
        sensor_bias_to_body(st_.accel_bias_sensor, st_.accel_bias_body);
    }

    // ----------------------------------------------------------
    // 4) 重力扣除（体坐标系下）
    // ----------------------------------------------------------
    float accel_lin_b[3];  // 扣重力后的“线加速度”

    if (cfg_.enable_gravity_compensation && cfg_.use_imu_rp_from_sample) {
        const float roll  = in.euler[0];
        const float pitch = in.euler[1];

        if (std::isfinite(roll) && std::isfinite(pitch)) {
            float g_b[3];
            gravity_in_body(static_cast<double>(roll),
                            static_cast<double>(pitch),
                            static_cast<double>(cfg_.g_ref_m_s2),
                            g_b);

            for (int i = 0; i < 3; ++i) {
                accel_lin_b[i] = accel_b[i] - g_b[i];
            }
        } else {
            // roll / pitch 不可用，则不做重力扣除
            for (int i = 0; i < 3; ++i) {
                accel_lin_b[i] = accel_b[i];
            }
        }
    } else {
        // 不启用重力扣除
        for (int i = 0; i < 3; ++i) {
            accel_lin_b[i] = accel_b[i];
        }
    }

    // ----------------------------------------------------------
    // 5) 航向角速度滤波 + 全轴 deadband
    // ----------------------------------------------------------
    float gyro_out_b[3];
    float accel_out_b[3];

    // yaw 轴：体坐标系的 Z_b 分量
    const float raw_yaw_rate_body = gyro_b[2];

    // 5.1 yaw 轴一阶低通（只对 yaw 相关分量）
    float yaw_filtered = raw_yaw_rate_body;
    if (cfg_.yaw_lpf_cutoff_hz > 0.0f && dt > 0.0) {
        const double alpha_yaw =
            lpf_alpha(dt, static_cast<double>(cfg_.yaw_lpf_cutoff_hz));
        float& y = st_.yaw_lpf_state_body;  // 上一时刻输出
        const float x = raw_yaw_rate_body;  // 当前输入
        y = static_cast<float>(y + alpha_yaw * (x - y));
        yaw_filtered = y;
    } else {
        // 不启用 yaw 低通：状态直接跟随当前值
        st_.yaw_lpf_state_body = raw_yaw_rate_body;
        yaw_filtered           = raw_yaw_rate_body;
    }

    // 5.2 deadband（复用 math::apply_deadzone）
    const double base_gyro_db = (cfg_.gyro_deadband_rad_s > 0.0f)
                                ? static_cast<double>(cfg_.gyro_deadband_rad_s)
                                : 0.0;
    const double yaw_db = (cfg_.yaw_deadband_rad_s > 0.0f)
                          ? static_cast<double>(cfg_.yaw_deadband_rad_s)
                          : base_gyro_db;

    // X/Y 轴 gyro：只用全轴 deadband
    gyro_out_b[0] = static_cast<float>(
        math::apply_deadzone(static_cast<double>(gyro_b[0]), base_gyro_db));
    gyro_out_b[1] = static_cast<float>(
        math::apply_deadzone(static_cast<double>(gyro_b[1]), base_gyro_db));
    // Z 轴 gyro：使用 yaw 轴 deadband（一般更严格）
    gyro_out_b[2] = static_cast<float>(
        math::apply_deadzone(static_cast<double>(yaw_filtered), yaw_db));

    // 加速度 deadband：用 Vec3d + math::apply_deadzone(Vec3d&)
    {
        Vec3d acc_v = to_vec3d(accel_lin_b);
        const double accel_db =
            (cfg_.accel_deadband_m_s2 > 0.0f)
                ? static_cast<double>(cfg_.accel_deadband_m_s2)
                : 0.0;
        math::apply_deadzone(acc_v, accel_db);
        from_vec3d(acc_v, accel_out_b);
    }

    // ----------------------------------------------------------
    // 6) 构造输出 ImuSample（体坐标系 + 去 bias + 扣重力 + yaw 滤波）
    // ----------------------------------------------------------
    out = in;  // 复制时间戳 / euler / temperature / status 等

    for (int i = 0; i < 3; ++i) {
        out.ang_vel[i] = gyro_out_b[i];
        out.lin_acc[i] = accel_out_b[i];
    }

    // 这里的 valid 表示“通过 basic 检查并完成预处理”
    out.valid = true;

    // status 暂时直接沿用输入（未来可以在 types.hpp 定义 bitmask 后在此处 OR 自己的标志）
    // out.status = in.status;

    return true;
}

} // namespace nav_core::preprocess
