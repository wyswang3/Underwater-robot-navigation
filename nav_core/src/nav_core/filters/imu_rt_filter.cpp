// nav_core/src/filters/imu_rt_filter.cpp

#include "nav_core/filters/imu_rt_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace nav_core::filters {

namespace {

// ======================= 小工具函数（仅在本文件内可见） =======================

inline double norm3(const float v[3]) noexcept
{
    return std::sqrt(
        static_cast<double>(v[0]) * v[0] +
        static_cast<double>(v[1]) * v[1] +
        static_cast<double>(v[2]) * v[2]
    );
}

inline void apply_deadband3(float v[3], float threshold) noexcept
{
    if (threshold <= 0.0f) {
        return;
    }
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(v[i]) < threshold) {
            v[i] = 0.0f;
        }
    }
}

/// 一阶低通滤波系数：
///   RC = 1/(2πfc), α = dt / (RC + dt)
inline double lpf_alpha(double dt, double cutoff_hz) noexcept
{
    if (cutoff_hz <= 0.0 || dt <= 0.0) {
        // 不启用/无有效 dt 时不做滤波（外层逻辑会特殊处理）
        return 1.0;
    }
    const double rc = 1.0 / (2.0 * 3.141592653589793 * cutoff_hz);
    return dt / (rc + dt);
}

/// basic 有效性检查：NaN / inf / 过大值过滤。
///
/// 这里完全按你给的 ImuFilterConfig 和 ImuFrame 字段名实现：
///   - accel 检查：lin_acc[3] vs max_abs_accel
///   - gyro  检查：ang_vel[3] vs max_abs_gyro
///   - euler 检查：euler[3] vs max_euler_abs（可选）
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
    } else {
        // 不强制检查欧拉角；如果你希望这里也过滤 NaN，可按需加
    }

    return true;
}

} // namespace

// ============================ ImuRtFilter 实现 ============================

ImuRtFilter::ImuRtFilter(const ImuRtFilterConfig& cfg)
{
    cfg_ = cfg;
    reset();
}

void ImuRtFilter::reset()
{
    // 全部状态恢复默认：
    st_ = ImuRtFilterState{};

    // bias / lpf_state 都在 ImuRtFilterState 里，已经随之清零
    // 静止窗统计:
    //   - static_window_acc_s
    //   - static_window_samples
    // 也一并归零
}

void ImuRtFilter::setConfig(const ImuRtFilterConfig& cfg)
{
    cfg_ = cfg;
    // 按当前设计：更新配置不强制 reset，保留已估计的 bias 和 LPF 状态。
}

ImuRtFilterConfig ImuRtFilter::config() const
{
    return cfg_;
}

ImuRtFilterState ImuRtFilter::state() const
{
    return st_;
}

bool ImuRtFilter::biasReady() const noexcept
{
    return st_.bias_ready;
}

bool ImuRtFilter::currentlyStatic() const noexcept
{
    return st_.currently_static;
}

bool ImuRtFilter::process(const ImuSample& in, ImuSample& out)
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

    const float* g_raw = in.ang_vel;
    const float* a_raw = in.lin_acc;

    // ----------------------------------------------------------
    // 1) 静止窗检测 + bias 估计（仅在 bias 未准备好时运行）
    // ----------------------------------------------------------
    bool is_static_now = false;

    {
        // 角速度模长
        const double gyro_norm = norm3(g_raw);

        // 加速度模长与 g_ref 偏差
        const double accel_norm = norm3(a_raw);
        const double accel_dev  = std::fabs(accel_norm - cfg_.g_ref_m_s2);

        if (gyro_norm <= static_cast<double>(cfg_.static_max_abs_gyro) &&
            accel_dev <= static_cast<double>(cfg_.static_max_abs_accel_dev))
        {
            is_static_now = true;
        }
    }

    if (!st_.bias_ready) {
        if (is_static_now && dt > 0.0) {
            // 仍在静止窗口内，累积时间/样本，用“递推均值”估计 bias
            st_.currently_static = true;
            st_.static_window_acc_s    += dt;
            st_.static_window_samples  += 1;

            const double n = static_cast<double>(st_.static_window_samples);

            // 递推均值：mean_new = mean_old + (x - mean_old) / n
            for (int i = 0; i < 3; ++i) {
                const double old_g = st_.gyro_bias[i];
                const double old_a = st_.accel_bias[i];

                const double new_g = old_g + (static_cast<double>(g_raw[i]) - old_g) / n;
                const double new_a = old_a + (static_cast<double>(a_raw[i]) - old_a) / n;

                st_.gyro_bias[i]  = static_cast<float>(new_g);
                st_.accel_bias[i] = static_cast<float>(new_a);
            }

            // 时间累计达到阈值 → 确认 bias_ready
            if (st_.static_window_acc_s >= cfg_.static_window_s) {
                st_.bias_ready = true;
                // 此时 st_.gyro_bias / st_.accel_bias 就是静止窗平均值
            }
        } else {
            // 静止条件中断：清空静止窗统计，下次重新累计
            st_.currently_static      = false;
            st_.static_window_acc_s   = 0.0;
            st_.static_window_samples = 0;

            // bias 数值可以保留为“最后一次部分估计”，也可以清零。
            // 这里选择清零，让下一个静止窗完全重新估计。
            for (int i = 0; i < 3; ++i) {
                st_.gyro_bias[i]  = 0.0f;
                st_.accel_bias[i] = 0.0f;
            }
        }
    } else {
        // bias 已经估计完成，静止窗只做“当前是否静止”的标记
        st_.currently_static = is_static_now;
    }

    // ----------------------------------------------------------
    // 2) 去 bias
    // ----------------------------------------------------------
    float gyro_corr[3];
    float accel_corr[3];

    for (int i = 0; i < 3; ++i) {
        gyro_corr[i]  = g_raw[i] - st_.gyro_bias[i];
        accel_corr[i] = a_raw[i] - st_.accel_bias[i];
    }

    // ----------------------------------------------------------
    // 3) 一阶低通：dt>0 且 cutoff>0 时启用；否则直接透传
    // ----------------------------------------------------------
    float gyro_f[3];
    float accel_f[3];

    // 角速度 LPF
    if (cfg_.gyro_lpf_cutoff_hz > 0.0f && dt > 0.0) {
        const double alpha_g = lpf_alpha(dt, cfg_.gyro_lpf_cutoff_hz);
        for (int i = 0; i < 3; ++i) {
            float& y = st_.gyro_lpf_state[i];  // 上一时刻输出
            const float x = gyro_corr[i];      // 当前输入
            y = static_cast<float>(y + alpha_g * (x - y));
            gyro_f[i] = y;
        }
    } else {
        // 不启用 LPF：状态直接跟随当前值
        for (int i = 0; i < 3; ++i) {
            st_.gyro_lpf_state[i] = gyro_corr[i];
            gyro_f[i]             = gyro_corr[i];
        }
    }

    // 加速度 LPF
    if (cfg_.accel_lpf_cutoff_hz > 0.0f && dt > 0.0) {
        const double alpha_a = lpf_alpha(dt, cfg_.accel_lpf_cutoff_hz);
        for (int i = 0; i < 3; ++i) {
            float& y = st_.accel_lpf_state[i];
            const float x = accel_corr[i];
            y = static_cast<float>(y + alpha_a * (x - y));
            accel_f[i] = y;
        }
    } else {
        for (int i = 0; i < 3; ++i) {
            st_.accel_lpf_state[i] = accel_corr[i];
            accel_f[i]             = accel_corr[i];
        }
    }

    // ----------------------------------------------------------
    // 4) deadband：抑制极小随机游走
    // ----------------------------------------------------------
    apply_deadband3(gyro_f,  cfg_.gyro_deadband_rad_s);
    apply_deadband3(accel_f, cfg_.accel_deadband_m_s2);

    // ----------------------------------------------------------
    // 5) 构造输出 ImuSample
    // ----------------------------------------------------------
    out = in;  // 复制时间戳 / euler / temperature / status 等

    for (int i = 0; i < 3; ++i) {
        out.ang_vel[i] = gyro_f[i];
        out.lin_acc[i] = accel_f[i];
    }

    // 这里的 valid 表示“通过 basic 检查并完成滤波”
    out.valid = true;

    // 若你后面想定义 status bit，例如：
    //   bit0 = valid_raw, bit1 = filtered_ok, bit2 = bias_ready 等
    // 可以在 types.hpp 里定义枚举，然后这里按位 OR 即可。
    // 目前先保留原 status：
    // out.status = in.status;

    return true;
}

} // namespace nav_core::filters
