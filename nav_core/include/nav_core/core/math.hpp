// nav_core/include/nav_core/core/math.hpp
//
// @file  math.hpp
// @brief nav_core 在线导航 / 离线因子图公用的基础数学工具。
// 
// 设计目标：
//   - 提供与“水下导航 + 轨迹积分 + 因子图平滑”强相关的、
//     轻量级数学工具函数；
//   - 仅依赖标准库和 nav_core::Vec3f/Vec3d，不直接依赖
//     第三方线性代数库（Eigen 等）；
//   - 头文件实现（header-only），便于在 drivers / estimator / io 中复用；
//   - 不承担“完整线性代数库”的职责，因子图内部的矩阵运算、
//     线性方程求解仍交由 estimator/*.cpp 中的实现处理。
// 
// 典型用途：
//   - 在线导航：
//       * IMU yaw 积分中的角度 wrap / unwrap；
//       * Body 速度 → ENU 速度的 yaw-only 旋转；
//       * ZUPT 零速判定；
//       * 轨迹梯形积分（配合 Kahan 补偿器）；
//       * DVL / IMU 噪声地板 & deadzone 处理；
//       * 滑动平均平滑 yaw / 速度 / 加速度；
//   - 离线因子图：
//       * 轨迹误差的 RMS 评估；
//       * 简单的向量工具（模长、归一化等）。
//

#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "nav_core/core/types.hpp"  // Vec3f / Vec3d

namespace nav_core::core::math {

// ===================== 1. 常量与角度工具 =====================

/// @brief π 常量（double 精度）。
inline constexpr double kPi = 3.14159265358979323846;

/// @brief 2π 常量（double 精度）。
inline constexpr double kTwoPi = 2.0 * kPi;

/// @brief 将角度归一化到 (-π, π] 区间。
///
/// 典型用法：
///   - IMU yaw 积分后，对结果做 wrap，避免数值发散；
inline double wrap_angle_rad(double angle) noexcept
{
    double x = std::fmod(angle, kTwoPi);
    if (x <= -kPi) x += kTwoPi;
    else if (x > kPi) x -= kTwoPi;
    return x;
}

/// @brief 对“被 wrap 到 (-π, π] 的角度序列”做 unwrap，得到连续角度。
///
/// @param prev_unwrapped 上一时刻的“连续角度”（未 wrap）
/// @param current_wrapped 当前测得的 angle，位于 (-π, π] 区间
/// @return 更新后的连续角度
inline double unwrap_angle_rad(double prev_unwrapped, double current_wrapped) noexcept
{
    double cw = wrap_angle_rad(current_wrapped);
    double pw = wrap_angle_rad(prev_unwrapped);

    double delta = cw - pw;
    if (delta > kPi) {
        delta -= kTwoPi;
    } else if (delta < -kPi) {
        delta += kTwoPi;
    }
    return prev_unwrapped + delta;
}

// ===================== 2. 向量工具（与 Vec3f/Vec3d 配合） =====================

inline double norm(const Vec3d& v) noexcept
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline float norm(const Vec3f& v) noexcept
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

inline Vec3d normalize(const Vec3d& v, double eps = 1e-9) noexcept
{
    const double n = norm(v);
    if (n < eps) {
        return v;
    }
    return Vec3d{v.x / n, v.y / n, v.z / n};
}

inline Vec3f normalize(const Vec3f& v, float eps = 1e-6f) noexcept
{
    const float n = norm(v);
    if (n < eps) {
        return v;
    }
    return Vec3f{v.x / n, v.y / n, v.z / n};
}

inline double clamp(double x, double lo, double hi) noexcept
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

inline float clamp(float x, float lo, float hi) noexcept
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}


// ===================== 3. 坐标系变换：Body → ENU（Yaw-only） =====================

/// @brief 根据 yaw 角（rad）进行「平面旋转」，将体坐标系速度投影到 ENU。
///
/// 假设：仅考虑 yaw 旋转（水平面），忽略 roll/pitch 对 x-y 分量的耦合。
inline void body_to_enu_yaw_only(double yaw_rad,
                                 const Vec3d& v_body,
                                 Vec3d& v_enu_out) noexcept
{
    const double c = std::cos(yaw_rad);
    const double s = std::sin(yaw_rad);

    const double vE =  c * v_body.x - s * v_body.y;
    const double vN =  s * v_body.x + c * v_body.y;
    const double vU =  v_body.z;  // 假设 body-z 与 ENU-Up 基本一致

    v_enu_out.x = vE;
    v_enu_out.y = vN;
    v_enu_out.z = vU;
}

// ===================== 4. 零速判定（ZUPT） =====================

/// @brief 基于 ENU 速度模长的 ZUPT 判定。
inline bool is_zupt(const Vec3d& v_enu, double threshold_mps) noexcept
{
    return norm(v_enu) < threshold_mps;
}

// ===================== 5. Kahan 补偿求和（标量 & 3D 向量） =====================

/// @brief Kahan 补偿求和（标量），用于减少长时间积分的数值误差。
struct KahanAccumulator {
    double sum{0.0};
    double c{0.0};

    inline void add(double x) noexcept {
        double y = x - c;
        double t = sum + y;
        c = (t - sum) - y;
        sum = t;
    }
};

/// @brief 针对 3D 向量的 Kahan 补偿求和。
struct KahanVec3Accumulator {
    Vec3d  sum{0.0, 0.0, 0.0};
    Vec3d  c  {0.0, 0.0, 0.0};

    inline void add(const Vec3d& x) noexcept {
        const double yx = x.x - c.x;
        const double yy = x.y - c.y;
        const double yz = x.z - c.z;

        const double tx = sum.x + yx;
        const double ty = sum.y + yy;
        const double tz = sum.z + yz;

        c.x = (tx - sum.x) - yx;
        c.y = (ty - sum.y) - yy;
        c.z = (tz - sum.z) - yz;

        sum.x = tx;
        sum.y = ty;
        sum.z = tz;
    }
};

// ===================== 6. 速度积分 → 位移（梯形 + 可选 Kahan） =====================

/// @brief 对 ENU 速度做一次 3D 梯形积分：p_{k+1} = p_k + 0.5*(v_k + v_{k+1})*dt。
inline void integrate_trapezoid(double dt,
                                const Vec3d& v0,
                                const Vec3d& v1,
                                Vec3d& p_inout,
                                KahanVec3Accumulator* acc = nullptr) noexcept
{
    const Vec3d delta{
        0.5 * (v0.x + v1.x) * dt,
        0.5 * (v0.y + v1.y) * dt,
        0.5 * (v0.z + v1.z) * dt
    };

    if (acc) {
        acc->add(delta);
    }
    p_inout.x += delta.x;
    p_inout.y += delta.y;
    p_inout.z += delta.z;
}

// ===================== 7. 误差指标（RMS） =====================

/// @brief 计算两个标量序列的均方根误差（RMS）。
inline double rms_error(const double* a, const double* b, std::size_t n) noexcept
{
    if (!a || !b || n == 0) {
        return 0.0;
    }
    double sum_sq = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double d = a[i] - b[i];
        sum_sq += d * d;
    }
    return std::sqrt(sum_sq / static_cast<double>(n));
}

// ===================== 8. 噪声地板 & 阈值滤波（DVL / IMU） =====================
//
// 这一节专门放与传感器物理精度相关的“工程阈值”，
// 例如：DVL 0.4% ± 5mm/s 的噪声地板、IMU 静止 bias/极小值 deadzone。
// 这些函数本质上是“基于传感器精度的硬剪裁”，用来在进入高层算法前
// 先把明显属于噪声的量归零，避免长时间积分/因子图被低幅噪声拖偏。

// ---- 8.1 DVL 速度噪声地板 ----

/// @brief DVL 测速噪声地板（工程经验值）。
///
/// 说明：
///   - 厂家指标：测量精度约 0.4% ± 5mm/s；
///   - 结合你当前数据，经验上“总速度 < 0.08m/s”时，
///     很可能只是噪声 / 抖动，而不是有效航迹变化；
///   - 因此，可以在在线导航中对 |v| < 0.08 m/s 的 DVL 速度做“硬置零”。
inline constexpr double kDvlNoiseFloorMps = 0.08;  ///< 默认噪声地板阈值（m/s）

/// @brief 对 ENU 速度向量应用 DVL 噪声地板：
///        若 |v| < noise_floor，则整向量置零。
///
/// @param v_enu       ENU 速度向量（输入 / 输出）
/// @param noise_floor 阈值（m/s），默认使用 kDvlNoiseFloorMps
/// @return true 表示发生了“置零”操作；false 表示保持原值。
inline bool apply_dvl_noise_floor(Vec3d& v_enu,
                                  double noise_floor = kDvlNoiseFloorMps) noexcept
{
    if (noise_floor <= 0.0) {
        return false;  // 不启用
    }
    if (norm(v_enu) < noise_floor) {
        v_enu.x = 0.0;
        v_enu.y = 0.0;
        v_enu.z = 0.0;
        return true;
    }
    return false;
}

// ---- 8.2 通用 deadzone 工具（IMU / 其他传感器都可用） ----

/// @brief 标量 deadzone：若 |x| < threshold，则返回 0。
inline double apply_deadzone(double x, double threshold) noexcept
{
    if (threshold <= 0.0) {
        return x;
    }
    return (std::fabs(x) < threshold) ? 0.0 : x;
}

/// @brief 3D 向量逐分量 deadzone：若 |v[i]| < threshold，则该分量置零。
///
/// 典型用法：
///   - 对 IMU 静止段统计出的 bias 进行补偿后，对残余极小值再做 deadzone；
///   - 对极小角速度 / 加速度分量做剪裁，避免随机游走慢慢积分。
inline void apply_deadzone(Vec3d& v, double threshold) noexcept
{
    if (threshold <= 0.0) {
        return;
    }
    v.x = (std::fabs(v.x) < threshold) ? 0.0 : v.x;
    v.y = (std::fabs(v.y) < threshold) ? 0.0 : v.y;
    v.z = (std::fabs(v.z) < threshold) ? 0.0 : v.z;
}

// ---- 8.3 IMU bias 补偿 + deadzone ----

/// @brief 对 IMU 测量值做“静止 bias 补偿 + deadzone”
///
/// @param meas_inout  输入：原始测量值；输出：补偿 + deadzone 后的值
/// @param bias        静止窗估计得到的 bias（例如 gyro bias / accel bias）
/// @param deadzone    剩余量的 deadzone 阈值（典型：静止时残余噪声上界）
///
/// 典型用法：
///   - gyro：
///       * 在静止窗上估计 ω_bias；
///       * 每帧：meas = ω_raw - ω_bias；
///       * 再对 meas 应用 deadzone（例如 0.001 rad/s），
///         抑制长期随机游走；
///   - accel：
///       * 在静止窗上估计 a_bias（含重力）；
///       * 根据需要做重力扣除，之后对极小线加速度做 deadzone。
inline void compensate_bias_and_deadzone(Vec3d& meas_inout,
                                         const Vec3d& bias,
                                         double deadzone) noexcept
{
    // 先减 bias
    meas_inout.x -= bias.x;
    meas_inout.y -= bias.y;
    meas_inout.z -= bias.z;

    // 再应用 deadzone
    apply_deadzone(meas_inout, deadzone);
}

// ===================== 9. 滑动平均窗口（标量 & 3D 向量） =====================
//
// 主要用途：
//   - 对 yaw_nav、ENU 速度、线加速度做简单平滑，抑制短时抖动；
//   - 窗口长度一般设置为“若干采样周期”，例如 fs≈100Hz 时，
//     可选 win=5~15 对应 0.05~0.15s 的时间常数。
//   - 底层实现为环形缓冲区 + 累计和，构造时一次性分配内存，
//     在线调用 add() 时不再做动态分配。

/// @brief 标量滑动平均（定长窗口）。
struct SlidingMeanScalar {
    std::size_t window_size{0};
    std::size_t count{0};
    std::size_t index{0};
    double      sum{0.0};
    std::vector<double> buffer;

    SlidingMeanScalar() = default;

    explicit SlidingMeanScalar(std::size_t win)
    {
        reset(win);
    }

    /// @brief 重置窗口长度并清空历史。
    inline void reset(std::size_t win)
    {
        window_size = win;
        count = 0;
        index = 0;
        sum = 0.0;
        buffer.assign(window_size > 0 ? window_size : 0, 0.0);
    }

    /// @brief 是否已经有效（窗口长度 > 0）。
    [[nodiscard]] inline bool valid() const noexcept
    {
        return window_size > 0 && !buffer.empty();
    }

    /// @brief 添加一个新样本，并返回当前滑动平均值。
    ///
    /// 若窗口未初始化（window_size==0），则直接返回 x。
    inline double add(double x) noexcept
    {
        if (!valid()) {
            return x;
        }

        if (count < window_size) {
            // 填充阶段
            buffer[count] = x;
            sum += x;
            ++count;
            index = count % window_size;
        } else {
            // 环形覆盖
            const double old = buffer[index];
            buffer[index] = x;
            sum += x - old;
            index = (index + 1) % window_size;
        }

        return sum / static_cast<double>(count);
    }
};

/// @brief Vec3d 滑动平均（逐分量平均）。
struct SlidingMeanVec3 {
    std::size_t window_size{0};
    std::size_t count{0};
    std::size_t index{0};
    Vec3d       sum{0.0, 0.0, 0.0};
    std::vector<Vec3d> buffer;

    SlidingMeanVec3() = default;

    explicit SlidingMeanVec3(std::size_t win)
    {
        reset(win);
    }

    /// @brief 重置窗口长度并清空历史。
    inline void reset(std::size_t win)
    {
        window_size = win;
        count = 0;
        index = 0;
        sum = Vec3d{0.0, 0.0, 0.0};
        buffer.assign(window_size > 0 ? window_size : 0, Vec3d{0.0, 0.0, 0.0});
    }

    /// @brief 是否已经有效（窗口长度 > 0）。
    [[nodiscard]] inline bool valid() const noexcept
    {
        return window_size > 0 && !buffer.empty();
    }
    inline void apply_deadzone(Vec3f& v, float threshold) noexcept
    {
        if (threshold <= 0.0f) return;
        v.x = (std::fabs(v.x) < threshold) ? 0.0f : v.x;
        v.y = (std::fabs(v.y) < threshold) ? 0.0f : v.y;
        v.z = (std::fabs(v.z) < threshold) ? 0.0f : v.z;
    }    

    inline void compensate_bias_and_deadzone(Vec3f& meas_inout,
                                             const Vec3f& bias,
                                             float deadzone) noexcept
    {
        meas_inout.x -= bias.x;
        meas_inout.y -= bias.y;
        meas_inout.z -= bias.z;
        apply_deadzone(meas_inout, deadzone);
    }

    /// @brief 添加一个新向量样本，并返回当前滑动平均值。
    ///
    /// 若窗口未初始化（window_size==0），则直接返回 x。
    inline Vec3d add(const Vec3d& x) noexcept
    {
        if (!valid()) {
            return x;
        }

        if (count < window_size) {
            // 填充阶段
            buffer[count] = x;
            sum.x += x.x;
            sum.y += x.y;
            sum.z += x.z;
            ++count;
            index = count % window_size;
        } else {
            // 环形覆盖
            const Vec3d& old = buffer[index];
            buffer[index] = x;

            sum.x += x.x - old.x;
            sum.y += x.y - old.y;
            sum.z += x.z - old.z;

            index = (index + 1) % window_size;
        }

        const double inv = 1.0 / static_cast<double>(count);
        return Vec3d{sum.x * inv, sum.y * inv, sum.z * inv};
    }
    
};

} // namespace nav_core::core::math
