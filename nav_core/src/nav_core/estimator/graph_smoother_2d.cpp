// nav_core/src/estimator/graph_smoother_2d.cpp
//
// 2D 因子图平滑器实现：
//   - 状态：对每个样本 k 定义 [x_k, y_k, vx_k, vy_k]（ENU 平面）；
//   - 因子：
//       * 先验：首节点位置/速度、末节点位置（软锚点）；
//       * 过程：位置梯形积分约束 + 速度平滑（加速度白噪声）；
//       * 观测：DVL 速度观测；
//       * ZUPT：对标记 zupt_hint 的节点施加“速度≈0”的软约束；
//   - 求解：构建正规方程 H x = g（加权最小二乘），
//           使用简单的稠密 Cholesky 求解。
// 
// 说明：
//   - 这是一个“离线 / 准在线”的工具类，不涉及线程 / IO；
//   - 若后续需要更高性能，可在不改接口的前提下，
//     将内部求解替换为 Eigen / SuiteSparse 等库。

#include "nav_core/estimator/graph_smoother_2d.hpp"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace nav_core::estimator {

// ============================ 工具函数（内部匿名命名空间） ============================

namespace {

// 将 (k, dim) 映射为全局状态索引：
//   state_k = [x, y, vx, vy] -> dim = 0,1,2,3
inline std::size_t state_index(std::size_t k, std::size_t dim) noexcept
{
    return 4 * k + dim;
}

// 向正规方程 H x = g 中加入一个“单行因子”
//
// 因子形式：
//   r(x) = a^T x - b
//   cost = w * r^2
//
// 其中：
//   - idx[i] 为 a 向量中非零项对应的状态索引；
//   - a[i]   为该项系数；
//   - b      为观测值；
//   - w      为权重（= 1/sigma^2）。
//
// 贡献：
//   H += w * a a^T
//   g += w * a * b
inline void add_factor_row(std::vector<double>& H,
                           std::vector<double>& g,
                           const std::size_t* idx,
                           const double*       a,
                           std::size_t         m,
                           double              b,
                           double              w)
{
    if (w <= 0.0 || m == 0) {
        return;
    }

    const std::size_t nvar = g.size();

    // H += w * a a^T（只构造下三角）
    for (std::size_t i = 0; i < m; ++i) {
        const std::size_t ii = idx[i];
        const double ai = a[i];
        const double w_ai = w * ai;

        for (std::size_t j = i; j < m; ++j) {
            const std::size_t jj = idx[j];
            const double aj = a[j];
            const double val = w_ai * aj;

            // 下三角
            H[ii * nvar + jj] += val;
            if (ii != jj) {
                // 对称上三角
                H[jj * nvar + ii] += val;
            }
        }
    }

    // g += w * a * b
    const double wb = w * b;
    for (std::size_t i = 0; i < m; ++i) {
        const std::size_t ii = idx[i];
        g[ii] += wb * a[i];
    }
}

// 简单稠密 Cholesky 分解 + 求解：
//   输入：对称正定矩阵 H (n×n) 和向量 g (n)
//   输出：解向量 x (n)，满足 H x ≈ g。
//
// 实现说明：
//   - 原地将 H 分解为 L（下三角），存放在 H 的下三角部分；
//   - 上三角不再使用；
//   - 然后解 L y = g，再解 L^T x = y。
bool cholesky_solve(std::vector<double>& H,
                    const std::vector<double>& g,
                    std::vector<double>& x)
{
    const std::size_t n = g.size();
    if (H.size() != n * n || x.size() != n) {
        return false;
    }

    // 1) Cholesky 分解：H = L L^T
    for (std::size_t i = 0; i < n; ++i) {
        // 对角元
        double sum = H[i * n + i];
        for (std::size_t k = 0; k < i; ++k) {
            const double Lik = H[i * n + k];
            sum -= Lik * Lik;
        }
        if (sum <= 0.0) {
            // 非正定 / 数值问题
            return false;
        }
        const double Lii = std::sqrt(sum);
        H[i * n + i] = Lii;

        // 下三角其余元素
        for (std::size_t j = i + 1; j < n; ++j) {
            double s = H[j * n + i];
            for (std::size_t k = 0; k < i; ++k) {
                s -= H[j * n + k] * H[i * n + k];
            }
            H[j * n + i] = s / Lii;
        }
    }

    // 2) 前代：L y = g
    std::vector<double> y(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        double sum = g[i];
        for (std::size_t k = 0; k < i; ++k) {
            sum -= H[i * n + k] * y[k];
        }
        const double Lii = H[i * n + i];
        y[i] = sum / Lii;
    }

    // 3) 回代：L^T x = y
    for (std::size_t i = n; i-- > 0;) {
        double sum = y[i];
        const double Lii = H[i * n + i];
        for (std::size_t k = i + 1; k < n; ++k) {
            sum -= H[k * n + i] * x[k]; // 注意 L^T(i,k) = L(k,i)
        }
        x[i] = sum / Lii;
    }

    return true;
}

} // namespace（匿名）

// ============================ GraphSmoother2D 成员实现 ============================

GraphSmoother2D::GraphSmoother2D(const GraphSmoother2DConfig& cfg)
    : cfg_(cfg)
{
}

void GraphSmoother2D::clear()
{
    samples_.clear();
    states_.clear();
    solved_ = false;
}

void GraphSmoother2D::setSamples(const std::vector<SmootherInputSample2D>& samples)
{
    clear();
    samples_ = samples;

    // 简单检查时间是否非递减（若不满足，仅打印警告，不打断）
    bool strictly_inc = true;
    for (std::size_t i = 1; i < samples_.size(); ++i) {
        if (!(samples_[i].t_ns > samples_[i - 1].t_ns)) {
            strictly_inc = false;
            break;
        }
    }
    if (!strictly_inc) {
        std::cerr << "[GraphSmoother2D] WARNING: samples are not strictly "
                  << "time-increasing; behavior may be undefined.\n";
    }
}

void GraphSmoother2D::addSample(const SmootherInputSample2D& sample)
{
    samples_.push_back(sample);
    solved_ = false;
}

// ============================ solve(): 核心因子图构建 + 求解 ============================

bool GraphSmoother2D::solve()
{
    solved_ = false;
    states_.clear();

    const std::size_t N = samples_.size();
    if (N < 2) {
        // 至少需要 2 个节点才能构建过程因子
        return false;
    }

    const std::size_t nvar = 4 * N; // 每个结点 4 维：[x, y, vx, vy]

    // 正规方程 H x = g
    std::vector<double> H(nvar * nvar, 0.0);
    std::vector<double> g(nvar, 0.0);

    // -------- 1) 先验因子：首/末结点 soft anchor --------

    // 1.1 pos0 先验：x_0 ≈ 0, y_0 ≈ 0
    if (cfg_.use_prior_pos0 && cfg_.prior_pos0_sigma > 0.0) {
        const double w_pos0 = 1.0 / (cfg_.prior_pos0_sigma * cfg_.prior_pos0_sigma);

        const std::size_t idx_x0 = state_index(0, 0);
        const std::size_t idx_y0 = state_index(0, 1);
        {
            // x0 ≈ 0
            std::size_t idxs[1] = { idx_x0 };
            double      coeff[1] = { 1.0 };
            add_factor_row(H, g, idxs, coeff, 1, 0.0, w_pos0);
        }
        {
            // y0 ≈ 0
            std::size_t idxs[1] = { idx_y0 };
            double      coeff[1] = { 1.0 };
            add_factor_row(H, g, idxs, coeff, 1, 0.0, w_pos0);
        }
    }

    // 1.2 vel0 先验：vx_0 ≈ 0, vy_0 ≈ 0
    if (cfg_.use_prior_vel0 && cfg_.prior_vel0_sigma > 0.0) {
        const double w_vel0 = 1.0 / (cfg_.prior_vel0_sigma * cfg_.prior_vel0_sigma);

        const std::size_t idx_vx0 = state_index(0, 2);
        const std::size_t idx_vy0 = state_index(0, 3);
        {
            // vx0 ≈ 0
            std::size_t idxs[1] = { idx_vx0 };
            double      coeff[1] = { 1.0 };
            add_factor_row(H, g, idxs, coeff, 1, 0.0, w_vel0);
        }
        {
            // vy0 ≈ 0
            std::size_t idxs[1] = { idx_vy0 };
            double      coeff[1] = { 1.0 };
            add_factor_row(H, g, idxs, coeff, 1, 0.0, w_vel0);
        }
    }

    // 1.3 posN 先验：x_{N-1} ≈ 0, y_{N-1} ≈ 0（可选）
    if (cfg_.use_prior_posN && cfg_.prior_posN_sigma > 0.0) {
        const double w_posN = 1.0 / (cfg_.prior_posN_sigma * cfg_.prior_posN_sigma);

        const std::size_t kN  = N - 1;
        const std::size_t idx_xN = state_index(kN, 0);
        const std::size_t idx_yN = state_index(kN, 1);

        {
            std::size_t idxs[1] = { idx_xN };
            double      coeff[1] = { 1.0 };
            add_factor_row(H, g, idxs, coeff, 1, 0.0, w_posN);
        }
        {
            std::size_t idxs[1] = { idx_yN };
            double      coeff[1] = { 1.0 };
            add_factor_row(H, g, idxs, coeff, 1, 0.0, w_posN);
        }
    }

    // -------- 2) 过程因子：相邻结点之间的运动学约束 --------

    const double sigma_acc  = std::max(cfg_.process_acc_sigma, 1e-6);
    const double sigma_pos  = std::max(cfg_.process_pos_sigma, 1e-6);

    for (std::size_t k = 0; k + 1 < N; ++k) {
        const auto& s0 = samples_[k];
        const auto& s1 = samples_[k + 1];

        if (s1.t_ns <= s0.t_ns) {
            continue;
        }

        const double dt_s = static_cast<double>(s1.t_ns - s0.t_ns) * 1e-9;

        if (dt_s <= 0.0 || dt_s > cfg_.max_knot_dt_s) {
            // 时间跨度太大，则跳过该过程因子（避免数值过重）
            continue;
        }

        // 索引
        const std::size_t idx_x0  = state_index(k,     0);
        const std::size_t idx_y0  = state_index(k,     1);
        const std::size_t idx_vx0 = state_index(k,     2);
        const std::size_t idx_vy0 = state_index(k,     3);

        const std::size_t idx_x1  = state_index(k + 1, 0);
        const std::size_t idx_y1  = state_index(k + 1, 1);
        const std::size_t idx_vx1 = state_index(k + 1, 2);
        const std::size_t idx_vy1 = state_index(k + 1, 3);

        // 2.1 位置过程因子：p_{k+1} - p_k ≈ 0.5 (v_k + v_{k+1}) dt
        //
        // 对 x 分量：
        //   r = x1 - x0 - 0.5*(vx0 + vx1)*dt
        //     = (-1)*x0 + (1)*x1 + (-0.5*dt)*vx0 + (-0.5*dt)*vx1
        //
        {
            std::size_t idxs[4] = { idx_x0, idx_x1, idx_vx0, idx_vx1 };
            double      a[4]    = { -1.0,   1.0,    -0.5 * dt_s, -0.5 * dt_s };
            const double w = 1.0 / (sigma_pos * sigma_pos);

            add_factor_row(H, g, idxs, a, 4, 0.0, w);
        }

        // 对 y 分量：
        {
            std::size_t idxs[4] = { idx_y0, idx_y1, idx_vy0, idx_vy1 };
            double      a[4]    = { -1.0,   1.0,    -0.5 * dt_s, -0.5 * dt_s };
            const double w = 1.0 / (sigma_pos * sigma_pos);

            add_factor_row(H, g, idxs, a, 4, 0.0, w);
        }

        // 2.2 速度平滑因子：惩罚 (v_{k+1} - v_k)，
        //     等价于假设加速度为白噪声，std ≈ sigma_acc。
        //
        // 形式：
        //   r_vx = (vx1 - vx0)  ~ N(0, (sigma_acc * dt)^2)
        //   权重 w = 1 / (sigma_acc^2 * dt^2)
        const double sigma_v = sigma_acc * dt_s;
        const double w_v     = 1.0 / (sigma_v * sigma_v);

        {
            // vx
            std::size_t idxs[2] = { idx_vx0, idx_vx1 };
            double      a[2]    = { -1.0,     1.0    };
            add_factor_row(H, g, idxs, a, 2, 0.0, w_v);
        }
        {
            // vy
            std::size_t idxs[2] = { idx_vy0, idx_vy1 };
            double      a[2]    = { -1.0,     1.0    };
            add_factor_row(H, g, idxs, a, 2, 0.0, w_v);
        }
    }

    // -------- 3) DVL 速度观测因子 --------

    const double sigma_dvl = std::max(cfg_.dvl_vel_sigma_xy, 1e-6);
    const double w_dvl     = 1.0 / (sigma_dvl * sigma_dvl);

    for (std::size_t k = 0; k < N; ++k) {
        const auto& s = samples_[k];
        if (!s.has_dvl_vel) {
            continue;
        }

        const std::size_t idx_vx = state_index(k, 2);
        const std::size_t idx_vy = state_index(k, 3);

        {
            // vx_k ≈ dvl_vel_x
            std::size_t idxs[1] = { idx_vx };
            double      a[1]    = { 1.0    };
            add_factor_row(H, g, idxs, a, 1, s.dvl_vel_x, w_dvl);
        }
        {
            // vy_k ≈ dvl_vel_y
            std::size_t idxs[1] = { idx_vy };
            double      a[1]    = { 1.0    };
            add_factor_row(H, g, idxs, a, 1, s.dvl_vel_y, w_dvl);
        }
    }

    // -------- 4) ZUPT 因子（零速软约束） --------
    //
    // 简单策略：对 zupt_hint=true 的节点施加 vx≈0, vy≈0，
    // 权重比 DVL 更强一些，例如 sigma_zupt = 0.5 * dvl_sigma。

    double sigma_zupt = sigma_dvl * 0.5;
    if (sigma_zupt <= 0.0) {
        sigma_zupt = sigma_dvl;
    }
    const double w_zupt = 1.0 / (sigma_zupt * sigma_zupt);

    for (std::size_t k = 0; k < N; ++k) {
        const auto& s = samples_[k];
        if (!s.zupt_hint) {
            continue;
        }

        const std::size_t idx_vx = state_index(k, 2);
        const std::size_t idx_vy = state_index(k, 3);

        {
            std::size_t idxs[1] = { idx_vx };
            double      a[1]    = { 1.0    };
            add_factor_row(H, g, idxs, a, 1, 0.0, w_zupt);
        }
        {
            std::size_t idxs[1] = { idx_vy };
            double      a[1]    = { 1.0    };
            add_factor_row(H, g, idxs, a, 1, 0.0, w_zupt);
        }
    }

    // -------- 5) 求解 H x = g --------

    std::vector<double> x(nvar, 0.0);
    if (!cholesky_solve(H, g, x)) {
        std::cerr << "[GraphSmoother2D] ERROR: Cholesky solve failed "
                  << "(matrix not SPD or numerical issue).\n";
        solved_ = false;
        states_.clear();
        return false;
    }

    // -------- 6) 解析解向量为 SmootherState2D 序列 --------

    states_.resize(N);
    for (std::size_t k = 0; k < N; ++k) {
        const std::size_t idx_x  = state_index(k, 0);
        const std::size_t idx_y  = state_index(k, 1);
        const std::size_t idx_vx = state_index(k, 2);
        const std::size_t idx_vy = state_index(k, 3);

        SmootherState2D st{};
        st.t_ns = samples_[k].t_ns;
        st.x    = x[idx_x];
        st.y    = x[idx_y];
        st.vx   = x[idx_vx];
        st.vy   = x[idx_vy];

        states_[k] = st;
    }

    solved_ = true;
    return true;
}

} // namespace nav_core::estimator
