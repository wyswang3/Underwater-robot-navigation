// nav_core/include/nav_core/estimator/graph_smoother_2d.hpp
//
// @file  graph_smoother_2d.hpp
// @brief 2D 因子图平滑器：离线 / 准在线的 [p, v] 平面轨迹二次型最小二乘。
// 
// 设计定位：
//   - 以“统一时间轴上的 DVL 速度观测 +（可选）ZUPT 提示”为输入；
//   - 在 x-y 平面上构建状态 [p, v] = [x, y, vx, vy] 的因子图：
//       * 过程因子：相邻结点之间的运动学约束（梯形积分 + 平滑项）；
//       * 观测因子：DVL ENU 速度观测 v_dvl ~ v_state + 噪声；
//       * 先验因子：首/末结点的软锚点（位置/速度先验）；
//   - 通过一次二次型最小二乘（线性方程组）求得全局最优平滑轨迹；
//   - 输出平滑后的节点序列，用于：
//       * 和在线导航 NavState 做轨迹对比（误差/RMS）；
//       * 作为离线“真值”近似，用来调参、验收导航精度。
// 
// 非目标：
//   - 不做完整 3D 姿态/深度估计，仅在 x-y 平面上工作；
//   - 不依赖线程 / IO / 日志，由上层 Python/C++ 工具喂入数据；
//   - 不暴露任何矩阵库（Eigen 等），内部实现可自由选择。
// 

#pragma once

#include <cstdint>
#include <vector>

#include "nav_core/core/types.hpp"    // MonoTimeNs / Vec3d（若需要）
// 若需要和 online_estimator 共用 math 工具，可在 .cpp 中包含 math.hpp

namespace nav_core::estimator {

// ============================ 1. 配置结构 ============================

/**
 * @brief 2D 因子图平滑器配置。
 *
 * 说明：
 *   - 整个平滑问题是一个加权二次型最小二乘：min_x ||A x - b||^2_W；
///   - 本配置通过“标准差 / 方差”等参数间接控制各因子权重；
///   - 你可以在 Python 侧调参，然后固化为 YAML → C++ 解析为本结构。
 */
struct GraphSmoother2DConfig {
    // ---- 观测噪声（DVL 速度）----

    /// DVL 速度观测的标准差（m/s），假设 x,y 同方差、独立。
    /// 对每个速度观测因子： cost ~= || v_xy - v_dvl_xy ||^2 / sigma^2。
    double dvl_vel_sigma_xy = 0.05;  // 经验值，可根据水池数据调整

    // ---- 过程模型噪声 / 平滑项 ----

    /// “过程模型”中的加速度白噪声标准差（m/s^2）。
    /// 可理解为：我们希望 v_{k+1} ~ v_k + a*dt，a 的方差 ~ sigma_acc^2。
    /// 在系数矩阵里会体现为对 (v_{k+1} - v_k) 的二次惩罚。
    double process_acc_sigma = 0.2;

    /// 位置过程噪声的标准差（m），对应“梯形积分”残差的权重。
    /// 典型约束：p_{k+1} - p_k ≈ 0.5 * (v_k + v_{k+1}) * dt。
    double process_pos_sigma = 0.05;

    // ---- 先验锚点（soft anchor）----

    /// 是否对第一个结点施加位置先验（例如来自在线导航初值）。
    bool use_prior_pos0 = true;

    /// 第一个结点位置先验的标准差（m）。
    double prior_pos0_sigma = 0.1;

    /// 是否对第一个结点速度施加先验（通常期望为 0）。
    bool use_prior_vel0 = true;

    /// 第一个结点速度先验的标准差（m/s）。
    double prior_vel0_sigma = 0.05;

    /// 是否对最后一个结点位置施加柔性锚点（可选）。
    bool use_prior_posN = false;

    /// 最后一个结点位置先验的标准差（m）。
    double prior_posN_sigma = 0.2;

    // ---- 时间轴与采样 ----

    /// 相邻结点允许的最大时间间隔（秒）。
    /// 若 dt 过大，可以在 .cpp 中选择“拆成多段”或降低过程因子权重。
    double max_knot_dt_s = 2.0;

    /// 是否对时间序列做“自动重采样”（当前版本可以忽略，预留扩展）。
    bool enable_resample = false;
};


// ============================ 2. 输入 / 输出数据结构 ============================

/**
 * @brief 2D 因子图平滑器输入样本（按时间序列排列）。
 *
 * 典型构造方式：
 *   - 从离线日志中读取 DVL + IMU 对齐后的时间轴；
 *   - 以固定 dt 或 DVL 采样时间为“结点时间 t_k”；
 *   - 对每个 t_k，记录：
///       * DVL ENU 速度（若该时刻附近有有效 DVL 观测）；
///       * ZUPT 提示（来自 DVL |v| 很小 或 IMU ZUPT 判定）；
///   - 然后按时间顺序依次调用 addSample()。
 *
 * 说明：
///   - 本结构仅做“速度观测 + 零速提示”，不关心 z/深度；
///   - 如果你希望加入“在线 NavState 作为软观测”，可以在未来扩展一个
 *     额外字段，比如 nav_vel_hint/nav_pos_hint 及其权重。
 */
struct SmootherInputSample2D {
    MonoTimeNs t_ns{0};      ///< 单调时间戳（ns），严格递增。

    // ---- DVL 速度观测（ENU 平面）----
    //
    // 约定：使用 ENU 速度的 x,y 分量：
    //   v_enu.x = vE, v_enu.y = vN，z 分量在本 2D 平滑器中忽略；
    // 如果 has_dvl_vel == false，则本样本不添加速度观测因子。
    bool  has_dvl_vel{false};
    double dvl_vel_x{0.0};   ///< ENU East 方向速度 (m/s)
    double dvl_vel_y{0.0};   ///< ENU North 方向速度 (m/s)

    // ---- ZUPT 提示（零速约束的候选）----
    //
    // 若 zupt_hint == true，则在本节点上可以附加一个“速度接近 0”的因子，
    // 权重可在 .cpp 中基于 dvl_vel_sigma_xy 与经验系数设定。
    bool  zupt_hint{false};
};

/**
 * @brief 平滑后的 2D 状态（结点）。
 *
 * 状态向量定义：
 *   - [x, y, vx, vy]，均在 ENU 平面坐标系下：
 *       * x: East，m
 *       * y: North，m
 *       * vx: East 方向速度，m/s
 *       * vy: North 方向速度，m/s
 */
struct SmootherState2D {
    MonoTimeNs t_ns{0};  ///< 对应的时间戳（与输入 sample 一一对应）

    double x{0.0};       ///< ENU East 位置 (m)
    double y{0.0};       ///< ENU North 位置 (m)
    double vx{0.0};      ///< ENU East 速度 (m/s)
    double vy{0.0};      ///< ENU North 速度 (m/s)
};


// ============================ 3. 主类接口 ============================

/**
 * @brief 2D 因子图平滑器（离线 / 准在线）。
 *
 * 使用示例（离线批处理）：
 *
 *   GraphSmoother2DConfig cfg;
 *   cfg.dvl_vel_sigma_xy = 0.05;
 *   // ... 其他参数 ...
 *
 *   GraphSmoother2D smoother(cfg);
 *
 *   std::vector<SmootherInputSample2D> samples = load_samples_from_log(...);
 *   smoother.setSamples(samples);
 *
 *   if (!smoother.solve()) {
 *       // 处理求解失败（数值问题 / 数据过少等）
 *   }
 *
 *   const auto& states = smoother.states();
 *   // 与 NavState 日志做轨迹对比 / RMS 计算 / 可视化
 */
class GraphSmoother2D {
public:
    explicit GraphSmoother2D(const GraphSmoother2DConfig& cfg = GraphSmoother2DConfig{});

    /// 更新配置（不自动重新求解）。
    void setConfig(const GraphSmoother2DConfig& cfg) { cfg_ = cfg; }
    [[nodiscard]] GraphSmoother2DConfig config() const { return cfg_; }

    /// 清空内部样本与结果，恢复到“未构建图”的状态。
    void clear();

    /// 以整体形式设置样本序列（内部会做一份拷贝）。
    ///
    /// 注意：
    ///   - 要求 samples 按 t_ns 严格递增；
    ///   - 该函数不会自动调用 solve()。
    void setSamples(const std::vector<SmootherInputSample2D>& samples);

    /// 追加单个样本（用于流式构建）。不会自动求解。
    void addSample(const SmootherInputSample2D& sample);

    /// 返回当前已有的输入样本（按时间顺序）。
    [[nodiscard]] const std::vector<SmootherInputSample2D>& samples() const {
        return samples_;
    }

    /**
     * @brief 构建因子图并执行一次全局平滑求解。
     *
     * 行为（在 .cpp 中实现的典型流程）：
     *   1) 检查样本数量，若少于 2 个则直接返回 false；
     *   2) 根据样本时间轴构建结点序列（每个样本对应一个 [x, y, vx, vy] 状态）； 
     *   3) 添加各类二次因子：
     *      - 先验因子（首/末结点位置/速度）；
     *      - 过程因子（位置梯形积分 + 速度平滑项）；
     *      - DVL 速度观测因子；
     *      - ZUPT 约束因子（对标记 zupt_hint 的结点施加 v≈0 的软约束）；
     *   4) 组装线性方程（例如正规方程），调用内部线性求解器；
     *   5) 将解映射回 SmootherState2D 序列，保存在 states_ 中。
     *
     * @return true  求解成功且 states() 可用；
     * @return false 求解失败（数据不足 / 数值问题等）。
     */
    bool solve();

    /// 是否已经有一组有效的平滑结果（solve() 成功）。
    [[nodiscard]] bool hasSolution() const noexcept { return solved_; }

    /// 返回平滑后的状态序列。
    ///
    /// 注意：仅在 hasSolution() == true 时，states_ 内容才有意义；
    ///       若未调用 solve() 或 solve() 返回 false，states_ 可能为空。
    [[nodiscard]] const std::vector<SmootherState2D>& states() const {
        return states_;
    }

private:
    GraphSmoother2DConfig              cfg_{};
    std::vector<SmootherInputSample2D> samples_{};
    std::vector<SmootherState2D>       states_{};

    bool solved_{false};

    // 内部实现（矩阵构建 / 求解）放在 .cpp 中，可使用 Eigen / 手写 Cholesky 等。
    // 头文件不暴露任何矩阵库类型。
};

} // namespace nav_core::estimator
