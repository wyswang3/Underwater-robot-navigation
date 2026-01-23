#pragma once

// nav_core/include/nav_core/estimator/eskf.hpp
//
// @file  eskf.hpp
// @brief IMU + DVL 的误差状态卡尔曼滤波器（ESKF），用于在线水下导航。
// 
// 核心功能：
//   - 利用高频 IMU（加速度 + 陀螺）做惯导积分，连续估计 ENU 下的位姿 / 速度；
//   - 用低频 DVL（BI/BE）速度观测对水平速度 / 垂向速度进行约束，抑制 IMU 累计误差；
//   - 将陀螺零偏、加速度零偏视为“随机游走状态”，通过长期 DVL 约束进行估计；
//   - 支持“柔性 ZUPT”策略：在 DVL 水平速度接近 0 时，构造零速观测，强力压制随机游走；
//   - 适配你当前工程：IMU 已由 ImuRtPreprocessor 预处理到体坐标 FRD、已扣重力的线加速度；
//     DVL 预处理模块已经负责 BI/BE/BD 分离与基础门控，ESKF 使用标准化 DvlSample。
// 
// 数学结构（当前版本假设 3D-NAV + 15 维误差状态）：
//
//   名义状态（nominal state）x：
//     - p_enu  = [E, N, U]^T                      位置（ENU）
//     - v_enu  = [vE, vN, vU]^T                   速度（ENU）
//     - q_nb   = nav←body 的单位四元数            姿态（body=FRD → nav=ENU）
//     - b_a    = [bax, bay, baz]^T                加速度零偏（body）
//     - b_g    = [bgx, bgy, bgz]^T                陀螺零偏（body）
//
//   误差状态（error state）δx（15x1）：
//     - δp     = [δE, δN, δU]^T
//     - δv     = [δvE, δvN, δvU]^T
//     - δθ     = [δθx, δθy, δθz]^T                小角度误差（body）
//     - δb_a   = [δbax, δbay, δbaz]^T
//     - δb_g   = [δbgx, δbgy, δbgz]^T
//
//   协方差：P（15x15）。
//
//   过程模型：
//     - 连续时间：
//         ṗ_enu = v_enu
//         v̇_enu = R_nb * (a_meas_b - b_a - n_a) + g_enu
//         q̇_nb  = 0.5 * Ω(ω_meas_b - b_g - n_g) * q_nb
//         ḃ_a   = n_ba_rw
//         ḃ_g   = n_bg_rw
//       其中：R_nb 由 q_nb 转换，g_enu = [0, 0, -g]^T（ENU 中向下）；
//            n_a, n_g 为 IMU 噪声；n_ba_rw, n_bg_rw 为零偏随机游走。
// 
//     - 离散时间（由 IMU 采样驱动）：
//         基于上一时刻状态 x_k，使用一阶或中值积分近似得到 x_{k+1}；
//         同时构造线性化矩阵 F_k, G_k，离散为状态转移 Φ_k 与过程噪声 Q_k：
//           δx_{k+1} = Φ_k * δx_k + w_k,   E[w_k w_k^T] = Q_k。
// 
//   观测模型（根据不同观测类型，有不同 H、R）：
//     - 水平 DVL 速度（BI/BE，经预处理转成 ENU）：
//         z_dvl_xy = [vE, vN]^T
//         h(x)     = [vE, vN]^T，从名义状态直接取 v_enu 的前两维；
//         线性化后：δz ≈ H_xy * δx + n_z,  H_xy 为 2x15 矩阵，
//           对应 δvE/δvN 的列为 1，其它列根据模型填 0。
// 
//     - 垂向 DVL 速度（来自 BE 的 vU，与 IMU 模型配合）：
//         z_dvl_z  = vU
//         h(x)     = vU
//         H_z      = [0 0 1  0 0 0  0...0]（仅对 δvU 敏感）。
// 
//     - 柔性 ZUPT：当 DVL 测得水平速度的模长 |v| 很小（< zupt_speed_mps）时，
//         构造零速观测：z = [0, 0]^T，而不是使用 noisy 的 v_meas；
//         通过 Kalman 更新把 vE, vN、姿态和陀螺零偏中的长期漂移压回接近 0。
// 
//   Kalman 更新（统一流程）：
//     - 给定观测 z，预测值 ẑ = h(x)，构造创新 r=z-ẑ；
//     - 构造 H、观测噪声协方差 R；
//     - 计算 S = H P H^T + R，利用 NIS = r^T S^{-1} r 做门控 / 动态膨胀；
//     - 若通过门控，求 K = P H^T S^{-1}，得到 δx = K r；
//     - 名义状态注入：
//         p_enu  ← p_enu  + δp
//         v_enu  ← v_enu  + δv
//         q_nb   ← δq ⊗ q_nb，δq ≈ [1, 0.5*δθ]^T 后归一化
//         b_a    ← b_a    + δb_a
//         b_g    ← b_g    + δb_g
//       然后把 δx 清零，并使用 Joseph 形式更新 P 保持对称正定性。
// 
// 工程约定：
//   - 坐标系：
//       * 体坐标系：body=FRD（X 前、Y 右、Z 下）；
//       * 导航坐标系：ENU（East, North, Up），重力 g 沿 U 负方向；
//       * IMU 实时预处理模块已经将原始传感器坐标系转换到 body=FRD，
//         并扣除重力，输出“线加速度 a_lin_b”；
//       * EskfFilter 的 propagate_imu() 默认认为 lin_acc 为“线加速度”，
//         若 cfg.imu_acc_is_linear=false，则内部会补偿 g。
//   - 当前阶段：尚未部署专门深度传感器，因此垂向方向主要依赖：
//       * IMU 垂向加速度积分（随机游走风险较大）；
//       * DVL BE 输出的 vU（垂向速度）与高度/累计位移，作为 z/vz 观测；
//       * 后续接入深度传感器后，可新增深度观测 update_depth()。
// 

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

#include "nav_core/core/types.hpp"  // Vec3f / Vec3d / ImuSample / DvlSample
#include "nav_core/core/math.hpp"   // wrap_angle, Kahan 等工具

namespace nav_core::estimator {

// ============================ 1. 配置与诊断类型 ============================

/**
 * @brief ESKF 配置参数（IMU 噪声 + 过程噪声 + DVL 观测噪声 + 门控）。
 *
 * 说明：
 *   - 原则：cfg 只存“物理意义明确”的标量，实际构造 F/Q/H/R 的细节在 .cpp 中； 
 *   - dt_min/dt_max 用于拒绝异常大的时间间隔（如日志跳变、传感器堵塞）。
 */
struct EskfConfig {
    // ============================================================
    // 1) 时间与数值稳定性（来自 time 部分）
    // ============================================================
    double dt_max_s = 0.05;     ///< 最大 IMU 传播步长（s）
    double dt_min_s = 1.0e-4;   ///< 最小 dt（避免数值问题）

    bool   yaw_wrap = true;     ///< 是否将 yaw wrap 到 (-pi,pi]

    // ============================================================
    // 2) 初始化策略（来自 init 部分）
    // ============================================================

    // yaw 初值来源： "imu" | "config"
    std::string init_yaw_source{"imu"};

    // ENU 初始位置/速度
    double init_E_m    = 0.0;
    double init_N_m    = 0.0;
    double init_U_m    = 0.0;   ///< 3D 版增加 U，方便后续深度接入

    double init_vE_mps = 0.0;
    double init_vN_mps = 0.0;
    double init_vU_mps = 0.0;

    // 初始 yaw（当 init_yaw_source=="config" 时使用）
    double init_yaw_rad = 0.0;

    // 初始 gyro Z 偏置（2D 的 bgz），3D 版扩展为矢量，但这里先保留一个标量接口
    double init_bgz_rad_s = 0.0;

    // 初始协方差（离线 2D 配置）
    double P0_pos_m2       = 0.25;      ///< (0.5 m)^2
    double P0_vel_m2s2     = 0.36;      ///< (0.6 m/s)^2
    double P0_yaw_rad2     = 0.03046;   ///< (10 deg)^2
    double P0_bgz_rad2s2   = 0.0004;    ///< (0.02 rad/s)^2

    // 3D ESKF 额外对角项（可以沿用 2D 的量级）
    double P0_U_m2         = 0.25;
    double P0_vU_m2s2      = 0.36;
    double P0_att_rad2     = 0.03046;   ///< roll/pitch 小角度误差
    double P0_ba_m2s4      = 1.0e-4;    ///< accel bias
    double P0_bg_rad2s2    = 1.0e-6;    ///< gyro bias (XYZ)

    // ============================================================
    // 3) IMU 语义（来自 imu 部分）
    // ============================================================
    bool   imu_acc_is_linear = true;   ///< true: 已扣重力；false: raw+g
    double gravity_mps2      = 9.78;

    // yaw 语义修正：used_yaw = yaw_sign * yaw_meas + yaw_offset_rad
    double yaw_sign          = 1.0;
    double yaw_offset_rad    = 0.0;

    // ============================================================
    // 4) 过程噪声（来自 process_noise 部分）
    // ============================================================

    // 线加速度噪声（离散 std），对应 2D: sigma_acc_mps2
    double sigma_acc_mps2    = 0.06;

    // yaw 轴陀螺噪声（Z 轴），对应 2D: sigma_gyro_z_rad_s
    double sigma_gyro_z_rad_s = 0.008;

    // 3D ESKF 通用陀螺噪声，可以默认用同一量级
    double sigma_gyro_xyz_rad_s = 0.008;

    // gyro Z bias 随机游走（continuous, sqrt(s)）
    double sigma_bgz_rw      = 2.0e-5;

    // 3D bias RW（XYZ），默认沿用同一数量级
    double sigma_ba_rw_mps2_sqrt_s  = 3.0e-5;
    double sigma_bg_rw_rad_s_sqrt_s = 2.0e-5;

    // 额外速度过程噪声，用来承认“动力学模型很烂”
    double q_vel_extra_mps2  = 1.20;

    // ============================================================
    // 5) DVL 水平观测（来自 dvl 部分）
    //    重要约束：水平观测只能用 DVL_BI（体坐标速度），
    //    先旋转到 ENU，再进入 ESKF。
    // ============================================================

    // 水平速度观测噪声（m/s），对应 sigma_xy_mps
    double sigma_dvl_xy_mps  = 0.008;

    // ZUPT 配置
    double zupt_speed_mps      = 0.04;   ///< |v_meas| <= 该值视为 ZUPT
    double sigma_dvl_zupt_mps  = 0.06;   ///< ZUPT 模式下观测噪声

    // 最低速度门限，避免 ratio 在小速度下炸掉
    double meas_speed_eps_mps  = 0.07;

    // ============================================================
    // 6) 创新门控 & R 膨胀（来自 gating 部分）
    // ============================================================

    // NIS 门限
    double nis_soft            = 25.0;
    double nis_target          = 25.0;
    double nis_hard            = 120.0;

    // 速度比值门控 |v_pre| / |v_meas|
    double ratio_soft          = 2.5;
    double ratio_hard          = 10.0;

    // R 膨胀上限
    double r_inflate_max       = 5.0e3;

    // 膨胀之后若仍然 NIS>nis_hard 是否拒绝（2D 里是 false）
    bool   post_inflate_hard_reject = false;

    // 绝对强制拒绝（safety net）
    bool   reject_huge_residual = true;
    double nis_abs_hard         = 300.0;

    // ============================================================
    // 7) 传播稳定器（来自 stabilizer 部分）
    // ============================================================

    // 速度泄漏系数（1/s），防止纯积分把速度拉飞
    double vel_leak_1ps        = 0.35;

    // 水平速度硬限幅（对 |v_xy| 生效）
    double v_hard_max_mps      = 0.80;

    // 3D: 垂向速度可单独给一个上限（先复用）
    double vU_hard_max_mps     = 0.80;

    // ============================================================
    // 8) 数值抖动（来自 numerics 部分）
    // ============================================================

    // 加到 R 上的小抖动（提高条件数）
    double meas_jitter         = 1.0e-9;

    // 加到 S 上的小抖动
    double S_jitter            = 1.0e-9;

    // ============================================================
    // 9) DVL 垂向相关（3D 特有，BE vU）
    // ============================================================

    double sigma_dvl_z_mps     = 0.05;   ///< vU 垂向速度观测噪声（先拍一个量级）
    double max_abs_dvl_z_mps   = 2.0;    ///< |vU| 超过则拒绝

    // 是否启用 DVL vU 更新（暂时可以开着，后面如果不稳定再在 YAML 里关）
    bool   enable_dvl_z_update = true;
};


/**
 * @brief 水平 DVL 更新诊断信息（用于日志 / 调参）。
 */
struct EskfUpdateDiagDvlXY {
    bool   ok{false};             ///< 本次观测是否被实际用于更新
    bool   is_zupt{false};        ///< 是否走了 ZUPT 路径（z=[0,0]）
    double nis{0.0};              ///< 最终 NIS 值
    double nis0{0.0};             ///< 初始 NIS（膨胀前）
    double nis1{0.0};             ///< 膨胀后 NIS
    double speed_pred_h{0.0};     ///< 预测水平速度模长 |vhat_xy|
    double speed_meas_h{0.0};     ///< 测量水平速度模长 |z_raw|
    double ratio_pred_over_meas{0.0}; ///< 预测/测量 速度比
    double R_inflate{1.0};        ///< 本次观测对 R 的膨胀倍数
    double HPHt_over_R{0.0};      ///< trace(HPHt)/trace(R)，粗略能量比例
    double r[2]{0.0, 0.0};        ///< 创新（残差）[rE, rN]
    double S_diag[2]{0.0, 0.0};   ///< S 的对角线（创新方差）
    std::string note;             ///< 文本说明（比如 "USED_OK" / "REJECT_NIS" 等）
};

/**
 * @brief 垂向 DVL vU 更新诊断信息。
 */
struct EskfUpdateDiagDvlZ {
    bool   ok{false};
    double nis{0.0};
    double r{0.0};                ///< 垂向速度残差
    double S{0.0};                ///< 创新协方差标量
    std::string note;
};

/**
 * @brief Yaw 伪观测（例如来自 IMU 姿态解算）的诊断信息。
 */
struct EskfUpdateDiagYaw {
    bool   ok{false};
    double nis{0.0};
    double yaw_residual_rad{0.0};
    std::string note;
};


// ============================ 2. 状态结构体 ============================

/**
 * @brief ESKF 名义状态的只读快照（方便外部获取）。
 */
struct EskfNominalState {
    nav_core::Vec3d p_enu;    ///< 位置 [E, N, U]
    nav_core::Vec3d v_enu;    ///< 速度 [vE, vN, vU]

    // 姿态：nav←body
    nav_core::Vec3d rpy_nb_rad; ///< 仅用于可视化 / 日志（内部用四元数）
    nav_core::Vec3d ba_mps2;    ///< 加速度零偏（body）
    nav_core::Vec3d bg_rad_s;   ///< 陀螺零偏（body）
};


// ============================ 3. ESKF 主类接口 ============================

/**
 * @brief IMU + DVL 误差状态卡尔曼滤波器。
 *
 * 使用流程（导航守护进程 nav_daemon 中典型模式）：
 *
 *   nav_core::estimator::EskfConfig cfg = ...;  // 从 YAML 载入
 *   nav_core::estimator::EskfFilter eskf(cfg);
 *
 *   // 1) 初始化（可选：利用静止窗估计的姿态/零偏作为初值）
 *   EskfNominalState x0{};
 *   x0.p_enu = {0, 0, 0};
 *   x0.v_enu = {0, 0, 0};
 *   x0.rpy_nb_rad = {roll0, pitch0, yaw0};
 *   x0.ba_mps2 = {0,0,0};
 *   x0.bg_rad_s = {0,0,0};
 *   eskf.reset(x0);
 *
 *   // 2) 主循环：每次 IMU 新样本时进行 propagate_imu()
 *   ImuSample imu;
 *   DvlSample dvl;
 *   while (running) {
 *       if (new_imu_sample(imu)) {
 *           eskf.propagate_imu(imu);    // 利用 ImuSample 的时间戳 auto 计算 dt
 *       }
 *       if (new_dvl_sample(dvl)) {
 *           // 水平速度：可以选用 BE 或 BI+IMU yaw 转 ENU
 *           EskfUpdateDiagDvlXY diag_xy;
 *           eskf.update_dvl_xy(dvl, &diag_xy);
 *
 *           // 垂向速度（暂时用 BE 的 vU，未来可叠加深度传感器）
 *           EskfUpdateDiagDvlZ diag_z;
 *           eskf.update_dvl_z(dvl, &diag_z);
 *       }
 *
 *       const auto x = eskf.state();
 *       // 将 x.p_enu / x.v_enu / x.rpy_nb_rad 提供给控制器或日志
 *   }
 *
 * 设计要点：
 *   - propagate_imu() 不负责任何硬门控，假定 ImuSample 已由 ImuRtPreprocessor
 *     做过 NaN/尖刺过滤与坐标系统一；若时间戳异常，会返回 false 并不更新状态；
 *   - update_dvl_xy() 与 update_dvl_z() 内部包含 NIS / 速度比值门控与 R 膨胀，
///     确保 DVL 异常时不会把 ESKF 拉飞；
///   - “柔性 ZUPT”策略在 update_dvl_xy() 内实现，通过 diag_xy.is_zupt 反映；
///   - 当前版本默认使用 ENU 作为导航坐标系，并约定 body=FRD，与预处理模块一致。
 */
class EskfFilter {
public:
    /// @brief 使用给定配置构造 ESKF，并按 cfg 的 init_* 初始化名义状态。
    explicit EskfFilter(const EskfConfig& cfg);

    /// @brief 用新的初值重置 ESKF 状态与协方差。
    void reset(const EskfNominalState& x0);

    /// @brief 只重置协方差为初始 P0（保持名义状态不变），用于在线重置。
    void reset_covariance();

    /// @brief 返回当前配置（按值复制）。
    [[nodiscard]] EskfConfig config() const { return cfg_; }

    /// @brief 设置新的配置（即时生效，不自动 reset）。
    void set_config(const EskfConfig& cfg) { cfg_ = cfg; }

    /// @brief 获取名义状态的只读快照。
    [[nodiscard]] EskfNominalState state() const;

    /// @brief 获取内部 15x15 协方差矩阵（行优先展平，便于日志/调试）。
    ///
    /// 注意：仅用于只读或外部导出；不建议外部直接修改。
    [[nodiscard]] const double* covariance_data() const { return P_.data(); }
    [[nodiscard]] std::size_t covariance_size() const { return P_.size(); } // 应为 15*15

    /// @brief 返回最近一次 IMU 传播的时间（MonoTimeNs，若尚未传播则为 0）。
    [[nodiscard]] MonoTimeNs last_propagate_time() const noexcept { return last_mono_ns_; }

    // ---------------------------------------------------------------------
    // 3.1 IMU 传播
    // ---------------------------------------------------------------------

    /**
     * @brief 使用一帧 IMU 数据进行惯导传播。
     *
     * 算法步骤（在 .cpp 中实现）：
     *   1) 由 imu.mono_ns 与内部 last_mono_ns_ 计算 dt；
     *      - 若首次调用，则仅设置时间，不做传播，返回 false；
     *      - 若 dt 不在 [cfg_.dt_min_s, cfg_.dt_max_s] 内，则忽略该帧，返回 false；
     *   2) 使用当前名义状态 (p,v,q,ba,bg) 与 imu 测量 (ang_vel, lin_acc) 做
     *      惯导积分（例如一阶或中值法），得到新的名义状态；
     *   3) 构造连续时间线性化矩阵 F,G，并离散为 Φ,Q，更新 P：
     *         P ← Φ P Φ^T + Q；
     *   4) 归一化四元数，保持数值稳定性；
     *   5) 记录 last_mono_ns_。
     *
     * @return true  本帧 IMU 已被用于传播
     * @return false 本帧被忽略（时间异常或首次调用）
     */
    bool propagate_imu(const ImuSample& imu);

    // ---------------------------------------------------------------------
    // 3.2 DVL 更新：水平速度（BI/BE） + 柔性 ZUPT
    // ---------------------------------------------------------------------

    /**
     * @brief 使用 DVL 的水平速度观测更新 ENU 下的 vE/vN（仅使用 BI）。
     *
     * 数据来源约定（工程约束）：
     *   - 水平观测只允许使用 DVL_BI（Instrument frame / body-like）速度；
     *   - DvlSample 应该在“预处理阶段”已经完成：
     *       * BI 体速的解析与质量门控（A/V、FOM、高度等）；
     *       * 坐标系统一：把 DVL 探头坐标系旋转到当前体坐标 body=FRD；
     *       * 得到 vel_body_mps[0:2]，即 body 下的 [vx_b, vy_b]；
     *   - EskfFilter::update_dvl_xy() 内部只做一件事：
     *       * 取当前 ESKF 名义姿态 q_nb（nav←body）；
     *       * 将 body 速度 [vx_b, vy_b, vz_b] 旋转到 ENU 得 [vE_meas, vN_meas, vU_meas]；
     *       * 仅使用 [vE_meas, vN_meas] 作为 z 输入水平更新。
     *
     * 观测模型：
     *   - z = [vE_meas, vN_meas]^T    （由 BI 体速 + ESKF 姿态旋转得到）
     *   - h(x) = [vE, vN]^T           （从名义状态 v_enu 直接取前两维）
     *
     * ZUPT 策略：
     *   - 根据 |z_raw| 与 cfg_.zupt_speed_mps 判定是否进入“零速观测”模式：
     *       * 若 |z_raw| <= zupt_speed_mps，则视为 ZUPT：
     *           z = [0, 0]^T；
     *           使用 cfg_.sigma_dvl_zupt_mps 作为观测噪声；
     *           重点压制 vE/vN 随时间漂移，以及姿态 / bgz 中累积的随机游走；
     *       * 否则使用 z=z_raw，噪声 cfg_.sigma_dvl_xy_mps。
     *
     * 门控与 R 膨胀（与 Python Eskf2D 同源）：
     *   - 计算预测速度模长 speed_pred_h 和测量模长 speed_meas_h；
     *   - 基于 NIS0 与速度比值 |v_pre|/|v_meas| 进行分级处理：
     *       * 若明显离谱（超 nis_hard 或 ratio_hard），直接拒绝；
     *       * 若“有点不合适”，对 R 做自适应膨胀（不轻易拒绝），并重新计算 NIS；
     *       * 若膨胀后仍超 nis_abs_hard，按 cfg_ 决策是否强制拒绝。
     *
     * 注意：
     *   - 本函数内部不会直接访问 DVL 的 BE 速度，BE 的水平分量被视为“不可信”，
     *     仅保留 BE 的 vU（垂向速度）供 update_dvl_z() 使用；
     *   - 水平速度 update 和垂向 vU update 是两个独立通道，门控与噪声配置也不同。
     */
    bool update_dvl_xy(const DvlSample& dvl,
                       EskfUpdateDiagDvlXY* diag = nullptr);

    // ---------------------------------------------------------------------
    // 3.3 DVL 更新：垂向速度 vU（来自 BE）
    // ---------------------------------------------------------------------

    /**
     * @brief 使用 DVL BE 的 vU 垂向速度观测更新 vU。
     *
     * 观测模型：
     *   - z = vU_meas
     *   - h(x) = vU
     *
     * 内部逻辑：
     *   - 若 |vU_meas| > cfg_.max_abs_dvl_z_mps，则拒绝；
     *   - 构造 1x15 的 H_z，仅对 δvU 敏感；
     *   - 使用 cfg_.sigma_dvl_z_mps 作为观测噪声；
     *   - 计算 NIS，实现简单门控，执行标量 Kalman 更新。
     */
    bool update_dvl_z(const DvlSample& dvl,
                      EskfUpdateDiagDvlZ* diag = nullptr);

    // ---------------------------------------------------------------------
    // 3.4 Yaw 伪观测（可选）
    // ---------------------------------------------------------------------

    /**
     * @brief 使用外部提供的 yaw 伪观测（例如 IMU 自己的姿态解算 yaw）。
     *
     * 说明：
     *   - 当前工程阶段，你倾向“只把 IMU yaw 当作相对量”，不强行纠偏；
     *   - 因此该函数默认不在控制循环中频繁使用，只在需要时（例如
     *     某些段落有可靠参考方向）才调用；
     *   - 观测模型：z = yaw_meas，h(x) = yaw(x)，误差为 wrap(yaw_meas - yaw_state)。
     */
    bool update_yaw(double yaw_meas_rad,
                    double sigma_yaw_meas_rad,
                    EskfUpdateDiagYaw* diag = nullptr);

private:
    EskfConfig cfg_{};

    // 名义状态（内部表示：位置 + 速度 + 四元数 + 零偏）
    nav_core::Vec3d p_enu_{0.0, 0.0, 0.0};
    nav_core::Vec3d v_enu_{0.0, 0.0, 0.0};

    // 四元数形式的 nav←body 姿态 [w, x, y, z]
    std::array<double, 4> q_nb_{1.0, 0.0, 0.0, 0.0};

    nav_core::Vec3d ba_mps2_{0.0, 0.0, 0.0};
    nav_core::Vec3d bg_rad_s_{0.0, 0.0, 0.0};

    // 15x15 协方差，按行优先展平存储
    std::array<double, 15 * 15> P_{};

    MonoTimeNs last_mono_ns_{0}; ///< 最近一次成功 propagate_imu() 的时间戳

    // ---- 内部辅助函数（在 .cpp 中实现）----

    // 将内部四元数转换为 RPY（供 state() 导出）
    void quat_to_rpy(nav_core::Vec3d& rpy_out) const;

    // 四元数归一化
    void normalize_quat();

    // 按索引访问 P 的 (i,j)
    double& P(std::size_t i, std::size_t j) { return P_[i * 15 + j]; }
    double  P(std::size_t i, std::size_t j) const { return P_[i * 15 + j]; }

    // 对称化 P，消除数值误差
    void symmetrize_P();

    // 内部构造 / 应用 Kalman 更新（通用 2D/1D 版本可在 .cpp 中实现）
};

} // namespace nav_core::estimator
