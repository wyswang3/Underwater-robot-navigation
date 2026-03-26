// nav_core/include/nav_core/preprocess/imu_rt_preprocessor.hpp
//
// @file  imu_rt_preprocessor.hpp
// @brief IMU 实时预处理：静止窗零偏估计 + 传感器坐标系→体坐标系 + 扣重力 + 航向角速度滤波。
//
// 设计定位：
//   - 位于「底层 ImuDriver（原始报文解析）」与「导航估计器（ESKF / MHE）」之间；
//   - 只负责单 IMU 传感器信号质量提升与坐标系归一化：
//       * 启动阶段静止窗内估计测量零偏（bias）和随机游走水平；
//       * 按固定映射将“传感器坐标系”数据转换到“体坐标系 FRD”；
//       * 使用 roll/pitch 估计实时扣除重力，加速度输出为“线加速度”； 
//       * 对 yaw 轴陀螺做零偏估计与一阶低通滤波；
//       * roll/pitch 角度直接信任 IMU 的磁场/姿态解算结果；
//   - 不负责：ESKF、导航坐标 ENU 下的姿态/位置解算、ZUPT 等高层逻辑。
//
// 坐标系约定（与你当前工程假设一致）：
//   - 传感器坐标系（sensor frame, S）
//       * X_s: 向右
//       * Y_s: 向前
//       * Z_s: 向上
//     对应 WitMotion 的 RFU 约定。
//   - 体坐标系（body frame, B = FRD）
//       * X_b: 向前 (Forward)
//       * Y_b: 向右 (Right)
//       * Z_b: 向下 (Down)
//     固定线性变换（v_b = R_sb * v_s）：
//       X_b =  Y_s
//       Y_b =  X_s
//       Z_b = -Z_s
//     即：
//       R_sb = [[0, 1, 0],
//               [1, 0, 0],
//               [0, 0,-1]]
//
//   - 导航坐标系：ENU（East, North, Up）
//       * X_n: East
//       * Y_n: North
//       * Z_n: Up
//       * 本模块不直接在 ENU 下运算，只保证体坐标系输出与 ENU 约定兼容，
//         供上层 ESKF 在 ENU 下做融合（例如将 body 加速度 / 角速度投影到 ENU）。
//

#pragma once

#include <cstdint>

#include "nav_core/core/types.hpp"   // MonoTimeNs / ImuSample / ImuFilterConfig

namespace nav_core::preprocess {

// ============================ 1. 配置与状态 ============================

/**
 * @brief IMU 实时预处理配置。
 *
 * basic         : 复用 nav_core::ImuFilterConfig，对尖刺 / NaN / 超量程做硬性过滤；
 * static_window : 启动阶段用于估计零偏的“静止窗”相关参数；
 * gravity       : 扣重力相关参数与参考 g；
 * yaw_filter    : 航向角速度（yaw 轴陀螺）的一阶低通与 deadband；
 * deadband      : 对很小的角速度 / 加速度做硬截断，抑制随机游走。
 *
 * 说明：
 *   - “静止窗”是在启动期自动检测机器人静止状态，从而估计： 
 *       * 传感器坐标系下的陀螺零偏（尤其是 yaw 轴）； 
 *       * 加速度的静态偏置与随机游走水平（用于诊断）； 
 *   - 坐标系转换与扣重力是在“体坐标系”下进行：
 *       * 输入 IMU 报文中包含 roll/pitch（可视为已做磁场/姿态融合）； 
 *       * process() 内部使用 roll/pitch 把重力从测量中扣除，输出线加速度。
 */
struct ImuRtPreprocessConfig {
    // ---- 1) 基本有效性检查（来自 types.hpp）----
    ImuFilterConfig basic{};   ///< 尖刺 / NaN / 过大数值过滤配置

    // ---- 2) 静止窗 bias 估计参数 ----
    //
    // static_window_s:
    //   - 上电后，若 |gyro| 和 |accel| 都小于静止阈值，则认为处于静止窗内；
    //   - 当连续静止时间累计 >= static_window_s 时，估计 gyro_bias / accel_bias；
    //   - 估计完成后 bias_ready=true，后续样本将减去该 bias。
    //
    double static_window_s          = 20.0;  ///< 静止窗持续时间 [s]（可根据实际调）
    float  static_max_abs_gyro      = 0.01f; ///< 静止判定阈值：|ω| < 0.01 rad/s
    float  static_max_abs_accel_dev = 0.05f; ///< 静止判定阈值：| |a| - g_ref | < 0.05 m/s^2

    // “静止窗内随机游走”评估的最大样本数（用于统计 std）
    std::uint64_t static_max_samples = 10000;

    // ---- 3) 重力参考 ----
    //
    // g_ref_m_s2:
    //   - 作为静止时加速度模长参考（重力），以及扣重力时的 g 大小；
    //
    float g_ref_m_s2 = 9.78f; ///< 实验场地重力近似值 [m/s^2]

    bool enable_gravity_compensation = true; ///< 是否在体坐标系下实时扣重力

    // ---- 4) 航向角速度（yaw 轴）滤波 ----
    //
    // yaw_lpf_cutoff_hz:
    //   - 若 <= 0，则不对 yaw 轴陀螺做低通，只减 bias + deadband；
    //
    float yaw_lpf_cutoff_hz  = 2.0f;   ///< yaw 轴 gyroscope 一阶低通截止频率 [Hz]
    float yaw_deadband_rad_s = 0.001f; ///< yaw 轴 deadband

    // ---- 5) 全轴 deadband（随机游走截断）----
    //
    float gyro_deadband_rad_s = 0.002f; ///< 所有轴统一 deadband（若 < yaw_deadband，会对 yaw 采用更严格的阈值）
    float accel_deadband_m_s2 = 0.01f;  ///< 加速度 deadband

    // ---- 6) 姿态来源开关 ----
    //
    // use_imu_rp_from_sample:
    //   - true 时，假定 ImuSample 中已经包含 roll/pitch（IMU 自己输出的姿态），
    //     本模块直接信任并用于扣重力；
    //   - false 时，视为无可靠 roll/pitch，不做扣重力，仅完成 bias 去除与
    //     坐标系转换，重力留给上层 ESKF 处理。
    bool use_imu_rp_from_sample = true;
};

/**
 * @brief ImuRtPreprocessor 的内部状态快照（用于诊断 / 调试）。
 *
 * 注意：
 *   - 该结构主要用于只读观察，不推荐上层直接修改；
 *   - 其中的“sensor_*”是传感器坐标系下的统计，“body_*”是 FRD 下的结果。
 */
struct ImuRtPreprocessState {
    MonoTimeNs last_mono_ns{0}; ///< 最近一次处理样本的时间戳（ns）

    // ---- 静止窗 / bias 相关 ----
    bool  bias_ready{false};       ///< 静止窗 bias 是否已经估计完成
    bool  currently_static{false}; ///< 最近一次 process 判定是否“静止”

    // 传感器坐标系下估计出的零偏（原始测量系）
    float gyro_bias_sensor[3]{0.f, 0.f, 0.f};   ///< [bx_s, by_s, bz_s] (rad/s)
    float accel_bias_sensor[3]{0.f, 0.f, 0.f};  ///< [bx_s, by_s, bz_s] (m/s^2)

    // 体坐标系下的零偏（转换后，用于导航层 debug）
    float gyro_bias_body[3]{0.f, 0.f, 0.f};     ///< [bx_b, by_b, bz_b]
    float accel_bias_body[3]{0.f, 0.f, 0.f};    ///< [bx_b, by_b, bz_b]

    // 静止窗统计（用于估计随机游走等指标）
    double        static_window_acc_s{0.0};     ///< 当前连续静止累计时长 [s]
    std::uint64_t static_window_samples{0};     ///< 静止窗累计样本数
    float         gyro_std_sensor[3]{0.f, 0.f, 0.f};  ///< 静止窗内陀螺 std 估计
    float         accel_std_sensor[3]{0.f, 0.f, 0.f}; ///< 静止窗内加速度 std 估计

    // ---- yaw 轴滤波内部状态 ----
    //
    // yaw 轴定义（语义说明）：
    //   - 体坐标系 FRD 中几何 z 轴为 Z_b（向下）； 
    //   - 数学上“绕世界 Up 轴（ENU +Z_n）逆时针”为正的 yaw_rate，
    //     与 FRD 的 Z_b 有一个符号关系：
    //        yaw_rate_about_Up ≈ -ω_z_b
    //   - 本模块内部会在 body 语义下选择合适的符号约定并进行滤波，
    //     上层只需要把输出当作“body 语义的 yaw 相关角速度分量”使用。
    float yaw_lpf_state_body{0.f};  ///< 一阶低通输出状态（body yaw 相关角速度）
};


// ============================ 2. 实时预处理类 ============================

/**
 * @brief IMU 实时预处理器
 *
 * 典型使用流程（C++ 导航守护进程中）： 
 *
 *   nav_core::preprocess::ImuRtPreprocessor::ImuRtPreprocessor(const ImuRtPreprocessConfig& cfg);
 *
 *   ImuRtPreprocessor imu_pp(cfg);
 *   ImuSample raw, proc;
 *   while (driver.read(raw)) {
 *       if (imu_pp.process(raw, proc)) {
 *           // proc 内：
 *           //   - ang_vel 已转换到 body=FRD 且减去零偏 + 已做 deadband / yaw 低通；
 *           //   - lin_acc 已转换到 body=FRD 且扣除了重力（若 enable_gravity_compensation）； 
 *           //   - roll/pitch 直接来自 IMU 报文（信任），yaw 可由上层决定如何使用 / 校正； 
 *           feed_to_eskf(proc);
 *       }
 *   }
 *
 * 设计约束：
 *   - 不改写时间戳：输出 ImuSample.out.mono_ns / est_ns 与输入保持一致；
 *   - 不在此处做全局（ENU）姿态/位置解算，ESKF/MHE 在更高层实现；
 *   - 非线程安全：默认按“单生产者”模型使用，若多线程访问请上层加锁。
 */
class ImuRtPreprocessor {
public:
    /// @brief 使用给定配置构造预处理器（不会自动 reset）。
    explicit ImuRtPreprocessor(const ImuRtPreprocessConfig& cfg = ImuRtPreprocessConfig{});

    /// @brief 重置内部状态（丢弃已有 bias / 统计 / yaw 滤波状态）。
    void reset();

    /// @brief 设置新的预处理配置（不自动 reset，配置即时生效）。
    void setConfig(const ImuRtPreprocessConfig& cfg);

    /// @brief 获取当前配置（按值返回）。
    [[nodiscard]] ImuRtPreprocessConfig config() const;

    /// @brief 获取当前内部状态快照（只读观察）。
    [[nodiscard]] ImuRtPreprocessState state() const;

    /// @brief 当前是否已经完成静止窗 bias 估计。
    [[nodiscard]] bool biasReady() const noexcept;

    /// @brief 最近一次样本是否被认为“处于静止状态”。
    [[nodiscard]] bool currentlyStatic() const noexcept;

    /**
     * @brief 处理一帧原始 IMU 样本，输出预处理后的样本。
     *
     * 预期实现步骤（在 .cpp 中完成）：
     *
     *   1) 基本有效性检查（basic）：
     *      - 若 in 中存在 NaN / inf / 绝对值超量程，则直接返回 false；
     *
     *   2) 静止窗检测与 bias 估计：
     *      - 基于“传感器坐标系”的原始 gyro/acc 测量，
     *        判断是否满足静止阈值（|gyro|、| |acc| - g_ref |）； 
     *      - 在连续静止时间累计 >= static_window_s 之前，
     *        不对外输出 bias_ready=true； 
     *      - 静止窗满后计算 gyro_bias_sensor / accel_bias_sensor 及随机游走 std；
     *
     *   3) 传感器坐标系 → 体坐标系：
     *      - 按固定映射：
     *           X_b =  Y_s；
     *           Y_b =  X_s；
     *           Z_b = -Z_s；
     *        将陀螺和加速度转换到 body=FRD；
     *
     *   4) 去零偏：
     *      - 若 bias_ready=true，则从 body 轴量测中减去对应 bias；
     *
     *   5) 扣重力（可选）：
     *      - 若 cfg.enable_gravity_compensation && cfg.use_imu_rp_from_sample：
     *           * 从 in 中读取 roll/pitch（IMU 内部解算的姿态）； 
     *           * 在体坐标系下构造重力向量 g_b； 
     *           * 从线加速度测量中减去 g_b，得到“线加速度 a_lin_b”； 
     *
     *   6) 航向角速度滤波：
     *      - 选出 body yaw 相关角速度分量（与“绕 Up 轴”数学定义一致的符号），
     *        在减 bias 后对其应用一阶低通（cutoff = yaw_lpf_cutoff_hz）和 yaw_deadband； 
     *      - 其他轴只做 deadband（由 gyro_deadband_rad_s 控制）； 
     *
     *   7) 填充输出 ImuSample：
     *      - 时间戳 / 序号沿用输入；
     *      - ang_vel / lin_acc 写入“体坐标系 + 去 bias + 扣重力（可选）”后的结果；
     *      - roll/pitch 原样从输入拷贝（信任 IMU 姿态），yaw 是否拷贝由实现决定；
     *      - 若成功生成有效输出样本，则返回 true；否则返回 false。
     *
     * @param in   原始 IMU 样本（来自 ImuDriver，仍在传感器坐标系）
     * @param out  输出：预处理后的 IMU 样本（体坐标系 / 去偏 / 扣重力 / yaw 滤波）
     * @return true    成功生成有效输出样本
     * @return false   输入无效 / basic 过滤丢弃
     */
    bool process(const ImuSample& in, ImuSample& out);

private:
    ImuRtPreprocessConfig cfg_{};
    ImuRtPreprocessState  st_{};

    // 实现细节会在 .cpp 中完成，包括：
    //   - 静止窗累积与 bias / std 估计；
    //   - 一阶低通滤波系数计算（基于 dt 与 cutoff_hz）；
    //   - 传感器坐标系到体坐标系的固定线性变换；
    //   - 基于 roll/pitch 的重力向量构造与扣除；
    //   - yaw 相关角速度的提取与符号约定。
};

} // namespace nav_core::preprocess
