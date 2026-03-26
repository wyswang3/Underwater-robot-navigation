// nav_core/include/nav_core/preprocess/dvl_rt_preprocessor.hpp
//
// @file  dvl_rt_preprocessor.hpp
// @brief DVL 实时预处理：门控（gating） + BI/BE/BS/BD 拆分 + 坐标系归一化。
// 
// 设计定位：
//   - 位于「底层 DvlDriver（串口报文解析）」与「导航估计器（ESKF / MHE）」之间；
//   - 只负责单 DVL 传感器信号质量提升与统一接口输出：
//       * 对 BI/BE 速度做相关性 / 量程 / 速度幅值等门控；
//       * 对 BS/BD 中的 bottom-lock / range / FOM 信息做质量评估；
//       * 将 BI 统一到 body(FRD) 速度，将 BE 统一到 ENU 速度；
//       * 提供“对 ESKF 可直接使用”的 DvlRtOutput 结构；
//   - 不负责：EKF/ESKF 观测更新矩阵的构造、因子图构建等高层逻辑，这些在
//     estimator 层实现。
// 
// 报文语义（结合你当前 Hover H1000 流水）：
//   - BI: Bottom-track velocity, Instrument frame（仪器坐标系速度）
//   - BE: Bottom-track velocity, Earth frame（地球/导航坐标系速度）
//   - BS: Bottom-track status（锁底标志、相关系数、FOM 等）
//   - BD: Bottom-track distance / debug（各 beam 测距、海底高度等）
//
// 坐标系约定：
//   - 体坐标系 B: FRD (Forward, Right, Down)
//   - 导航坐标系 N: ENU (East, North, Up)
//   - DVL 仪器坐标系 I：由 DvlDriver 定义，这里通过配置给出 I→B 与 I→N 的关系。
// 
// 注：本头文件只依赖 nav_core/core/types.hpp 不直接依赖 Eigen，
//     旋转矩阵 / yaw-only 旋转可复用 nav_core/core/math.hpp 中工具。

#pragma once

#include <cstdint>

#include "nav_core/core/types.hpp"   // MonoTimeNs / Vec3f / Vec3d 等
#include "nav_core/core/math.hpp"    // norm / body_to_enu_yaw_only / apply_deadzone 等

namespace nav_core::preprocess {

// ============================ 1. DVL 原始采样接口约定 ============================
//
// 为了避免强绑定具体驱动实现，这里假定 DvlDriver 向上层提供一个
// nav_core::DvlRawSample（在 core/types.hpp 中定义），大致包含：
//   - 时间戳：mono_ns / est_ns
//   - 各报文分支：BI / BE / BS / BD 原始字段（如 vel_inst, vel_earth, range, corr, flags 等）
//   - mode / kind：当前帧属于 BI/BE/BS/BD 中的哪一种
//
// 预处理器只关心其中与“底速 / 质量评估”相关的字段：
//   - BI: 仪器坐标系速度 v_i = [vx, vy, vz] (m/s)
//   - BE: 地球坐标系速度 v_e = [vE, vN, vU] (m/s)
//   - BS/BD: bottom_lock 标志、beam 相关系数、测距、FOM 等。
//
// 若你的 core/types.hpp 中暂时没有 DvlRawSample，可以先按以下注释定义：
//
struct DvlRawSample {
      MonoTimeNs mono_ns{0};
      MonoTimeNs est_ns{0};
      std::uint8_t kind{0};     // 0=BI, 1=BE, 2=BS, 3=BD
      bool   bottom_lock{false};
      Vec3f  vel_inst_mps;      // BI: 仪器坐标系速度 (I-frame)
      Vec3f  vel_earth_mps;     // BE: Earth/ENU (?) 速度
      float  range_m[4];        // 各 beam 测距
      float  corr[4];           // 各 beam 相关系数
      float  fom;               // Figure-of-merit，如有
      std::uint32_t status;     // 厂家 status bitmask，如有
};

// 为避免循环依赖，本头文件只在接口上使用前置声明：



// ============================ 2. 预处理输出给导航层的结构 ============================

/**
 * @brief DVL 实时预处理输出（已经门控 + 坐标系统一）。
 *
 * 这是供 ESKF / MHE / 因子图使用的“干净接口”：
 *   - vel_be_enu: 底速在 ENU 下的速度（优先 BI+yaw 推出,垂向使用BE）；
 *   - vel_bi_body: 底速在体坐标系 FRD 下的速度（优先使用 BI）；
 *   - alt_bottom_m: DVL 底距（若可获得），可用于深度 / 地形约束；
 *   - bottom_lock: 是否判定为“有效锁底”；
 *   - gated_ok: 当前样本是否通过门控；
 *   - reason_code: 若 gated_ok=false，可选的门控失败标志，便于诊断。
 */
struct DvlRtOutput {
    MonoTimeNs mono_ns{0};
    MonoTimeNs est_ns{0};

    bool   has_be{false};
    Vec3d  vel_be_enu{0.0, 0.0, 0.0};   ///< ENU 速度（优先给 ESKF 用）

    bool   has_bi{false};
    Vec3d  vel_bi_body{0.0, 0.0, 0.0};  ///< body(FRD) 速度（可选，用于诊断 / 辅助算法）

    bool   has_alt{false};
    double alt_bottom_m{0.0};           ///< DVL 底距（若有），通常来自 BD/BS beams

    bool   bottom_lock{false};          ///< 预处理判定的锁底状态（综合 BS/BD）

    bool   gated_ok{false};             ///< 是否通过门控（只有 true 才建议用于 ESKF 更新）
    std::uint32_t reason_code{0};       ///< 门控失败原因 bitmask（0 表示 OK）

    // 可选：方便统计的质量指标汇总
    float mean_corr{0.0f};              ///< 有效 beams 的平均相关系数
    float min_range_m{0.0f};            ///< beams 中的最小测距（海底距离的粗略替代）
};


// ============================ 3. 配置与状态 ============================

/**
 * @brief DVL 实时预处理配置（门控 + 坐标系 + 噪声地板）。
 *
 * gate_*: 与“是否接受当前底速样本”相关的门控阈值；
 * frame  : 坐标系与 yaw 对齐策略；
 * noise  : DVL 速度噪声地板 / deadzone；
 */
struct DvlRtPreprocessConfig {
    // ---- 3.1 时间与基本开关 ----
    double max_gap_s           = 0.5;   ///< 若距离上一次有效样本时间间隔过长，可触发降级逻辑
    bool   enable_gating       = true;  ///< 总开关：关闭时仅做简单坐标系转换

    // ---- 3.2 门控阈值（基于物理与经验） ----
    //
    // 相关系数 / beams：
    float  min_corr            = 0.4f;  ///< beam 相关系数下限，低于该值视为不可靠
    std::uint8_t min_good_beams= 3;     ///< 至少要求多少个 beam 通过 min_corr 才认为有效

    // 测距 / 海底距离：
    float  min_alt_m           = 0.3f;  ///< DVL 与海底最小允许距离 [m]
    float  max_alt_m           = 30.0f; ///< DVL 与海底最大允许距离 [m]

    // 速度幅值：
    double max_abs_speed_mps   = 3.0;   ///< |v| 超过此值视为异常（超出 ROV 能力）
    double dvl_noise_floor_mps = nav_core::core::math::kDvlNoiseFloorMps; ///< 噪声地板

    // bottom-lock & status：
    bool   require_bottom_lock = true;  ///< 是否必须锁底才接受 BE/BI 作为底速
    std::uint32_t status_mask_ok = 0;   ///< 如有厂家 status bitmask，可在实现中使用

    // ---- 3.3 坐标系 / yaw 对齐 ----
    //
    // DVL 仪器坐标系 I → 体坐标系 B 的映射，简单起见先假定“固定 pitch/roll 装配”，
    //  yaw 对齐由导航层统一管理。
    //
    // mount_yaw_rad:
    //   - DVL 机体坐标系与 ROV 体坐标系之间的固定 yaw 偏差；
    //
    double mount_yaw_rad       = 0.0;   ///< I→B 的固定 yaw 角（rad）
    bool   use_be_as_enu       = true;  ///< 若 true，假定 BE vel_earth 已是 ENU；否则由 BI+yaw 推 ENU

    // 若 use_be_as_enu=false，则需要从 BI + 当前 nav yaw 计算 ENU 速度，
    // 本模块可以保留 BI 在 body 中，ESKF 内部再用 yaw 旋转到 ENU。
};


/**
 * @brief DVL 预处理内部状态，用于诊断 / 日志。
 */
struct DvlRtPreprocessState {
    MonoTimeNs last_in_mono_ns{0};   ///< 最近一次收到原始帧的时间
    MonoTimeNs last_ok_mono_ns{0};   ///< 最近一次通过门控的时间

    std::uint64_t total_samples{0};  ///< 收到的总样本数（所有 kind）
    std::uint64_t total_ok{0};       ///< 通过门控的样本数（用于统计接受率）
    std::uint64_t total_rejected{0}; ///< 被门控丢弃的样本数

    // 最近一次门控结果（方便上层打印 debug 信息）
    bool          last_gated_ok{false};
    std::uint32_t last_reason_code{0};

    // 最近一次输出（便于在不再更新时仍复用上一次底速）
    DvlRtOutput   last_output{};
};


// ============================ 4. 实时预处理类 ============================

/**
 * @brief DVL 实时预处理器。
 *
 * 典型使用流程（C++ nav_daemon 中）： 
 *
 *   DvlRtPreprocessConfig cfg;
 *   // 根据实验调参：cfg.min_corr / max_abs_speed_mps / dvl_noise_floor_mps / mount_yaw_rad 等
 *   DvlRtPreprocessor dvl_pp(cfg);
 *
 *   DvlRawSample raw;
 *   DvlRtOutput  out;
 *   while (dvl_driver.read(raw)) {
 *       if (dvl_pp.process(raw, out)) {
 *           // out.gated_ok == true 才推荐喂给 ESKF：
///           eskf.update_dvl(out);
 *       }
 *   }
 *
 * 注意：
 *   - process() 以“单帧”为单位，可以对 BI/BE/BS/BD 混流进行门控和拆分；
///   - 门控逻辑大致为：
///       * 读取 BS/BD 更新 bottom-lock / range / corr；
///       * 对 BI/BE 速度结合最新的 BS/BD 信息做质量判定；
///       * 仅当质量 OK 时，填充 DvlRtOutput 并返回 true；
///   - 若某帧只包含 BS/BD 而无速度（BI/BE），process() 可以返回 false，
///     但会更新内部状态，为下一次 BI/BE 提供参考。
 */
class DvlRtPreprocessor {
public:
    explicit DvlRtPreprocessor(const DvlRtPreprocessConfig& cfg = DvlRtPreprocessConfig{});

    /// @brief 重置内部状态（统计清零、上一次输出清空）。
    void reset();

    /// @brief 替换配置（即时生效，不自动 reset）。
    void setConfig(const DvlRtPreprocessConfig& cfg);

    /// @brief 获取当前配置。
    [[nodiscard]] DvlRtPreprocessConfig config() const;

    /// @brief 获取内部状态快照。
    [[nodiscard]] DvlRtPreprocessState state() const;

    /// @brief 主入口：处理一帧原始 DVL 采样，给出预处理后的输出。
    ///
    /// 预期实现步骤（在 .cpp 中完成）：
    ///
    ///   1) 更新时间戳与计数器（total_samples 等）；
    ///
    ///   2) 若 kind 属于 BS/BD：
    ///        - 更新内部“最新 bottom-lock / range / corr / FOM”等缓存；
    ///        - 更新 bottom_lock 判定（结合 min_corr / min_good_beams / alt 范围）；
    ///        - 通常不直接对外输出（返回 false），但为下一帧 BI/BE 做准备；
    ///
    ///   3) 若 kind 属于 BI/BE：
    ///        - 从原始速度取出 v_i 或 v_e；
    ///        - 结合最近的 BS/BD 缓存信息，做如下门控：
    ///            * bottom_lock（若 require_bottom_lock）；
///            * beams 相关系数 >= min_corr 且数量 >= min_good_beams；
///            * alt_bottom_m 在 [min_alt_m, max_alt_m] 范围内；
///            * |v| 不超过 max_abs_speed_mps；
///        - 若通过门控：
///            * 对速度向量应用 dvl_noise_floor_mps（norm < floor 则置零）；
///            * 若 use_be_as_enu 且当前为 BE，则直接写 vel_be_enu；
///            * 若当前为 BI，则写 vel_bi_body，
///              并可由上层 ESKF 使用 yaw 把其旋转到 ENU；
///            * 填充 DvlRtOutput（时间戳 / bottom_lock / alt / mean_corr 等）；
///            * 更新 state.last_output & 统计计数，返回 true；
///        - 若未通过门控：
///            * 更新 state.total_rejected / last_reason_code；
///            * 返回 false；
    ///
    /// @param in   原始 DVL 采样（DvlDriver 提供）
/// @param out  输出：若返回 true，则包含最新通过门控的底速信息
    /// @return true  当前帧产生了“可用于导航”的底速输出
    /// @return false 当前帧仅更新内部状态 / 被门控丢弃
    bool process(const DvlRawSample& in, DvlRtOutput& out);

private:
    DvlRtPreprocessConfig cfg_{};
    DvlRtPreprocessState  st_{};

    // 内部缓存：最近一次 BS/BD 解析出的质量信息、beam 相关系数等。
    // 实际字段在 .cpp 中定义（例如 last_corr_[4], last_range_[4], last_bottom_lock_raw_ 等）。
};

} // namespace nav_core::preprocess
