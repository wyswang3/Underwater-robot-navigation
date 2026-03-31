// nav_core/src/preprocess/dvl_rt_preprocessor.cpp
//
// DVL 实时预处理（Runtime Preprocessor）
// - 输入：DvlRawSample（由 DvlDriver 解析 BI/BE/BS/BD 文本帧后填充）
// - 输出：DvlRtOutput（给 ESKF / MHE / 因子图使用的“干净观测摘要”）
// - 责任：
//     * beams 相关系数 / 离底距离 / bottom-lock / 速度幅值 等门控
//     * I-frame → body(FRD) 的固定 yaw 标定（mount_yaw_rad）
//     * ENU 速度（BE）与 body 速度（BI）的统一输出
//
// 观测策略（当前工程阶段）：
//   - 水平 / 体速：只信 BottomTrack BI（kind=0），从 vel_inst_mps 经过 I→B yaw 旋转得到 vel_bi_body；
//   - 垂向 / ENU：只信 BottomTrack BE（kind=1），直接把 vel_earth_mps 视作 ENU 速度 vel_be_enu；
//   - BS / BD（kind=2/3）：只用于更新 beams 相关系数 / range / bottom_lock 等质量信息，不产生速度输出；
//
// 这样可以避免“误用水团速度作为底速”的风险，默认行为更保守。
// 若将来需要利用 BS 水团速度，可在配置中增加相应开关后扩展 is_bi 判定逻辑。

#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"

#include <cmath>   // std::sqrt, std::cos, std::sin
#include <utility> // std::swap

namespace nav_core::preprocess {

namespace {

// 门控失败原因 bitmask（可多个并存）
enum : std::uint32_t {
    kReject_NoVelocity  = 1u << 0,  ///< 没有任何可用速度
    kReject_BottomLock  = 1u << 1,  ///< 未锁底但 require_bottom_lock=true
    kReject_SpeedTooBig = 1u << 2,  ///< |v| 超过 max_abs_speed_mps
    kReject_Quality     = 1u << 3,  ///< beam 相关系数不足 / 有效 beam 数太少
    kReject_Altitude    = 1u << 4,  ///< 离底距离不在 [min_alt_m, max_alt_m] 内
};

} // anonymous namespace

// ============================ 1. 构造 / 重置 / 配置 ============================

DvlRtPreprocessor::DvlRtPreprocessor(const DvlRtPreprocessConfig& cfg)
    : cfg_(cfg)
{
    reset();
}

void DvlRtPreprocessor::reset()
{
    st_ = DvlRtPreprocessState{};
}

void DvlRtPreprocessor::setConfig(const DvlRtPreprocessConfig& cfg)
{
    cfg_ = cfg;
}

DvlRtPreprocessConfig DvlRtPreprocessor::config() const
{
    return cfg_;
}

DvlRtPreprocessState DvlRtPreprocessor::state() const
{
    return st_;
}

// ============================ 2. 主处理函数 ============================

bool DvlRtPreprocessor::process(const DvlRawSample& in,
                                DvlRtOutput&        out)
{
    // ---------- 0) 统计与基础字段 ----------
    st_.total_samples++;
    st_.last_in_mono_ns = in.mono_ns;

    // 初始化输出
    out = DvlRtOutput{};
    out.sensor_time_ns = in.sensor_time_ns;
    out.recv_mono_ns = in.recv_mono_ns;
    out.consume_mono_ns = in.consume_mono_ns;
    out.mono_ns     = in.mono_ns;
    out.est_ns      = in.est_ns;
    out.bottom_lock = in.bottom_lock;
    out.gated_ok    = false;
    out.reason_code = 0;

    out.has_be      = false;
    out.vel_be_enu  = Vec3d{0.0, 0.0, 0.0};

    out.has_bi      = false;
    out.vel_bi_body = Vec3d{0.0, 0.0, 0.0};

    out.has_alt      = false;
    out.alt_bottom_m = 0.0;

    out.mean_corr   = 0.0f;
    out.min_range_m = 0.0f;

    std::uint32_t reason = 0;

    // ---------- 1) 基于 beams 的质量与离底距离评估 ----------
    //
    // 这里的逻辑融合了你旧实现的做法：
    //   - 统计 range_m 中最小的正值，视作离底距离（alt_bottom_m）；
    //   - 统计 corr 中 >0 的 beams 数与平均相关系数；
    //   - 记录 min_range_m / mean_corr 供日志与上层使用；
    //   - 使用 min_good_beams / min_corr / [min_alt_m, max_alt_m] 做初步门控。
    //
    int   n_corr_beams = 0;
    int   n_good_beams = 0;
    float corr_sum     = 0.0f;

    float min_range_m  = 0.0f;
    bool  has_range    = false;

    for (int i = 0; i < 4; ++i) {
        const float r = in.range_m[i];
        const float c = in.corr[i];

        // 有效测距
        if (r > 0.0f) {
            if (!has_range || r < min_range_m) {
                min_range_m = r;
                has_range   = true;
            }
        }

        // 有效相关系数
        if (c > 0.0f) {
            ++n_corr_beams;
            corr_sum += c;
            if (c >= cfg_.min_corr) {
                ++n_good_beams;
            }
        }
    }

    if (has_range) {
        out.has_alt      = true;
        out.alt_bottom_m = static_cast<double>(min_range_m);
        out.min_range_m  = min_range_m;
    }

    if (n_corr_beams > 0) {
        out.mean_corr = corr_sum / static_cast<float>(n_corr_beams);
    } else {
        out.mean_corr = 0.0f;
    }

    // beams 质量门控
    if (n_good_beams < static_cast<int>(cfg_.min_good_beams)) {
        reason |= kReject_Quality;
    }

    // 离底距离门控
    if (out.has_alt) {
        if (out.alt_bottom_m < cfg_.min_alt_m ||
            out.alt_bottom_m > cfg_.max_alt_m) {
            reason |= kReject_Altitude;
        }
    }

    // bottom-lock 门控（若配置要求）
    if (cfg_.require_bottom_lock && !in.bottom_lock) {
        reason |= kReject_BottomLock;
    }

    // ---------- 2) 速度观测：BI / BE ----------
    //
    // kind 约定（结合你现在的 DVL 流水）： 
    //   0: BI / 底跟踪 Instrument frame
    //   1: BE / 底跟踪 Earth frame (ENU)
    //   2: BS / status / 水团相关信息（当前仅用于质量评估，不产出速度）
    //   3: BD / 累计位移 / debug（当前仅用于离底距离等质量评估）
    //
    const bool is_bi = (in.kind == 0u);                 // BottomTrack / Instrument
    const bool is_be = (in.kind == 1u && cfg_.use_be_as_enu);  // BottomTrack / Earth(ENU)

    bool has_any_vel = false;

    // ---- 2a) BI: 仪器坐标 → 体坐标（body=FRD） ----
    //
    // 这里融合了你旧代码对 mount_yaw_rad 的使用：
    //   - v_inst 视作 I-frame 速度；
    //   - 只绕 z 轴旋转：I→B 的固定 yaw 标定；
    //   - v_body.z 目前直接继承 v_inst.z（pitch/roll 由安装或上层考虑）。
    if (is_bi) {
        Vec3d v_inst{
            static_cast<double>(in.vel_inst_mps.x),
            static_cast<double>(in.vel_inst_mps.y),
            static_cast<double>(in.vel_inst_mps.z)
        };

        const double cy = std::cos(cfg_.mount_yaw_rad);
        const double sy = std::sin(cfg_.mount_yaw_rad);

        Vec3d v_body;
        v_body.x =  cy * v_inst.x - sy * v_inst.y;
        v_body.y =  sy * v_inst.x + cy * v_inst.y;
        v_body.z =  v_inst.z;  // pitch/roll 认为在机械安装或上层中处理

        const double v_norm = std::sqrt(v_body.x * v_body.x +
                                        v_body.y * v_body.y +
                                        v_body.z * v_body.z);

        if (cfg_.max_abs_speed_mps > 0.0 &&
            v_norm > cfg_.max_abs_speed_mps) {
            reason |= kReject_SpeedTooBig;
        } else {
            // 噪声地板：速度太小直接视为 0
            if (v_norm < cfg_.dvl_noise_floor_mps) {
                v_body.x = 0.0;
                v_body.y = 0.0;
                v_body.z = 0.0;
            }

            out.has_bi      = true;
            out.vel_bi_body = v_body;
            has_any_vel     = true;
        }
    }

    // ---- 2b) BE: 直接视作 ENU 速度 ----
    //
    // 当前阶段假定：vel_earth_mps 即为 ENU (E, N, U)。
    // 若后续发现 Hover H1000 的 Earth 帧不是 ENU，可在 driver 层或这里统一修正。
    if (is_be) {
        Vec3d v_enu{
            static_cast<double>(in.vel_earth_mps.x),
            static_cast<double>(in.vel_earth_mps.y),
            static_cast<double>(in.vel_earth_mps.z)
        };

        const double v_norm = std::sqrt(v_enu.x * v_enu.x +
                                        v_enu.y * v_enu.y +
                                        v_enu.z * v_enu.z);

        if (cfg_.max_abs_speed_mps > 0.0 &&
            v_norm > cfg_.max_abs_speed_mps) {
            reason |= kReject_SpeedTooBig;
        } else {
            if (v_norm < cfg_.dvl_noise_floor_mps) {
                v_enu.x = 0.0;
                v_enu.y = 0.0;
                v_enu.z = 0.0;
            }

            out.has_be     = true;
            out.vel_be_enu = v_enu;
            has_any_vel    = true;
        }
    }

    // 注意：当前版本 BS / BD 不直接产生速度观测，仅更新 beams / alt / bottom_lock。
    // 若完全没有速度，则标记 NoVelocity。
    if (!has_any_vel) {
        reason |= kReject_NoVelocity;
    }

    // ---------- 3) 汇总门控结果 ----------
    //
    // gating 策略：
    //   - cfg_.enable_gating=true：只要 reason!=0 就视为门控失败；
    //   - cfg_.enable_gating=false：忽略 reason 但仍要求有速度（has_any_vel）才输出。
    //
    if (cfg_.enable_gating) {
        if (reason != 0u || !has_any_vel) {
            out.gated_ok    = false;
            out.reason_code = reason;

            st_.last_gated_ok    = false;
            st_.last_reason_code = reason;
            st_.total_rejected++;
            return false;
        }
    } else {
        // 不启用 gating 时，仅以“是否有速度观测”为准。
        if (!has_any_vel) {
            out.gated_ok    = false;
            out.reason_code = reason | kReject_NoVelocity;

            st_.last_gated_ok    = false;
            st_.last_reason_code = out.reason_code;
            st_.total_rejected++;
            return false;
        }
    }

    // 通过门控：认为 out 包含“可用于导航”的底速信息
    out.gated_ok    = true;
    out.reason_code = 0;

    st_.last_ok_mono_ns   = in.mono_ns;
    st_.last_gated_ok     = true;
    st_.last_reason_code  = 0;
    st_.total_ok++;
    st_.last_output       = out;

    return true;
}

} // namespace nav_core::preprocess
