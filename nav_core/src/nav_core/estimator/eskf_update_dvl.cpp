// nav_core/src/estimator/eskf_update_dvl.cpp

#include "nav_core/estimator/eskf.hpp"
#include "nav_core/estimator/eskf_math.hpp"
#include "nav_core/filters/filter_common.hpp"
#include "nav_core/core/math.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace nav_core::estimator {

using namespace eskf_math;
namespace math_core = nav_core::core::math;

// 桥接 filter_common 索引
namespace {
constexpr int IDX_P  = static_cast<int>(nav_core::filters::IDX_POS);
constexpr int IDX_V  = static_cast<int>(nav_core::filters::IDX_VEL);
constexpr int IDX_TH = static_cast<int>(nav_core::filters::IDX_THETA);
constexpr int IDX_BA = static_cast<int>(nav_core::filters::IDX_BA);
constexpr int IDX_BG = static_cast<int>(nav_core::filters::IDX_BG);
} // namespace

// ---------------- DVL 水平更新（BI + yaw-only） ----------------

bool EskfFilter::update_dvl_xy(const DvlSample& dvl,
                               EskfUpdateDiagDvlXY* diag)
{
    if (!dvl.valid || !dvl.has_body_vel ||
        !dvl.bottom_lock ||
        dvl.track_type != nav_core::DvlTrackType::BottomTrack)
    {
        if (diag) {
            diag->ok   = false;
            diag->is_zupt = false;
            diag->note = "REJECT_INVALID_DVL_XY";
            diag->nis  = 0.0;
            diag->nis0 = 0.0;
            diag->nis1 = 0.0;
            diag->speed_pred_h = 0.0;
            diag->speed_meas_h = 0.0;
            diag->ratio_pred_over_meas = 0.0;
            diag->R_inflate  = 1.0;
            diag->HPHt_over_R = 0.0;
            diag->r[0] = diag->r[1] = 0.0;
            diag->S_diag[0] = diag->S_diag[1] = 0.0;
        }
        return false;
    }

    // 当前 yaw（来自 ESKF 姿态）
    nav_core::Vec3d rpy{};
    quat_to_rpy(rpy);
    const double yaw = rpy.z;

    // BI 体速 → ENU（yaw-only）
    nav_core::Vec3d v_bi_body{
        static_cast<double>(dvl.vel_body_mps[0]),
        static_cast<double>(dvl.vel_body_mps[1]),
        static_cast<double>(dvl.vel_body_mps[2])
    };
    nav_core::Vec3d v_enu_meas{};
    math_core::body_to_enu_yaw_only(yaw, v_bi_body, v_enu_meas);

    // 噪声地板
    math_core::apply_dvl_noise_floor(v_enu_meas, math_core::kDvlNoiseFloorMps);

    EVec2d z_raw(v_enu_meas.x, v_enu_meas.y);

    if (!std::isfinite(z_raw.x()) || !std::isfinite(z_raw.y())) {
        if (diag) {
            diag->ok   = false;
            diag->is_zupt = false;
            diag->note = "REJECT_Z_NAN";
            double nan = std::numeric_limits<double>::quiet_NaN();
            diag->nis  = nan;
            diag->nis0 = nan;
            diag->nis1 = nan;
            diag->speed_pred_h = 0.0;
            diag->speed_meas_h = 0.0;
            diag->ratio_pred_over_meas = 0.0;
            diag->R_inflate = 1.0;
            diag->HPHt_over_R = 0.0;
            diag->r[0] = diag->r[1] = nan;
            diag->S_diag[0] = diag->S_diag[1] = nan;
        }
        return false;
    }

    // 预测速度
    EVec2d vhat_xy(v_enu_.x, v_enu_.y);

    const double speed_pred_h     = vhat_xy.norm();
    const double speed_meas_h_raw = z_raw.norm();

    const double eps_sp  = cfg_.meas_speed_eps_mps;
    const double zupt_sp = cfg_.zupt_speed_mps;

    const bool is_zupt = (speed_meas_h_raw <= zupt_sp);

    EVec2d z = is_zupt ? EVec2d::Zero() : z_raw;

    double ratio_pred_over_meas = std::numeric_limits<double>::infinity();
    if (speed_meas_h_raw > eps_sp) {
        ratio_pred_over_meas = speed_pred_h / (speed_meas_h_raw + 1e-12);
    }

    // H, R
    Eigen::Matrix<double,2,15> H = Eigen::Matrix<double,2,15>::Zero();
    H(0, IDX_V + 0) = 1.0;  // δvE
    H(1, IDX_V + 1) = 1.0;  // δvN

    EMat2d Rm;
    if (is_zupt) {
        const double s = cfg_.sigma_dvl_zupt_mps;
        Rm = EMat2d::Identity() * (s * s);
    } else {
        const double s = cfg_.sigma_dvl_xy_mps;
        Rm = EMat2d::Identity() * (s * s);
    }
    Rm = sym2(Rm);
    Rm += cfg_.meas_jitter * EMat2d::Identity();

    EVec2d r = z - vhat_xy;

    Eigen::Map<EMat15> Pm(P_.data());
    EMat2d HPHt = H * Pm * H.transpose();
    EMat2d S = sym2(HPHt + Rm);
    S += cfg_.S_jitter * EMat2d::Identity();

    // NIS0
    double nis0 = 0.0;
    {
        EVec2d tmp;
        Eigen::LLT<EMat2d> llt(S);
        if (llt.info() == Eigen::Success) {
            tmp = llt.solve(r);
        } else {
            tmp = S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(r);
        }
        nis0 = r.dot(tmp);
    }

    const double nis_soft  = cfg_.nis_soft;
    const double nis_hard  = cfg_.nis_hard;
    const double nis_abs_h = cfg_.nis_abs_hard;

    const double ratio_soft = cfg_.ratio_soft;
    const double ratio_hard = cfg_.ratio_hard;

    const double inflate_max = cfg_.r_inflate_max;
    const bool   post_inflate_hard_reject = cfg_.post_inflate_hard_reject;
    const bool   reject_huge = cfg_.reject_huge_residual;

    std::string note = "USED_OK";
    double inflated  = 1.0;

    // ---------- ZUPT ----------
    if (is_zupt) {
        Eigen::Matrix<double,15,2> PHt = Pm * H.transpose();
        Eigen::Matrix<double,15,2> K;

        {
            Eigen::LLT<EMat2d> llt(S);
            if (llt.info() == Eigen::Success) {
                K = PHt * llt.solve(EMat2d::Identity());
            } else {
                K = PHt * S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                           .solve(EMat2d::Identity());
            }
        }

        EVec15 dx = K * r;

        p_enu_.x += dx(IDX_P + 0);
        p_enu_.y += dx(IDX_P + 1);
        p_enu_.z += dx(IDX_P + 2);

        v_enu_.x += dx(IDX_V + 0);
        v_enu_.y += dx(IDX_V + 1);
        v_enu_.z += dx(IDX_V + 2);

        {
            EVec3d dtheta(dx(IDX_TH+0), dx(IDX_TH+1), dx(IDX_TH+2));
            auto dq = small_angle_quat(dtheta);
            q_nb_ = quat_mul(dq, q_nb_);
            normalize_quat();
        }

        ba_mps2_.x  += dx(IDX_BA+0);
        ba_mps2_.y  += dx(IDX_BA+1);
        ba_mps2_.z  += dx(IDX_BA+2);
        bg_rad_s_.x += dx(IDX_BG+0);
        bg_rad_s_.y += dx(IDX_BG+1);
        bg_rad_s_.z += dx(IDX_BG+2);

        EMat15 I15 = EMat15::Identity();
        EMat15 A   = I15 - K * H;
        Pm = A * Pm * A.transpose() + K * Rm * K.transpose();
        Pm = sym15(Pm);

        if (diag) {
            diag->ok   = true;
            diag->is_zupt = true;
            diag->note = "ZUPT_USED";
            diag->nis  = nis0;
            diag->nis0 = nis0;
            diag->nis1 = nis0;
            diag->speed_pred_h = speed_pred_h;
            diag->speed_meas_h = speed_meas_h_raw;
            diag->ratio_pred_over_meas = ratio_pred_over_meas;
            diag->R_inflate = 1.0;
            diag->HPHt_over_R =
                static_cast<double>(HPHt.trace() / (Rm.trace() + 1e-12));
            diag->r[0] = r.x();
            diag->r[1] = r.y();
            diag->S_diag[0] = S(0,0);
            diag->S_diag[1] = S(1,1);
        }
        return true;
    }

    // pre hard reject
    bool hard_ratio_bad = (speed_meas_h_raw > eps_sp) &&
                          (ratio_pred_over_meas > ratio_hard);
    bool hard_nis_bad   = (nis0 > nis_hard) ||
                          (reject_huge && nis0 > nis_abs_h);

    if (hard_nis_bad || hard_ratio_bad) {
        if (diag) {
            diag->ok   = false;
            diag->is_zupt = false;
            diag->note = hard_nis_bad ? "REJECT_NIS" : "REJECT_RATIO";
            diag->nis  = nis0;
            diag->nis0 = nis0;
            diag->nis1 = nis0;
            diag->speed_pred_h = speed_pred_h;
            diag->speed_meas_h = speed_meas_h_raw;
            diag->ratio_pred_over_meas = ratio_pred_over_meas;
            diag->R_inflate = 1.0;
            diag->HPHt_over_R =
                static_cast<double>(HPHt.trace() / (Rm.trace() + 1e-12));
            diag->r[0] = r.x();
            diag->r[1] = r.y();
            diag->S_diag[0] = S(0,0);
            diag->S_diag[1] = S(1,1);
        }
        return false;
    }

    // 软膨胀
    bool soft_ratio_bad = (speed_meas_h_raw > eps_sp) &&
                          (ratio_pred_over_meas > ratio_soft);
    bool need_inflate   = (nis0 > nis_soft) || soft_ratio_bad;

    if (need_inflate) {
        const double nis_target = cfg_.nis_target;

        double f_nis   = std::max(1.0, nis0 / std::max(1e-9, nis_target));
        double f_ratio = 1.0;
        if (soft_ratio_bad) {
            double rr = ratio_pred_over_meas / std::max(1e-9, ratio_soft);
            f_ratio = std::max(1.0, rr * rr);
        }

        inflated = std::min(inflate_max, std::max(f_nis, f_ratio));
        Rm *= inflated;

        note = "INFLATE_R";

        S = sym2(HPHt + Rm);
        S += cfg_.S_jitter * EMat2d::Identity();
    }

    // NIS1
    double nis1 = 0.0;
    {
        EVec2d tmp;
        Eigen::LLT<EMat2d> llt(S);
        if (llt.info() == Eigen::Success) {
            tmp = llt.solve(r);
        } else {
            tmp = S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(r);
        }
        nis1 = r.dot(tmp);
    }

    if (post_inflate_hard_reject &&
        (nis1 > nis_hard || (reject_huge && nis1 > nis_abs_h))) {
        if (diag) {
            diag->ok   = false;
            diag->is_zupt = false;
            diag->note = "REJECT_POST_INFLATE";
            diag->nis  = nis1;
            diag->nis0 = nis0;
            diag->nis1 = nis1;
            diag->speed_pred_h = speed_pred_h;
            diag->speed_meas_h = speed_meas_h_raw;
            diag->ratio_pred_over_meas = ratio_pred_over_meas;
            diag->R_inflate = inflated;
            diag->HPHt_over_R =
                static_cast<double>(HPHt.trace() / (Rm.trace() + 1e-12));
            diag->r[0] = r.x();
            diag->r[1] = r.y();
            diag->S_diag[0] = S(0,0);
            diag->S_diag[1] = S(1,1);
        }
        return false;
    }

    // 正式更新
    Eigen::Matrix<double,15,2> PHt = Pm * H.transpose();
    Eigen::Matrix<double,15,2> K;
    {
        Eigen::LLT<EMat2d> llt(S);
        if (llt.info() == Eigen::Success) {
            K = PHt * llt.solve(EMat2d::Identity());
        } else {
            K = PHt * S.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV)
                       .solve(EMat2d::Identity());
        }
    }

    EVec15 dx = K * r;

    p_enu_.x += dx(IDX_P + 0);
    p_enu_.y += dx(IDX_P + 1);
    p_enu_.z += dx(IDX_P + 2);

    v_enu_.x += dx(IDX_V + 0);
    v_enu_.y += dx(IDX_V + 1);
    v_enu_.z += dx(IDX_V + 2);

    {
        EVec3d dtheta(dx(IDX_TH+0), dx(IDX_TH+1), dx(IDX_TH+2));
        auto dq = small_angle_quat(dtheta);
        q_nb_ = quat_mul(dq, q_nb_);
        normalize_quat();
    }

    ba_mps2_.x  += dx(IDX_BA+0);
    ba_mps2_.y  += dx(IDX_BA+1);
    ba_mps2_.z  += dx(IDX_BA+2);
    bg_rad_s_.x += dx(IDX_BG+0);
    bg_rad_s_.y += dx(IDX_BG+1);
    bg_rad_s_.z += dx(IDX_BG+2);

    EMat15 I15 = EMat15::Identity();
    EMat15 A   = I15 - K * H;
    Pm = A * Pm * A.transpose() + K * Rm * K.transpose();
    Pm = sym15(Pm);

    if (diag) {
        diag->ok   = true;
        diag->is_zupt = false;
        diag->note = note;
        diag->nis  = nis1;
        diag->nis0 = nis0;
        diag->nis1 = nis1;
        diag->speed_pred_h = speed_pred_h;
        diag->speed_meas_h = speed_meas_h_raw;
        diag->ratio_pred_over_meas = ratio_pred_over_meas;
        diag->R_inflate = inflated;
        diag->HPHt_over_R =
            static_cast<double>(HPHt.trace() / (Rm.trace() + 1e-12));
        diag->r[0] = r.x();
        diag->r[1] = r.y();
        diag->S_diag[0] = S(0,0);
        diag->S_diag[1] = S(1,1);
    }

    return true;
}

// ---------------- DVL 垂向 vU 更新 ----------------

bool EskfFilter::update_dvl_z(const DvlSample& dvl,
                              EskfUpdateDiagDvlZ* diag)
{
    if (!cfg_.enable_dvl_z_update) {
        if (diag) {
            diag->ok   = false;
            diag->note = "DISABLED";
            diag->nis  = 0.0;
            diag->r    = 0.0;
            diag->S    = 0.0;
        }
        return false;
    }

    if (!dvl.valid || !dvl.has_enu_vel ||
        !dvl.bottom_lock ||
        dvl.track_type != nav_core::DvlTrackType::BottomTrack)
    {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_INVALID_DVL_Z";
            diag->nis  = 0.0;
            diag->r    = 0.0;
            diag->S    = 0.0;
        }
        return false;
    }

    const double z_meas = static_cast<double>(dvl.vel_enu_mps[2]);

    if (!std::isfinite(z_meas)) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_Z_NAN";
            double nan = std::numeric_limits<double>::quiet_NaN();
            diag->nis  = nan;
            diag->r    = nan;
            diag->S    = nan;
        }
        return false;
    }

    if (std::fabs(z_meas) > cfg_.max_abs_dvl_z_mps) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_Z_ABS";
            diag->nis  = 0.0;
            diag->r    = 0.0;
            diag->S    = 0.0;
        }
        return false;
    }

    const double vU_hat = v_enu_.z;
    const double r      = z_meas - vU_hat;

    const double R = cfg_.sigma_dvl_z_mps * cfg_.sigma_dvl_z_mps;

    Eigen::Map<EMat15> Pm(P_.data());

    Eigen::Matrix<double,1,15> H = Eigen::Matrix<double,1,15>::Zero();
    H(0, IDX_V + 2) = 1.0;   // δvU

    const double HPHt = (H * Pm * H.transpose())(0,0);
    double S = HPHt + R + cfg_.S_jitter;

    if (S <= 0.0 || !std::isfinite(S)) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_S_BAD";
            double nan = std::numeric_limits<double>::quiet_NaN();
            diag->nis  = nan;
            diag->r    = r;
            diag->S    = S;
        }
        return false;
    }

    const double nis = r * r / S;

    if (nis > cfg_.nis_abs_hard && cfg_.reject_huge_residual) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_NIS_Z";
            diag->nis  = nis;
            diag->r    = r;
            diag->S    = S;
        }
        return false;
    }

    Eigen::Matrix<double,15,1> PHt = Pm * H.transpose();
    Eigen::Matrix<double,15,1> K   = PHt / S;

    EVec15 dx = K * r;

    p_enu_.x += dx(IDX_P + 0);
    p_enu_.y += dx(IDX_P + 1);
    p_enu_.z += dx(IDX_P + 2);

    v_enu_.x += dx(IDX_V + 0);
    v_enu_.y += dx(IDX_V + 1);
    v_enu_.z += dx(IDX_V + 2);

    {
        EVec3d dtheta(dx(IDX_TH+0), dx(IDX_TH+1), dx(IDX_TH+2));
        auto dq = small_angle_quat(dtheta);
        q_nb_ = quat_mul(dq, q_nb_);
        normalize_quat();
    }

    ba_mps2_.x  += dx(IDX_BA+0);
    ba_mps2_.y  += dx(IDX_BA+1);
    ba_mps2_.z  += dx(IDX_BA+2);
    bg_rad_s_.x += dx(IDX_BG+0);
    bg_rad_s_.y += dx(IDX_BG+1);
    bg_rad_s_.z += dx(IDX_BG+2);

    EMat15 I15 = EMat15::Identity();
    EMat15 A   = I15 - K * H;
    Pm = A * Pm * A.transpose() + K * R * K.transpose();
    Pm = sym15(Pm);

    if (diag) {
        diag->ok   = true;
        diag->note = "USED_OK";
        diag->nis  = nis;
        diag->r    = r;
        diag->S    = S;
    }

    return true;
}

} // namespace nav_core::estimator
