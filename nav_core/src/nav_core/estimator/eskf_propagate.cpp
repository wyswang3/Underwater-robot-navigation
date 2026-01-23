// nav_core/src/estimator/eskf_propagate.cpp

#include "nav_core/estimator/eskf.hpp"
#include "nav_core/estimator/eskf_math.hpp"
#include "nav_core/filters/filter_common.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace nav_core::estimator {

using namespace eskf_math;

// 从 filter_common.hpp“桥接”索引，避免再硬编码一套
namespace {
constexpr int IDX_P  = static_cast<int>(nav_core::filters::IDX_POS);
constexpr int IDX_V  = static_cast<int>(nav_core::filters::IDX_VEL);
constexpr int IDX_TH = static_cast<int>(nav_core::filters::IDX_THETA);
constexpr int IDX_BA = static_cast<int>(nav_core::filters::IDX_BA);
constexpr int IDX_BG = static_cast<int>(nav_core::filters::IDX_BG);
} // namespace

bool EskfFilter::propagate_imu(const ImuSample& imu)
{
    const MonoTimeNs t_ns = imu.mono_ns;

    if (last_mono_ns_ == 0) {
        last_mono_ns_ = t_ns;
        return false;
    }

    const double dt = static_cast<double>(t_ns - last_mono_ns_) * 1e-9;
    if (!std::isfinite(dt) || dt <= cfg_.dt_min_s || dt > cfg_.dt_max_s) {
        last_mono_ns_ = t_ns;
        return false;
    }

    last_mono_ns_ = t_ns;

    // 1) IMU 测量减零偏（这里的 ba/bg 是“残余 bias”，主 bias 已由 preproc 吃掉）
    EVec3d omega_meas(
        static_cast<double>(imu.ang_vel[0]),
        static_cast<double>(imu.ang_vel[1]),
        static_cast<double>(imu.ang_vel[2])
    );
    EVec3d acc_meas(
        static_cast<double>(imu.lin_acc[0]),
        static_cast<double>(imu.lin_acc[1]),
        static_cast<double>(imu.lin_acc[2])
    );

    EVec3d bg = toEigen(bg_rad_s_);
    EVec3d ba = toEigen(ba_mps2_);

    EVec3d omega_corr = omega_meas - bg;
    EVec3d a_corr_b   = acc_meas - ba;

    // 2) 姿态积分：nav←body
    const double ang = omega_corr.norm() * dt;
    std::array<double,4> dq{};
    if (ang > 1e-9) {
        const double half = 0.5 * ang;
        const double s = std::sin(half);
        const double c = std::cos(half);
        EVec3d axis = omega_corr / (ang + 1e-12);
        dq[0] = c;
        dq[1] = s * axis.x();
        dq[2] = s * axis.y();
        dq[3] = s * axis.z();
    } else {
        dq[0] = 1.0;
        dq[1] = 0.5 * omega_corr.x() * dt;
        dq[2] = 0.5 * omega_corr.y() * dt;
        dq[3] = 0.5 * omega_corr.z() * dt;
    }

    q_nb_ = quat_mul(q_nb_, dq);
    normalize_quat();

    // 3) 加速度从 body→ENU
    EMat3d R_nb = quat_to_R(q_nb_);
    EVec3d acc_nav = R_nb * a_corr_b;

    // 若 imu_acc_is_linear=false，则内部补重力
    if (!cfg_.imu_acc_is_linear) {
        const EVec3d g_enu(0.0, 0.0, -cfg_.gravity_mps2);
        acc_nav += g_enu;
    }

    // 4) 名义状态积分：v, p
    EVec3d v_enu = toEigen(v_enu_);
    v_enu += acc_nav * dt;

    // 速度泄漏
    if (cfg_.vel_leak_1ps > 0.0) {
        const double fac = std::max(0.0, 1.0 - cfg_.vel_leak_1ps * dt);
        v_enu *= fac;
    }

    // 水平限幅
    const double vxy_max = cfg_.v_hard_max_mps;
    if (std::isfinite(vxy_max) && vxy_max > 0.0) {
        const double vxy = std::sqrt(v_enu.x()*v_enu.x() + v_enu.y()*v_enu.y());
        if (vxy > vxy_max) {
            const double s = vxy_max / (vxy + 1e-12);
            v_enu.x() *= s;
            v_enu.y() *= s;
        }
    }

    // 垂向限幅
    const double vU_max = cfg_.vU_hard_max_mps;
    if (std::isfinite(vU_max) && vU_max > 0.0) {
        if (v_enu.z() >  vU_max) v_enu.z() =  vU_max;
        if (v_enu.z() < -vU_max) v_enu.z() = -vU_max;
    }

    v_enu_ = fromEigen(v_enu);

    // 位置积分（简单 p += v*dt）
    p_enu_.x += v_enu_.x * dt;
    p_enu_.y += v_enu_.y * dt;
    p_enu_.z += v_enu_.z * dt;

    // 5) 协方差传播
    Eigen::Map<EMat15> Pm(P_.data());

    EMat15 F = EMat15::Zero();

    // δṗ = δv
    F.block<3,3>(IDX_P, IDX_V) = EMat3d::Identity();

    // δv̇ = -R_nb * [a_corr_b]_x * δθ - R_nb * δba + 噪声
    EMat3d F_v_th = -R_nb * skew(a_corr_b);
    EMat3d F_v_ba = -R_nb;
    F.block<3,3>(IDX_V, IDX_TH) = F_v_th;
    F.block<3,3>(IDX_V, IDX_BA) = F_v_ba;

    // δθ̇ = -[ω_corr]_x * δθ - δbg + 噪声
    EMat3d F_th_th = -skew(omega_corr);
    EMat3d F_th_bg = -EMat3d::Identity();
    F.block<3,3>(IDX_TH, IDX_TH) = F_th_th;
    F.block<3,3>(IDX_TH, IDX_BG) = F_th_bg;

    EMat15 Phi = EMat15::Identity() + F * dt;

    // 过程噪声
    const double sig_acc   = cfg_.sigma_acc_mps2;
    const double sig_extra = cfg_.q_vel_extra_mps2;
    const double sig_acc_eff = std::hypot(sig_acc, sig_extra);
    const double qa = sig_acc_eff * sig_acc_eff;

    const double sig_gyro = cfg_.sigma_gyro_xyz_rad_s;
    const double qg = sig_gyro * sig_gyro;

    const double q_ba = cfg_.sigma_ba_rw_mps2_sqrt_s * cfg_.sigma_ba_rw_mps2_sqrt_s;
    const double q_bg = cfg_.sigma_bg_rw_rad_s_sqrt_s * cfg_.sigma_bg_rw_rad_s_sqrt_s;

    Eigen::Matrix<double,12,12> Qc = Eigen::Matrix<double,12,12>::Zero();
    Qc.block<3,3>(0,0) = qa * EMat3d::Identity();  // n_a
    Qc.block<3,3>(3,3) = qg * EMat3d::Identity();  // n_g
    Qc.block<3,3>(6,6) = q_ba * EMat3d::Identity();
    Qc.block<3,3>(9,9) = q_bg * EMat3d::Identity();

    Eigen::Matrix<double,15,12> G = Eigen::Matrix<double,15,12>::Zero();
    G.block<3,3>(IDX_V,  0) = R_nb;               // δv 受 n_a
    G.block<3,3>(IDX_TH, 3) = EMat3d::Identity(); // δθ 受 n_g
    G.block<3,3>(IDX_BA, 6) = EMat3d::Identity(); // δba 受 n_ba_rw
    G.block<3,3>(IDX_BG, 9) = EMat3d::Identity(); // δbg 受 n_bg_rw

    EMat15 Qd = (G * Qc * G.transpose()) * dt;

    Pm = Phi * Pm * Phi.transpose() + Qd;
    Pm = sym15(Pm);

    return true;
}

} // namespace nav_core::estimator
