// nav_core/src/estimator/eskf_core.cpp

#include "nav_core/estimator/eskf.hpp"
#include "nav_core/estimator/eskf_math.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>

namespace nav_core::estimator {

using namespace eskf_math;

EskfFilter::EskfFilter(const EskfConfig& cfg)
    : cfg_(cfg)
{
    EskfNominalState x0{};
    x0.p_enu    = nav_core::Vec3d{cfg_.init_E_m,  cfg_.init_N_m,  cfg_.init_U_m};
    x0.v_enu    = nav_core::Vec3d{cfg_.init_vE_mps, cfg_.init_vN_mps, cfg_.init_vU_mps};
    x0.rpy_nb_rad = nav_core::Vec3d{0.0, 0.0, cfg_.init_yaw_rad};
    x0.ba_mps2  = nav_core::Vec3d{0.0, 0.0, 0.0};
    x0.bg_rad_s = nav_core::Vec3d{0.0, 0.0, cfg_.init_bgz_rad_s};

    reset(x0);
}

void EskfFilter::reset(const EskfNominalState& x0)
{
    p_enu_    = x0.p_enu;
    v_enu_    = x0.v_enu;
    ba_mps2_  = x0.ba_mps2;
    bg_rad_s_ = x0.bg_rad_s;

    const double roll  = x0.rpy_nb_rad.x;
    const double pitch = x0.rpy_nb_rad.y;
    const double yaw   = x0.rpy_nb_rad.z;
    q_nb_ = rpy_to_quat(roll, pitch, yaw);
    normalize_quat();

    reset_covariance();
    last_mono_ns_ = 0;
}

void EskfFilter::reset_covariance()
{
    for (std::size_t i = 0; i < P_.size(); ++i) {
        P_[i] = 0.0;
    }

    auto set_diag = [this](std::size_t idx, double val) {
        P_[idx * 15 + idx] = val;
    };

    // δp: E,N,U
    set_diag(0, cfg_.P0_pos_m2);
    set_diag(1, cfg_.P0_pos_m2);
    set_diag(2, cfg_.P0_U_m2);

    // δv: vE,vN,vU
    set_diag(3, cfg_.P0_vel_m2s2);
    set_diag(4, cfg_.P0_vel_m2s2);
    set_diag(5, cfg_.P0_vU_m2s2);

    // δθ: roll,pitch,yaw
    set_diag(6, cfg_.P0_att_rad2);
    set_diag(7, cfg_.P0_att_rad2);
    set_diag(8, cfg_.P0_yaw_rad2);

    // δba
    set_diag(9,  cfg_.P0_ba_m2s4);
    set_diag(10, cfg_.P0_ba_m2s4);
    set_diag(11, cfg_.P0_ba_m2s4);

    // δbg
    set_diag(12, cfg_.P0_bg_rad2s2);
    set_diag(13, cfg_.P0_bg_rad2s2);
    set_diag(14, cfg_.P0_bgz_rad2s2);
}

EskfNominalState EskfFilter::state() const
{
    EskfNominalState x{};
    x.p_enu    = p_enu_;
    x.v_enu    = v_enu_;
    x.ba_mps2  = ba_mps2_;
    x.bg_rad_s = bg_rad_s_;
    quat_to_rpy(x.rpy_nb_rad);
    return x;
}

// ---------------- 内部工具：quat normalize / quat->rpy / P 对称化 ----------------

void EskfFilter::normalize_quat()
{
    double n2 = q_nb_[0]*q_nb_[0] + q_nb_[1]*q_nb_[1] +
                q_nb_[2]*q_nb_[2] + q_nb_[3]*q_nb_[3];
    if (n2 <= 0.0) {
        q_nb_ = {1.0, 0.0, 0.0, 0.0};
        return;
    }
    const double inv = 1.0 / std::sqrt(n2);
    q_nb_[0] *= inv;
    q_nb_[1] *= inv;
    q_nb_[2] *= inv;
    q_nb_[3] *= inv;
}

void EskfFilter::quat_to_rpy(nav_core::Vec3d& rpy_out) const
{
    EMat3d R = quat_to_R(q_nb_);

    double roll, pitch, yaw;
    pitch = std::asin(std::clamp(-R(2,0), -1.0, 1.0));
    roll  = std::atan2(R(2,1), R(2,2));
    yaw   = std::atan2(R(1,0), R(0,0));
    yaw   = wrap_yaw(yaw);

    rpy_out = nav_core::Vec3d{roll, pitch, yaw};
}

void EskfFilter::symmetrize_P()
{
    Eigen::Map<EMat15> Pm(P_.data());
    Pm = sym15(Pm);
}

} // namespace nav_core::estimator
