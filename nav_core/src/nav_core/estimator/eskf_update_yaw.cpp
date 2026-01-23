// nav_core/src/estimator/eskf_update_yaw.cpp

#include "nav_core/estimator/eskf.hpp"
#include "nav_core/estimator/eskf_math.hpp"
#include "nav_core/filters/filter_common.hpp"

#include <Eigen/Dense>
#include <cmath>
#include <limits>

namespace nav_core::estimator {

using namespace eskf_math;

// 桥接 filter_common 索引
namespace {
constexpr int IDX_P  = static_cast<int>(nav_core::filters::IDX_POS);
constexpr int IDX_V  = static_cast<int>(nav_core::filters::IDX_VEL);
constexpr int IDX_TH = static_cast<int>(nav_core::filters::IDX_THETA);
constexpr int IDX_BA = static_cast<int>(nav_core::filters::IDX_BA);
constexpr int IDX_BG = static_cast<int>(nav_core::filters::IDX_BG);
} // namespace

bool EskfFilter::update_yaw(double yaw_meas_rad,
                            double sigma_yaw_meas_rad,
                            EskfUpdateDiagYaw* diag)
{
    nav_core::Vec3d rpy{};
    quat_to_rpy(rpy);
    const double yaw_state = rpy.z;

    const double y_meas = wrap_yaw(yaw_meas_rad);
    double yaw_err = wrap_yaw(y_meas - yaw_state);

    const double R = sigma_yaw_meas_rad * sigma_yaw_meas_rad;
    if (!std::isfinite(R) || R <= 0.0) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_BAD_R";
            double nan = std::numeric_limits<double>::quiet_NaN();
            diag->nis  = nan;
            diag->yaw_residual_rad = yaw_err;
        }
        return false;
    }

    Eigen::Map<EMat15> Pm(P_.data());

    Eigen::Matrix<double,1,15> H = Eigen::Matrix<double,1,15>::Zero();
    H(0, IDX_TH + 2) = 1.0;   // δθz

    const double HPHt = (H * Pm * H.transpose())(0,0);
    double S = HPHt + R + cfg_.S_jitter;

    if (S <= 0.0 || !std::isfinite(S)) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_S_BAD";
            double nan = std::numeric_limits<double>::quiet_NaN();
            diag->nis  = nan;
            diag->yaw_residual_rad = yaw_err;
        }
        return false;
    }

    const double nis = yaw_err * yaw_err / S;

    if (nis > cfg_.nis_abs_hard && cfg_.reject_huge_residual) {
        if (diag) {
            diag->ok   = false;
            diag->note = "REJECT_NIS_YAW";
            diag->nis  = nis;
            diag->yaw_residual_rad = yaw_err;
        }
        return false;
    }

    Eigen::Matrix<double,15,1> PHt = Pm * H.transpose();
    Eigen::Matrix<double,15,1> K   = PHt / S;

    EVec15 dx = K * yaw_err;

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
        diag->yaw_residual_rad = yaw_err;
    }

    return true;
}

} // namespace nav_core::estimator
