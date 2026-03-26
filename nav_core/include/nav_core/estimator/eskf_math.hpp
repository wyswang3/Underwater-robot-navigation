// nav_core/include/nav_core/estimator/eskf_math.hpp
#pragma once

#include <array>
#include <cmath>

#include <Eigen/Dense>

#include "nav_core/core/types.hpp"
#include "nav_core/core/math.hpp"

namespace nav_core::estimator::eskf_math {

using EVec3d = Eigen::Vector3d;
using EVec2d = Eigen::Vector2d;
using EVec15 = Eigen::Matrix<double, 15, 1>;
using EMat3d = Eigen::Matrix3d;
using EMat2d = Eigen::Matrix2d;
using EMat15 = Eigen::Matrix<double, 15, 15, Eigen::RowMajor>;

// ---------------- Vec3d <-> Eigen ----------------

inline EVec3d toEigen(const nav_core::Vec3d& v)
{
    return {v.x, v.y, v.z};
}

inline nav_core::Vec3d fromEigen(const EVec3d& v)
{
    return nav_core::Vec3d{v.x(), v.y(), v.z()};
}

// ---------------- skew 对称矩阵 ----------------

inline EMat3d skew(const EVec3d& v)
{
    EMat3d S;
    S <<      0.0, -v.z(),  v.y(),
           v.z(),    0.0, -v.x(),
          -v.y(),  v.x(),   0.0;
    return S;
}

// ---------------- 四元数 <-> 旋转矩阵 ----------------
// q = [w,x,y,z], 表示 R_nb (nav ← body)

inline EMat3d quat_to_R(const std::array<double,4>& q)
{
    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    const double ww = w*w;
    const double xx = x*x;
    const double yy = y*y;
    const double zz = z*z;

    const double wx = w*x;
    const double wy = w*y;
    const double wz = w*z;
    const double xy = x*y;
    const double xz = x*z;
    const double yz = y*z;

    EMat3d R;
    R << ww + xx - yy - zz,      2.0*(xy - wz),      2.0*(xz + wy),
                 2.0*(xy + wz), ww - xx + yy - zz,   2.0*(yz - wx),
                 2.0*(xz - wy), 2.0*(yz + wx), ww - xx - yy + zz;
    return R;
}

// 四元数乘法 q = q1 ⊗ q2
inline std::array<double,4> quat_mul(const std::array<double,4>& q1,
                                     const std::array<double,4>& q2)
{
    const double w1 = q1[0];
    const double x1 = q1[1];
    const double y1 = q1[2];
    const double z1 = q1[3];

    const double w2 = q2[0];
    const double x2 = q2[1];
    const double y2 = q2[2];
    const double z2 = q2[3];

    std::array<double,4> q{};
    q[0] = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    q[1] = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    q[2] = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    q[3] = w1*z2 + x1*y2 - y1*x2 + z1*w2;
    return q;
}

// 小角度 δθ → 增量四元数 δq ≈ [1, 0.5 δθ]
inline std::array<double,4> small_angle_quat(const EVec3d& dtheta)
{
    std::array<double,4> dq{};
    dq[0] = 1.0;
    dq[1] = 0.5 * dtheta.x();
    dq[2] = 0.5 * dtheta.y();
    dq[3] = 0.5 * dtheta.z();
    return dq;
}

// RPY → 四元数（ZYX: yaw->pitch->roll, nav←body）
inline std::array<double,4> rpy_to_quat(double roll, double pitch, double yaw)
{
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    std::array<double,4> q{};
    q[0] = cy*cp*cr + sy*sp*sr;
    q[1] = cy*cp*sr - sy*sp*cr;
    q[2] = cy*sp*cr + sy*cp*sr;
    q[3] = sy*cp*cr - cy*sp*sr;
    return q;
}

// ---------------- 对称化工具 ----------------

inline EMat2d sym2(const EMat2d& M)
{
    return 0.5 * (M + M.transpose());
}

inline EMat15 sym15(const EMat15& M)
{
    return 0.5 * (M + EMat15(M.transpose()));
}

// ---------------- yaw wrap ----------------

inline double wrap_yaw(double yaw)
{
    return nav_core::core::math::wrap_angle_rad(yaw);
}

} // namespace nav_core::estimator::eskf_math
