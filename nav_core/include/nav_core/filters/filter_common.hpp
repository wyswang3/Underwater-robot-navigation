// nav_core/include/nav_core/filters/filter_common.hpp
#pragma once

#include <cstddef>

namespace nav_core::filters {

constexpr std::size_t ESKF_ERR_DIM = 15;

// 误差状态索引（位置/速度/姿态/加速度 bias/陀螺 bias）
constexpr std::size_t IDX_POS = 0;   // E, N, U
constexpr std::size_t IDX_VEL = 3;   // vE, vN, vU
constexpr std::size_t IDX_THETA = 6; // droll, dpitch, dyaw
constexpr std::size_t IDX_BA = 9;    // accel bias
constexpr std::size_t IDX_BG = 12;   // gyro bias

// 以后还可以在这里加：build_process_noise_Q(...),
// build_meas_noise_R_dvl_xy(...) 的声明，实现在 eskf.cpp 里。

} // namespace nav_core::filters
