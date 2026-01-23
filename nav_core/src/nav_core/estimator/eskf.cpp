// nav_core/src/nav_core/estimator/eskf.cpp
//
// ESKF 在线导航模块 —— 统合入口 & 管线说明
// =============================================================
//
// 本文件的定位：
//   - “单一入口 + 文档说明”
//   - 对上层（nav_daemon / 控制程序）暴露统一接口：
//         #include "nav_core/filters/eskf.hpp"
//     上层完全不需要关心 ESKF 内部拆成多少个 cpp 文件；
//   - 具体实现已经拆分到：
//       * eskf_core.cpp         : 构造 / reset / 取状态 / 协方差管理
//       * eskf_propagate.cpp    : IMU 传播（predict）
//       * eskf_update_dvl.cpp   : DVL 相关更新（水平 BI / 垂向 BE）
//       * eskf_update_yaw.cpp   : yaw 伪观测更新
//
// 本文件主要用来：
//   1) 说明 nav_core ESKF 管线对“传感器预处理”的工程假设；
//   2) 说明水平 / 垂向上 DVL 与 IMU 的“取信策略”；
//   3) 把 ESKF 当作一个“黑盒模块”统一对外描述，方便新人阅读。
//
// -------------------------------------------------------------
// 1. 坐标系与总体约定
// -------------------------------------------------------------
//   - 全局坐标系：ENU（东-北-上）；
//   - 机体系：body = FRD（前-右-下）；
//   - 姿态 q_nb 表示 nav←body 的旋转；
//   - 深度 / U 方向：U 为 Up（向上为正），与 ENU 约定保持一致；
//
//   传感器经过 nav_core 的预处理模块后，均对齐到上述约定：
//   - IMU 原始数据 → ImuRtPreprocessor（实时预处理）；
//   - DVL 原始数据 → DvlRtPreprocessor（实时预处理）。
//
// -------------------------------------------------------------
// 2. IMU 实时预处理（ImuRtPreprocessor / ImuRtFilter）假设
// -------------------------------------------------------------
// 入口：nav_core::preprocess::ImuRtPreprocessor
//       （内部使用 nav_core::filters::ImuRtFilter）
//
// 对原始 IMU 数据做的事情：
//   1) 基本有效性检查：
//        - NaN / inf 过滤；
//        - 限幅：|ang_vel| < max_abs_gyro, |accel| < max_abs_accel;
//        - 可选欧拉角范围检查；
//   2) 静止窗检测 + 主零偏估计：
//        - 利用“静止窗口”（gyro 模长 + accel 模长接近 g）统计：
//            * gyro_bias[3]  （rad/s）
//            * accel_bias[3] （m/s^2，含重力项）
//        - 累积时间达到 static_window_s 后确认 bias_ready=true；
//   3) 去零偏：
//        - gyro_corr  = gyro_raw  - gyro_bias
//        - accel_corr = accel_raw - accel_bias
//   4) 一阶低通滤波（LPF）：
//        - 角速度：根据 gyro_lpf_cutoff_hz 做一阶单极点低通；
//        - 加速度：根据 accel_lpf_cutoff_hz 做一阶单极点低通；
//   5) deadband 抑制极小随机游走：
//        - 对滤波后的 gyro / accel 每分量应用 deadband；
//   6) 坐标系统一：
//        - 将厂商系统一映射为 body = FRD；
//
// 重要结论（ESKF 的输入约定）：
//   - ImuSample 满足：
//       * imu.ang_vel[3] : 已去主零偏、已低通、已 deadband，单位 rad/s，body=FRD
//       * imu.lin_acc[3] : 已从原始加速度中扣除了重力的“线加速度”，单位 m/s^2，body=FRD
//   - 因为重力已经在预处理阶段扣掉，ESKF 内部不需要再添加或扣除重力；
//   - ESKF 配置中应设置：
//         cfg_.imu_acc_is_linear = true;
//
//   ESKF 内部的 ba_mps2_ / bg_rad_s_：
//   ---------------------------------------------------------
//   - 不再代表“大尺度的主零偏”，而是仅仅表示“残余 bias / 漂移”；
//   - 过程噪声 Q 中对应项 sigma_ba_rw / sigma_bg_rw 可以设得相对较小；
//   - 大头的 bias 校正已经交给 IMU 预处理模块完成。
//
// -------------------------------------------------------------
// 3. DVL 预处理与“水平/垂向取信策略”
// -------------------------------------------------------------
// 3.1 DVL-BI：体速度（Body / Inertial） —— 用于水平速度观测
// -------------------------------------------------------------
// 入口：DvlRtPreprocessor 对“底跟踪 BI 体速”进行预处理：
//   - 原始 DVL 体速 → 统一为 body=FRD；
//   - 基本质量检查：valid / bottom_lock / track_type == BottomTrack；
//   - 可选噪声地板剪裁：|v_body| 很小时直接视为零；
//
// 在 ESKF 中（update_dvl_xy）：
//   - 当前姿态由 IMU+ESKF 提供：q_nb（nav←body）；
//   - 提取 yaw（注意 wrap 到 (-π, π]）：
//         yaw = rpy_nb.z;
//   - 使用 yaw-only 旋转，将 BI 体速投影到 ENU：
//         v_bi_body  →  math::body_to_enu_yaw_only(yaw, v_bi_body, v_enu_meas);
//   - 得到 vE / vN 作为水平速度观测 z = [vE, vN]^T；
//   - 内部策略：
//       * 小速度（|v| < noise_floor）→ 视为 ZUPT，z=0，给水平速度做零速约束；
//       * 结合 IMU 积分速度 v_enu_ 做 NIS 检验、软膨胀 / 硬拒绝；
//
// 工程含义：
//   - 水平方向“取信 BI 体速 + IMU yaw” —— IMU 提供姿态（尤其 yaw），
//     DVL 提供水平速度模和方向，两者互相约束；
//   - BI 体速在锁底工况下可靠，且对局部流场更敏感，适合做水平速度观测；
//   - yaw-only 的假设可以显著简化实现，并且在 ROV 小姿态机动的场景下足够使用。
//
// -------------------------------------------------------------
// 3.2 DVL-BE / ENU：垂向速度（vU）—— 用于垂向速度观测
// -------------------------------------------------------------
// 入口：DvlRtPreprocessor 对“BE / ENU 速度”进行预处理：
//   - 保证 vel_enu_mps[3] 对应 ENU（特别是 z 是 ENU.Up）；
//   - bottom_lock / BottomTrack 门控；
//   - 可以对 |vU| 做合理限幅，剔除明显错误点；
//
// 在 ESKF 中（update_dvl_z）：
//   - 使用 dvl.vel_enu_mps[2] 作为 vU_meas；
//   - 对 vU_meas 做 |vU_meas| < max_abs_dvl_z_mps 的门控；
//   - 与当前状态中的 v_enu_.z 比较，构造垂向速度残差；
//   - 通过一维卡尔曼更新约束垂向速度和与之耦合的姿态 / bias；
//
// 工程含义：
//   - 垂向方向“取信 BE 垂向速度 + IMU 垂向积分”：
//       * IMU 负责连续时间内对垂向加速度积分，能捕捉短时、高频的垂向运动；
//       * DVL-BE 提供低频、绝对参考的 vU，抑制积分漂移；
//       * 若叠加深度传感器（后续可在 ESKF 或高层图优化中加入），则形成
//         垂向位置 / 速度的双重约束，整体稳定性更好。
//
// -------------------------------------------------------------
// 4. ESKF 对外 API（统一入口）
// -------------------------------------------------------------
// 上层只需要关心 eskf.hpp 中的接口：
//
//   #include "nav_core/filters/eskf.hpp"
//
//   using nav_core::estimator::EskfConfig;
//   using nav_core::estimator::EskfFilter;
//   using nav_core::estimator::EskfNominalState;
//   using nav_core::estimator::EskfUpdateDiagDvlXY;
//   using nav_core::estimator::EskfUpdateDiagDvlZ;
//   using nav_core::estimator::EskfUpdateDiagYaw;
//
//   // 1) 初始化
//   EskfConfig cfg = ...;          // 从 YAML 加载（eskf_config_yaml）
//   EskfFilter eskf(cfg);
//
//   // 2) IMU 传播：使用“IMU 实时预处理”的输出
//   eskf.propagate_imu(imu_sample);
//
//   // 3) DVL 更新：使用“DVL 实时预处理”的输出
//   EskfUpdateDiagDvlXY diag_xy;
//   eskf.update_dvl_xy(dvl_sample, &diag_xy);
//
//   EskfUpdateDiagDvlZ diag_z;
//   eskf.update_dvl_z(dvl_sample, &diag_z);
//
//   // 4) yaw 伪观测（例如来自磁罗盘 / 航向滤波）
//   EskfUpdateDiagYaw diag_yaw;
//   eskf.update_yaw(yaw_meas_rad, sigma_yaw_rad, &diag_yaw);
//
//   // 5) 读取当前名义状态
//   EskfNominalState x = eskf.state();
//
// 这样：
//
//   - nav_daemon / 控制程序只依赖一个“ESKF 模块”，不需要自己关心内部数学细节；
//   - 与 IMU / DVL 预处理模块之间的职责边界清晰：
//       * 预处理：做坐标统一、去主零偏、扣重力、滤波、基本质量控制；
//       * ESKF：在“预处理后的干净观测”基础上做残余状态估计 + 多传感器融合；
//   - 水平 / 垂向上的“取信策略”也在本文件说明清楚，方便日后调整参数或更换传感器。
// =============================================================

#include "nav_core/estimator/eskf.hpp"
#include "nav_core/estimator/eskf_math.hpp"
#include "nav_core/filters/filter_common.hpp"
#include "nav_core/core/math.hpp"

namespace nav_core::estimator {

// 目前所有 EskfFilter 的成员函数实现都在：
//   - eskf_core.cpp
//   - eskf_propagate.cpp
//   - eskf_update_dvl.cpp
//   - eskf_update_yaw.cpp
//
// 本文件只作为“逻辑上的单点入口 + 设计文档”存在，
// 不额外实现新的接口，避免重复定义。

} // namespace nav_core::estimator
