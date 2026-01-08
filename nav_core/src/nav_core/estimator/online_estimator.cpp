// nav_core/src/estimator/online_estimator.cpp
//
// 在线导航解算器：IMU 驱动时间轴 + DVL 低频速度约束（轻量级 2D/2.5D）
//
// 当前实现范围（v1）：
//   - IMU 驱动：以 IMU mono_ns 为主时间轴，计算 dt；
//   - 航向角：使用“滤波后的 IMU 角速度 ang_vel[2]”积分得到连续 yaw（机体系 z 轴）；
//   - 速度：使用最近一帧 DVL 的“体坐标系速度”（存放在 vel_enu_mps[3]）+ 当前 yaw，
//           通过 math::body_to_enu_yaw_only() 转到 ENU，叠加噪声地板 / ZUPT / 垂直策略；
//   - 位置：对 ENU 速度做 3D 积分（Euler 法 + 单步位移上限检查）；
//   - DVL 健康：基于 DVL 时间戳做简单掉线超时判断；
//   - NavState：内部维护一份快照，通过 lastNavState() 提供拷贝。
//
// 不在本文件实现的内容：
//   - IMU 线加速度 + DVL 速度的高级融合 / ESKF；
//   - 更复杂的健康分级（目前只做 IMU_OK / DVL_OK 组合）；
//   - 对深度 / USBL / ESKF 状态的详细管理。

#include "nav_core/estimator/online_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace nav_core::estimator {

namespace math = nav_core::core::math;
namespace smsg = shared::msg;

// ============================ 1. 构造 / 配置 / 重置 ============================

OnlineEstimator::OnlineEstimator(const OnlineEstimatorConfig& cfg)
    : cfg_(cfg)
{
    // 其它成员在声明处已有默认值
}

void OnlineEstimator::setConfig(const OnlineEstimatorConfig& cfg)
{
    cfg_ = cfg;
}

void OnlineEstimator::reset(const Vec3d& init_pos_enu,
                            const Vec3d& init_vel_enu,
                            double       init_yaw_rad,
                            double       init_depth_m)
{
    // 1) 内部状态复位
    initialized_        = true;
    last_imu_mono_ns_   = 0;

    last_imu_           = ImuSample{};
    last_dvl_           = DvlSample{};
    last_dvl_mono_ns_   = 0;
    has_dvl_            = false;

    yaw_unwrapped_rad_  = init_yaw_rad;
    pos_enu_            = init_pos_enu;
    vel_enu_            = init_vel_enu;
    depth_m_            = init_depth_m;

    health_             = smsg::NavHealth::UNINITIALIZED;
    status_flags_       = smsg::NAV_FLAG_NONE;

    // 2) 打一份 NavState 快照
    {
        std::lock_guard<std::mutex> lock(mutex_);
        packNavStateNoLock();
    }
}

// ============================ 2. 传感器输入接口实现 ============================

void OnlineEstimator::handleImuSample(const ImuSample& imu)
{
    const MonoTimeNs mono_ns = imu.mono_ns;

    // 第一次收到 IMU：建立时间轴基准
    if (!initialized_) {
        initialized_      = true;
        last_imu_mono_ns_ = mono_ns;
        last_imu_         = imu;

        // 航向初始值：使用 IMU 提供的 yaw（假定已经过 ImuRtFilter）
        yaw_unwrapped_rad_ = static_cast<double>(imu.euler[2]);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            packNavStateNoLock();
        }
        return;
    }

    // 计算 dt（秒）
    double dt_s = 0.0;
    if (last_imu_mono_ns_ > 0 && mono_ns > last_imu_mono_ns_) {
        const double dt_ns = static_cast<double>(mono_ns - last_imu_mono_ns_);
        dt_s = dt_ns * 1e-9;
    }

    // dt 非法或过大：只更新时间轴与 NavState 的时间戳，不做积分
    if (dt_s <= 0.0 || dt_s > cfg_.max_imu_dt_s) {
        last_imu_mono_ns_ = mono_ns;
        last_imu_         = imu;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            packNavStateNoLock();
        }
        return;
    }

    // 1) yaw 积分：使用滤波后的 IMU 角速度 ang_vel[2]（机体系 z 轴）
    integrateYawFromImu(imu, dt_s);

    // 2) 更新 ENU 速度：DVL 体速度 + 当前航向角（yaw-only 旋转）
    updateVelocityEnu(imu, dt_s);

    // 3) 基于 ENU 速度做位置积分
    integratePosition(dt_s);

    // 深度：当前实现暂不从 DVL / 压力计更新，保留 depth_m_，
    // 将来可在其它模块写入 depth_m_。

    // 4) 刷新时间轴与最近一帧 IMU
    last_imu_mono_ns_ = mono_ns;
    last_imu_         = imu;

    // 5) 更新健康标志
    updateDvlHealth(mono_ns);
    updateHealthFlags();

    // 6) 更新 NavState 快照
    {
        std::lock_guard<std::mutex> lock(mutex_);
        packNavStateNoLock();
    }
}

void OnlineEstimator::handleDvlSample(const DvlSample& dvl)
{
    // 记录最新一帧 DVL（假定 vel_enu_mps[3] 存放的是“体坐标系速度”）
    last_dvl_         = dvl;
    last_dvl_mono_ns_ = dvl.mono_ns;

    // has_dvl_ 表示：曾收到过一帧“基本有效”的 DVL 速度
    has_dvl_ = (dvl.valid && dvl.has_enu_vel);

    // 不在这里做坐标变换 / 噪声地板，由 IMU 驱动的 updateVelocityEnu() 统一处理。
    // 不直接更新 NavState，由下一次 IMU 驱动时统一更新。
}

// ============================ 3. 状态查询接口实现 ============================

bool OnlineEstimator::lastNavState(smsg::NavState& out) const
{
    if (!initialized_) {
        return false;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    out = nav_state_;
    return true;
}

// ============================ 4. 内部辅助函数实现 ============================

void OnlineEstimator::integrateYawFromImu(const ImuSample& imu, double dt_s)
{
    // 使用“滤波后的机体系角速度”绕 z 轴积分航向角：
    //
    //   yaw_{k+1} = yaw_k + ω_z * dt
    //
    // bias / deadband 已由 ImuRtFilter 处理，这里直接数值积分。
    const double wz = static_cast<double>(imu.ang_vel[2]);  // rad/s
    yaw_unwrapped_rad_ += wz * dt_s;

    // 如需限制数值发散，可在此处进行 wrap 或额外监控，目前保留连续 yaw。
}

void OnlineEstimator::updateVelocityEnu(const ImuSample& /*imu*/, double /*dt_s*/)
{
    // 当前版本：使用“最近一帧 DVL 体坐标系速度 + 当前 yaw”更新 ENU 速度。
    //
    // 条件：
    //   - has_dvl_ == true；
    //   - last_dvl_.valid && last_dvl_.has_enu_vel；
    //   - DVL 最近一帧距离当前 IMU 时间不超过 dvl_timeout_s。

    if (!has_dvl_) {
        return;
    }
    if (!last_dvl_.valid || !last_dvl_.has_enu_vel) {
        return;
    }
    if (last_dvl_mono_ns_ == 0 || last_imu_mono_ns_ == 0) {
        return;
    }

    // DVL 采样年龄（相对当前 IMU 时间）
    const double age_ns = static_cast<double>(last_imu_mono_ns_ - last_dvl_mono_ns_);
    const double age_s  = age_ns * 1e-9;
    if (age_s < 0.0 || age_s > cfg_.dvl_timeout_s) {
        // 太旧的 DVL 观测不再用于速度更新，但保留 has_dvl_ 状态，
        // 由 updateHealthFlags() 统一决定 DVL_OK / DEGRADED。
        return;
    }

    // 1) 取“体坐标系速度”
    Vec3d v_body{
        static_cast<double>(last_dvl_.vel_enu_mps[0]),
        static_cast<double>(last_dvl_.vel_enu_mps[1]),
        static_cast<double>(last_dvl_.vel_enu_mps[2])
    };

    // 2) 通过 yaw-only 旋转转换到 ENU：v_enu = R(yaw) * v_body
    Vec3d v_enu{};
    math::body_to_enu_yaw_only(yaw_unwrapped_rad_, v_body, v_enu);

    // 3) 噪声地板：|v| < noise_floor → 整向量置零
    math::apply_dvl_noise_floor(v_enu, cfg_.dvl_noise_floor_mps);

    // 4) ZUPT 判定：|v_enu| < zupt_threshold → 认为近似静止
    const bool zupt = math::is_zupt(v_enu, cfg_.zupt_threshold_mps);
    if (zupt) {
        // 近似静止：保守起见直接把水平速度拉到零
        v_enu.x = 0.0;
        v_enu.y = 0.0;
        if (!cfg_.use_dvl_vertical) {
            v_enu.z = 0.0;
        }
    }

    // 5) 垂直分量策略
    if (!cfg_.use_dvl_vertical) {
        v_enu.z = 0.0;
    }

    vel_enu_ = v_enu;
}

void OnlineEstimator::integratePosition(double dt_s)
{
    // 简化版积分：
    //   p_{k+1} ≈ p_k + v_enu_ * dt
    //
    // 若后续需要“严格梯形 + Kahan 补偿”，可以在类成员中增加 prev_vel_enu_
    // 并调用 math::integrate_trapezoid。

    Vec3d delta{
        vel_enu_.x * dt_s,
        vel_enu_.y * dt_s,
        vel_enu_.z * dt_s
    };

    // 单步位移上限检查
    const double step_norm = math::norm(delta);
    if (step_norm > cfg_.max_step_displacement_m && step_norm > 0.0) {
        const double scale = cfg_.max_step_displacement_m / step_norm;
        delta.x *= scale;
        delta.y *= scale;
        delta.z *= scale;
    }

    pos_enu_.x += delta.x;
    pos_enu_.y += delta.y;
    pos_enu_.z += delta.z;
}

void OnlineEstimator::updateDvlHealth(MonoTimeNs /*now_mono_ns*/)
{
    // 当前版本把 DVL 的“是否超时 / 是否可用”集中放到 updateHealthFlags() 中判断，
    // 这里留空，后续如需更细粒度的内部状态可以在此扩展。
}

void OnlineEstimator::updateHealthFlags()
{
    // 基本策略：
    //   - 只用 IMU / DVL 两个信源做一个最小可用的健康判断；
    //   - 其它（深度 / USBL / ESKF）留给后续扩展；
    //
    // 逻辑：
    //   1) 若未初始化：health = UNINITIALIZED, flags = 0；
    //   2) 若 IMU 正常、DVL 正常：health = OK，flags 含 IMU_OK | DVL_OK；
    //   3) 若 IMU 正常、DVL 不正常：health = DEGRADED，flags 含 IMU_OK；
    //   4) 其它情况：health = INVALID，flags = 0。

    if (!initialized_) {
        health_       = smsg::NavHealth::UNINITIALIZED;
        status_flags_ = smsg::NAV_FLAG_NONE;
        return;
    }

    std::uint16_t flags = smsg::NAV_FLAG_NONE;

    // 1) IMU 状态：当前只要有 IMU 时间轴就是 OK
    if (last_imu_mono_ns_ > 0) {
        smsg::nav_flag_set(flags, smsg::NavStatusFlags::NAV_FLAG_IMU_OK);
    }

    // 2) DVL 状态：是否在超时时间内有一帧“有效速度”
    bool dvl_ok = false;
    if (has_dvl_ && last_dvl_mono_ns_ > 0 && last_imu_mono_ns_ > 0) {
        const double dt_ns = static_cast<double>(last_imu_mono_ns_ - last_dvl_mono_ns_);
        const double dt_s  = dt_ns * 1e-9;

        if (dt_s >= 0.0 && dt_s <= cfg_.dvl_timeout_s &&
            last_dvl_.valid && last_dvl_.has_enu_vel)
        {
            dvl_ok = true;
        }
    }

    if (dvl_ok) {
        smsg::nav_flag_set(flags, smsg::NavStatusFlags::NAV_FLAG_DVL_OK);
    }

    // 3) 计算 NavHealth
    const bool imu_ok   = smsg::nav_flag_has(flags, smsg::NavStatusFlags::NAV_FLAG_IMU_OK);
    const bool dvl_good = smsg::nav_flag_has(flags, smsg::NavStatusFlags::NAV_FLAG_DVL_OK);

    if (!imu_ok) {
        health_ = smsg::NavHealth::INVALID;
        flags   = smsg::NAV_FLAG_NONE;  // IMU 挂掉，整体无效
    } else if (imu_ok && dvl_good) {
        health_ = smsg::NavHealth::OK;
    } else { // imu_ok && !dvl_good
        health_ = smsg::NavHealth::DEGRADED;
    }

    status_flags_ = flags;
}

void OnlineEstimator::packNavStateNoLock()
{
    // 将内部 ENU 状态打包进 NavState。
    //
    // 约定：
    //   - pos / vel 视为 ENU: [x, y, z] = [E, N, U]；
    //   - rpy 使用 IMU 的 roll/pitch + 解算器内部的连续 yaw；
    //   - depth 单独使用 depth_m_ 字段，不从 pos_enu_.z 派生；
    //   - omega_b / acc_b 直接使用最后一帧 IMU 的机体系数据。

    // 时间戳：优先使用 IMU mono_ns
    nav_state_.t_ns = static_cast<std::uint64_t>(
        (last_imu_mono_ns_ > 0) ? last_imu_mono_ns_ : 0
    );

    // 位置（ENU）
    nav_state_.pos[0] = pos_enu_.x;
    nav_state_.pos[1] = pos_enu_.y;
    nav_state_.pos[2] = pos_enu_.z;

    // 速度（ENU）
    nav_state_.vel[0] = vel_enu_.x;
    nav_state_.vel[1] = vel_enu_.y;
    nav_state_.vel[2] = vel_enu_.z;

    // 姿态：roll/pitch 来自最后一帧 IMU，yaw 使用连续航向角
    nav_state_.rpy[0] = static_cast<double>(last_imu_.euler[0]); // roll
    nav_state_.rpy[1] = static_cast<double>(last_imu_.euler[1]); // pitch
    nav_state_.rpy[2] = yaw_unwrapped_rad_;                       // yaw (continuous)

    // 深度
    nav_state_.depth = depth_m_;

    // 机体系角速度 / 加速度
    nav_state_.omega_b[0] = static_cast<double>(last_imu_.ang_vel[0]);
    nav_state_.omega_b[1] = static_cast<double>(last_imu_.ang_vel[1]);
    nav_state_.omega_b[2] = static_cast<double>(last_imu_.ang_vel[2]);

    nav_state_.acc_b[0] = static_cast<double>(last_imu_.lin_acc[0]);
    nav_state_.acc_b[1] = static_cast<double>(last_imu_.lin_acc[1]);
    nav_state_.acc_b[2] = static_cast<double>(last_imu_.lin_acc[2]);

    // 健康状态 / 标志位
    nav_state_.health       = health_;
    nav_state_.reserved     = 0;
    nav_state_.status_flags = status_flags_;
}

} // namespace nav_core::estimator
