// nav_core/src/nav_core/estimator/nav_health_monitor.cpp

#include "nav_core/estimator/nav_health_monitor.hpp"

#include <algorithm>
#include <cmath>

namespace nav_core::estimator {

// ============================ 工具函数 ============================

namespace {

inline double ns_to_s(MonoTimeNs dt_ns) noexcept
{
    return static_cast<double>(dt_ns) * 1e-9;
}

} // anonymous namespace

// ============================ NavHealthMonitor 实现 ============================

NavHealthMonitor::NavHealthMonitor(const NavHealthConfig& cfg)
    : cfg_(cfg)
{
    // stats_window_begin_ns_ 保持 0，待首次 notify_* / evaluate 时初始化
}

void NavHealthMonitor::setConfig(const NavHealthConfig& cfg) noexcept
{
    cfg_ = cfg;
}

// -------------------- 滤波器整体事件 --------------------

void NavHealthMonitor::notify_eskf_reset(MonoTimeNs now_mono_ns) noexcept
{
    eskf_initialized_   = true;
    eskf_last_reset_ns_ = now_mono_ns;

    // 重置统计窗口，从当前时刻重新累计 ESKF 相关指标
    reset_stats_window(now_mono_ns);
}

void NavHealthMonitor::notify_imu_propagate(MonoTimeNs now_mono_ns) noexcept
{
    // 当前实现中仅记录“最后一次评估时间”的参考，
    // 若将来需要统计 IMU 预测频率，可在此处扩展计数。
    last_eval_mono_ns_ = now_mono_ns;

    if (stats_window_begin_ns_ == 0) {
        // 尚未初始化统计窗口，则以第一次调用的时间作为起点
        reset_stats_window(now_mono_ns);
    }
}

// -------------------- ESKF 观测更新事件 --------------------

void NavHealthMonitor::notify_dvl_xy_update(MonoTimeNs now_mono_ns,
                                            double nis,
                                            bool accepted) noexcept
{
    // 初始化 / 滑动重置统计窗口
    if (stats_window_begin_ns_ == 0) {
        reset_stats_window(now_mono_ns);
    } else {
        const double window_s = cfg_.stats_window_duration_s;
        if (window_s > 0.0) {
            const double dt_s = ns_to_s(now_mono_ns - stats_window_begin_ns_);
            if (dt_s > window_s) {
                reset_stats_window(now_mono_ns);
            }
        }
    }

    ++dvl_xy_updates_total_;
    if (accepted) {
        ++dvl_xy_updates_accepted_;
    }

    if (nis >= 0.0) {
        dvl_xy_nis_last_ = nis;
        dvl_xy_nis_sum_  += nis;
        if (nis > dvl_xy_nis_max_) {
            dvl_xy_nis_max_ = nis;
        }
    }
}

void NavHealthMonitor::notify_z_update(MonoTimeNs now_mono_ns,
                                       double nis,
                                       bool accepted) noexcept
{
    // 与 DVL XY 相同的窗口逻辑
    if (stats_window_begin_ns_ == 0) {
        reset_stats_window(now_mono_ns);
    } else {
        const double window_s = cfg_.stats_window_duration_s;
        if (window_s > 0.0) {
            const double dt_s = ns_to_s(now_mono_ns - stats_window_begin_ns_);
            if (dt_s > window_s) {
                reset_stats_window(now_mono_ns);
            }
        }
    }

    ++z_updates_total_;
    if (accepted) {
        ++z_updates_accepted_;
    }

    if (nis >= 0.0) {
        z_nis_last_ = nis;
        z_nis_sum_  += nis;
        if (nis > z_nis_max_) {
            z_nis_max_ = nis;
        }
    }
}

// -------------------- 传感器心跳更新 --------------------

void NavHealthMonitor::update_sensor_heartbeat(MonoTimeNs now_mono_ns,
                                               MonoTimeNs last_imu_ns,
                                               MonoTimeNs last_dvl_ns) noexcept
{
    last_eval_mono_ns_ = now_mono_ns;

    last_imu_ns_ = last_imu_ns;
    last_dvl_ns_ = last_dvl_ns;

    // 若尚未有统计窗口，则在第一次收到心跳时初始化
    if (stats_window_begin_ns_ == 0) {
        reset_stats_window(now_mono_ns);
    }
}

// -------------------- 在线导航状态更新 --------------------

void NavHealthMonitor::update_online_state(const shared::msg::NavState& nav) noexcept
{
    last_nav_state_  = nav;
    has_nav_state_   = true;
}

// -------------------- 主评估接口 --------------------

NavHealthReport NavHealthMonitor::evaluate(MonoTimeNs now_mono_ns) const noexcept
{
    NavHealthReport rep{};

    // ---------- 填充基础指标 ----------
    NavHealthMetrics& m = rep.metrics;
    NavHealthDecision& d = rep.decision;

    m.eval_mono_ns     = now_mono_ns;
    m.eskf_initialized = eskf_initialized_;

    // 1) 传感器心跳
    if (last_imu_ns_ > 0) {
        m.imu_age_s = ns_to_s(now_mono_ns - last_imu_ns_);
        m.imu_alive = (m.imu_age_s * 1e9 <= static_cast<double>(cfg_.imu_timeout_ns));
    } else {
        m.imu_age_s = std::numeric_limits<double>::infinity();
        m.imu_alive = false;
    }

    if (last_dvl_ns_ > 0) {
        m.dvl_age_s = ns_to_s(now_mono_ns - last_dvl_ns_);
        m.dvl_alive = (m.dvl_age_s * 1e9 <= static_cast<double>(cfg_.dvl_timeout_ns));
    } else {
        m.dvl_age_s = std::numeric_limits<double>::infinity();
        m.dvl_alive = false;
    }

    // 2) DVL XY 统计
    if (dvl_xy_updates_total_ > 0) {
        const double total   = static_cast<double>(dvl_xy_updates_total_);
        const double accepted = static_cast<double>(dvl_xy_updates_accepted_);
        const double denom   = std::max(1.0, total);

        m.dvl_xy_updates_total   = dvl_xy_updates_total_;
        m.dvl_xy_updates_accepted = dvl_xy_updates_accepted_;

        m.dvl_xy_accept_ratio    = accepted / denom;
        m.dvl_xy_nis_mean        = dvl_xy_nis_sum_ / denom;
        m.dvl_xy_nis_max         = dvl_xy_nis_max_;
        m.dvl_xy_nis_last        = dvl_xy_nis_last_;
    } else {
        // 保持默认 0 值
        m.dvl_xy_updates_total    = 0;
        m.dvl_xy_updates_accepted = 0;
        m.dvl_xy_accept_ratio     = 0.0;
        m.dvl_xy_nis_mean         = 0.0;
        m.dvl_xy_nis_max          = 0.0;
        m.dvl_xy_nis_last         = 0.0;
    }

    // 3) 垂向更新统计
    if (z_updates_total_ > 0) {
        const double total   = static_cast<double>(z_updates_total_);
        const double accepted = static_cast<double>(z_updates_accepted_);
        const double denom   = std::max(1.0, total);

        m.z_updates_total    = z_updates_total_;
        m.z_updates_accepted = z_updates_accepted_;

        m.z_accept_ratio     = accepted / denom;
        m.z_nis_mean         = z_nis_sum_ / denom;
        m.z_nis_max          = z_nis_max_;
        m.z_nis_last         = z_nis_last_;
    } else {
        m.z_updates_total    = 0;
        m.z_updates_accepted = 0;
        m.z_accept_ratio     = 0.0;
        m.z_nis_mean         = 0.0;
        m.z_nis_max          = 0.0;
        m.z_nis_last         = 0.0;
    }

    // 4) 最近一次 NavState 的 yaw / 速度（当前实现中暂不从 NavState 读取，
    //    保持 0.0；如需使用，可在此处按工程约定访问 last_nav_state_ 的字段）
    m.last_yaw_rad      = 0.0;
    m.last_speed_xy_mps = 0.0;

    // ---------- 健康等级判定 ----------

    // 默认：若尚未初始化，则 health=UNINITIALIZED，否则从 OK 开始
    if (!eskf_initialized_) {
        d.health = shared::msg::NavHealth::UNINITIALIZED;
    } else {
        d.health = shared::msg::NavHealth::OK;
    }

    // 1) 传感器掉线情况优先处理
    const bool imu_dead = !m.imu_alive;
    const bool dvl_dead = !m.dvl_alive;

    if (imu_dead) {
        // 没有 IMU，不可能做合理的导航 → 直接 INVALID
        d.health = shared::msg::NavHealth::INVALID;
    } else if (dvl_dead && eskf_initialized_) {
        // 没有 DVL，可以短时间依赖纯惯导，但总体质量退化
        if (d.health == shared::msg::NavHealth::OK) {
            d.health = shared::msg::NavHealth::DEGRADED;
        }
    }

    // 2) 观测接受率与 NIS 统计（仅在 ESKF 已初始化时评估）
    if (eskf_initialized_) {
        // ---- DVL XY ----
        if (m.dvl_xy_updates_total > 0) {
            const double ratio = m.dvl_xy_accept_ratio;

            if (ratio < cfg_.dvl_accept_ratio_invalid) {
                d.health = shared::msg::NavHealth::INVALID;
            } else if (ratio < cfg_.dvl_accept_ratio_degraded &&
                       d.health == shared::msg::NavHealth::OK) {
                d.health = shared::msg::NavHealth::DEGRADED;
            }

            // 根据 NIS 均值判断
            if (m.dvl_xy_nis_mean > cfg_.dvl_nis_reject_max) {
                d.health = shared::msg::NavHealth::INVALID;
            } else if (m.dvl_xy_nis_mean > cfg_.dvl_nis_ok_max &&
                       d.health == shared::msg::NavHealth::OK) {
                d.health = shared::msg::NavHealth::DEGRADED;
            }
        }

        // ---- 垂向观测 Z ----
        if (m.z_updates_total > 0) {
            const double ratio = m.z_accept_ratio;

            if (ratio < cfg_.z_accept_ratio_invalid) {
                d.health = shared::msg::NavHealth::INVALID;
            } else if (ratio < cfg_.z_accept_ratio_degraded &&
                       d.health == shared::msg::NavHealth::OK) {
                d.health = shared::msg::NavHealth::DEGRADED;
            }

            if (m.z_nis_mean > cfg_.z_nis_reject_max) {
                d.health = shared::msg::NavHealth::INVALID;
            } else if (m.z_nis_mean > cfg_.z_nis_ok_max &&
                       d.health == shared::msg::NavHealth::OK) {
                d.health = shared::msg::NavHealth::DEGRADED;
            }
        }
    }

    // 3) status_flags 填充（此处仅预留位，具体与 NavStatusFlags 的映射
    //    建议在后续根据 shared::msg::NavStatusFlags 的定义统一规划）
    d.status_flags = 0;
    // 例如将来可以做类似：
    // if (!m.imu_alive) d.status_flags |= NavStatusFlags::IMU_DEAD;
    // if (!m.dvl_alive) d.status_flags |= NavStatusFlags::DVL_DEAD;
    // if (d.health == shared::msg::NavHealth::DEGRADED) d.status_flags |= NavStatusFlags::NAV_DEGRADED;
    // if (d.health == shared::msg::NavHealth::INVALID)  d.status_flags |= NavStatusFlags::NAV_INVALID;

    // 4) 工程建议
    if (d.health == shared::msg::NavHealth::INVALID) {
        if (cfg_.enable_stop_suggestion) {
            d.recommend_stop_motion = true;
        }
        if (cfg_.enable_reloc_suggestion) {
            d.recommend_relocalize = true;
        }
    } else if (d.health == shared::msg::NavHealth::DEGRADED) {
        if (cfg_.enable_speed_reduce_suggestion) {
            d.recommend_reduce_speed = true;
        }
    }

    return rep;
}

// -------------------- 辅助：重置统计窗口 --------------------

void NavHealthMonitor::reset_stats_window(MonoTimeNs new_begin_ns) noexcept
{
    stats_window_begin_ns_ = new_begin_ns;

    // DVL XY 统计
    dvl_xy_updates_total_    = 0;
    dvl_xy_updates_accepted_ = 0;
    dvl_xy_nis_sum_          = 0.0;
    dvl_xy_nis_max_          = 0.0;
    dvl_xy_nis_last_         = 0.0;

    // 垂向统计
    z_updates_total_         = 0;
    z_updates_accepted_      = 0;
    z_nis_sum_               = 0.0;
    z_nis_max_               = 0.0;
    z_nis_last_              = 0.0;
}

} // namespace nav_core::estimator
