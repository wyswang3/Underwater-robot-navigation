// nav_core/src/nav_core/app/nav_daemon_config.cpp
//
// @file  nav_daemon_config.cpp
// @brief 从 YAML 文件加载 NavDaemonConfig 的实现。
//        使用 yaml-cpp，尽量保证：
//          - 缺少字段使用结构体默认值；
//          - 出错时返回 false，并在 err_msg 中写清原因；
//          - 尽量打印清晰的日志信息（std::cerr）。
//

#include "nav_core/app/nav_daemon_config.hpp"

#include <yaml-cpp/yaml.h>

#include <iostream>
#include <stdexcept>
#include <string>

namespace nav_core::app {

// --------------------------- 小工具函数 ---------------------------

namespace {

/// 简单的日志输出前缀
inline void log_warn(const std::string& msg)
{
    std::cerr << "[nav_daemon_config][WARN] " << msg << std::endl;
}

inline void log_info(const std::string& msg)
{
    std::cerr << "[nav_daemon_config][INFO] " << msg << std::endl;
}

/// 读取标量字段（若不存在则保持 dst 原值）
template <typename T>
void load_scalar(const YAML::Node& parent,
                 const char*       key,
                 T&                dst)
{
    if (!parent || !parent.IsMap()) {
        return;
    }
    const YAML::Node node = parent[key];
    if (!node || node.IsNull()) {
        return;
    }

    try {
        dst = node.as<T>();
    } catch (const YAML::Exception& e) {
        log_warn(std::string("failed to parse key '") + key + "': " + e.what());
    }
}

/// 获取子节点（不存在则返回空 Node）
inline YAML::Node get_child(const YAML::Node& parent, const char* key)
{
    if (!parent || !parent.IsMap()) {
        return YAML::Node{};
    }
    return parent[key];
}

} // namespace


// --------------------------- 主实现 ---------------------------

bool load_nav_daemon_config_from_yaml(const std::string& yaml_path,
                                      NavDaemonConfig&   out_cfg,
                                      std::string*       err_msg)
{
    // 先用结构体默认值填充
    out_cfg = NavDaemonConfig{};

    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        if (err_msg) {
            *err_msg = "failed to load YAML file '" + yaml_path +
                       "': " + e.what();
        }
        log_warn("cannot open or parse yaml file: " + yaml_path +
                 " : " + e.what());
        return false;
    }

    if (!root || !root.IsMap()) {
        if (err_msg) {
            *err_msg = "yaml root is not a map: " + yaml_path;
        }
        log_warn("yaml root is not a map: " + yaml_path);
        return false;
    }

    log_info("loading nav_daemon config from: " + yaml_path);

    // ============================ 1. loop ============================
    {
        YAML::Node loop_n = get_child(root, "loop");
        if (loop_n) {
            load_scalar(loop_n, "nav_loop_hz",        out_cfg.loop.nav_loop_hz);
            load_scalar(loop_n, "max_imu_age_s",      out_cfg.loop.max_imu_age_s);
            load_scalar(loop_n, "max_dvl_age_s",      out_cfg.loop.max_dvl_age_s);
            load_scalar(loop_n, "smoother_window_s",  out_cfg.loop.smoother_window_s);
        } else {
            log_warn("section 'loop' missing, using defaults");
        }
    }

    // ============================ 2. imu ============================
    {
        YAML::Node imu_n = get_child(root, "imu");
        if (imu_n) {
            load_scalar(imu_n, "enable", out_cfg.imu.enable);

            // 2.1 driver
            YAML::Node drv_n = get_child(imu_n, "driver");
            if (drv_n) {
                load_scalar(drv_n, "port",       out_cfg.imu.driver.port);
                load_scalar(drv_n, "baud",       out_cfg.imu.driver.baud);
                load_scalar(drv_n, "slave_addr", out_cfg.imu.driver.slave_addr);

                // 2.1.1 filter (ImuFilterConfig)
                YAML::Node filt_n = get_child(drv_n, "filter");
                if (filt_n) {
                    auto& f = out_cfg.imu.driver.filter;
                    load_scalar(filt_n, "enable_accel",  f.enable_accel);
                    load_scalar(filt_n, "enable_gyro",   f.enable_gyro);
                    load_scalar(filt_n, "enable_euler",  f.enable_euler);
                    load_scalar(filt_n, "max_abs_accel", f.max_abs_accel);
                    load_scalar(filt_n, "max_abs_gyro",  f.max_abs_gyro);
                    load_scalar(filt_n, "max_euler_abs", f.max_euler_abs);
                }
            } else {
                log_warn("section 'imu.driver' missing, using default ImuConfig");
            }

            // 2.2 rt_filter (ImuRtFilterConfig)
            YAML::Node rtf_n = get_child(imu_n, "rt_filter");
            if (rtf_n) {
                auto& rtf = out_cfg.imu.rt_filter;

                // 新版 ImuRtFilterConfig 至少包含 deadband 这两个参数，先对齐这两个字段；
                // 其他旧字段（static_* / *_bias_* / *_deadzone_* / yaw_unwrap_* 等）已在
                // 结构体中移除，这里先不再从 YAML 读取，避免访问不存在成员。
                load_scalar(rtf_n, "gyro_deadband_rad_s",   rtf.gyro_deadband_rad_s);
                load_scalar(rtf_n, "accel_deadband_m_s2",   rtf.accel_deadband_m_s2);
            }
        } else {
            log_warn("section 'imu' missing, IMU will use defaults and may be disabled");
        }
    }

    // ============================ 3. dvl ============================
    {
        YAML::Node dvl_n = get_child(root, "dvl");
        if (dvl_n) {
            load_scalar(dvl_n, "enable", out_cfg.dvl.enable);

            YAML::Node drv_n = get_child(dvl_n, "driver");
            if (drv_n) {
                load_scalar(drv_n, "port", out_cfg.dvl.driver.port);
                load_scalar(drv_n, "baud", out_cfg.dvl.driver.baud);

                // DvlFilterConfig
                YAML::Node filt_n = get_child(drv_n, "filter");
                if (filt_n) {
                    auto& f = out_cfg.dvl.driver.filter;
                    load_scalar(filt_n, "enable_velocity", f.enable_velocity);
                    load_scalar(filt_n, "enable_altitude", f.enable_altitude);
                    load_scalar(filt_n, "only_valid_flag", f.only_valid_flag);
                    load_scalar(filt_n, "require_all_axes", f.require_all_axes);
                    load_scalar(filt_n, "max_abs_vel_mps", f.max_abs_vel_mps);
                    load_scalar(filt_n, "max_altitude_m",  f.max_altitude_m);
                    load_scalar(filt_n, "min_altitude_m",  f.min_altitude_m);
                    load_scalar(filt_n, "min_quality",     f.min_quality);
                }

                // CS 呼率 / 平均次数（如果你已经在 DvlConfig 中加了这两个字段）
                load_scalar(drv_n, "cs_ping_rate", out_cfg.dvl.driver.cs_ping_rate);
                load_scalar(drv_n, "cs_avg_count", out_cfg.dvl.driver.cs_avg_count);
            } else {
                log_warn("section 'dvl.driver' missing, DVL will use defaults and may be disabled");
            }
        } else {
            log_warn("section 'dvl' missing, DVL will use defaults and may be disabled");
        }
    }

    // ============================ 4. estimator ============================
    {
        YAML::Node est_n = get_child(root, "estimator");
        if (est_n) {
            load_scalar(est_n, "enable_online",   out_cfg.estimator.enable_online);
            load_scalar(est_n, "enable_smoother", out_cfg.estimator.enable_smoother);
            load_scalar(est_n, "enable_health",   out_cfg.estimator.enable_health);

            // 4.1 online (OnlineEstimatorConfig)
            YAML::Node on_n = get_child(est_n, "online");
            if (on_n) {
                auto& ocfg = out_cfg.estimator.online;

                // ZUPT 阈值
                load_scalar(on_n, "zupt_threshold_mps", ocfg.zupt_threshold_mps);

                // YAML 中叫 max_imu_gap_s / max_dvl_gap_s，
                // 对应 OnlineEstimatorConfig 里的 max_imu_dt_s / dvl_timeout_s
                if (on_n["max_imu_gap_s"]) {
                    load_scalar(on_n, "max_imu_gap_s", ocfg.max_imu_dt_s);
                }
                if (on_n["max_dvl_gap_s"]) {
                    load_scalar(on_n, "max_dvl_gap_s", ocfg.dvl_timeout_s);
                }

                // dvl_noise_floor_mps：简单策略——如果配置 use_dvl_noise_floor==true，
                // 没有单独给数值，就用 zupt_threshold_mps 当噪声地板；否则保持默认。
                bool use_noise_floor = false;
                if (on_n["use_dvl_noise_floor"]) {
                    load_scalar(on_n, "use_dvl_noise_floor", use_noise_floor);
                }
                if (use_noise_floor) {
                    // 若 YAML 里单独写了 dvl_noise_floor_mps 就用那个，否则退回 zupt_threshold
                    if (on_n["dvl_noise_floor_mps"]) {
                        load_scalar(on_n, "dvl_noise_floor_mps", ocfg.dvl_noise_floor_mps);
                    } else if (on_n["zupt_threshold_mps"]) {
                        ocfg.dvl_noise_floor_mps = on_n["zupt_threshold_mps"].as<double>();
                    }
                }

                // 是否使用 DVL 垂直速度：把 use_dvl_depth 解释成 use_dvl_vertical
                if (on_n["use_dvl_depth"]) {
                    load_scalar(on_n, "use_dvl_depth", ocfg.use_dvl_vertical);
                }
            }

            // 4.2 smoother / 4.3 health
            //
            // 当前版本你还没确定 GraphSmoother2D / NavHealthMonitor 的最终指标，
            // 这里先只解析 enable_*，具体 sigma / lambda / 阈值保持默认。
            // 未来如果你把 GraphSmoother2DConfig / NavHealthConfig 设计稳定了，
            // 再在这里把字段一一对齐即可。
        } else {
            log_warn("section 'estimator' missing, estimator configs will use defaults");
        }
    }

    // ============================ 5. logging ============================
    {
        YAML::Node log_n = get_child(root, "logging");
        if (log_n) {
            load_scalar(log_n, "enable",        out_cfg.logging.enable);
            load_scalar(log_n, "base_dir",      out_cfg.logging.base_dir);
            load_scalar(log_n, "split_by_date", out_cfg.logging.split_by_date);

            load_scalar(log_n, "log_imu",       out_cfg.logging.log_imu);
            load_scalar(log_n, "log_dvl",       out_cfg.logging.log_dvl);
            load_scalar(log_n, "log_nav_state", out_cfg.logging.log_nav_state);
            load_scalar(log_n, "log_eskf",      out_cfg.logging.log_eskf);
        } else {
            log_warn("section 'logging' missing, logging will use defaults");
        }
    }

    // ============================ 6. publisher ============================
    {
        YAML::Node pub_n = get_child(root, "publisher");
        if (pub_n) {
            YAML::Node shm_n = get_child(pub_n, "shm");
            if (shm_n) {
                auto& shm_cfg = out_cfg.publisher.shm;
                load_scalar(shm_n, "enable",   shm_cfg.enable);
                load_scalar(shm_n, "shm_name", shm_cfg.shm_name);
                load_scalar(shm_n, "shm_size", shm_cfg.shm_size);
            } else {
                log_warn("section 'publisher.shm' missing, SHM publisher will use defaults");
            }
        } else {
            log_warn("section 'publisher' missing, publisher configs will use defaults");
        }
    }

    log_info("nav_daemon config loaded successfully.");
    return true;
}

} // namespace nav_core::app
