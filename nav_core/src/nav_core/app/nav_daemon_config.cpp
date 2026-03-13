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

void load_string_list(const YAML::Node& parent,
                      const char*       key,
                      std::vector<std::string>& dst)
{
    if (!parent || !parent.IsMap()) {
        return;
    }
    const YAML::Node node = parent[key];
    if (!node || node.IsNull() || !node.IsSequence()) {
        return;
    }

    std::vector<std::string> values;
    values.reserve(node.size());
    for (const auto& item : node) {
        try {
            values.push_back(item.as<std::string>());
        } catch (const YAML::Exception& e) {
            log_warn(std::string("failed to parse string list key '") + key + "': " + e.what());
            return;
        }
    }
    dst = std::move(values);
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
            load_scalar(loop_n, "nav_loop_hz",   out_cfg.loop.nav_loop_hz);
            load_scalar(loop_n, "max_imu_age_s", out_cfg.loop.max_imu_age_s);
            load_scalar(loop_n, "max_dvl_age_s", out_cfg.loop.max_dvl_age_s);
            // 因子图相关的 smoother_window_s 已经取消，这里不再读取
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

                YAML::Node binding_n = get_child(drv_n, "binding");
                if (binding_n) {
                    auto& b = out_cfg.imu.driver.binding;
                    load_string_list(binding_n, "candidate_paths", b.candidate_paths);
                    load_scalar(binding_n, "expected_by_id_substring", b.expected_by_id_substring);
                    load_scalar(binding_n, "expected_vid", b.expected_vid);
                    load_scalar(binding_n, "expected_pid", b.expected_pid);
                    load_scalar(binding_n, "expected_serial", b.expected_serial);
                    load_scalar(binding_n, "offline_timeout_ms", b.offline_timeout_ms);
                    load_scalar(binding_n, "reconnect_backoff_ms", b.reconnect_backoff_ms);
                }

                // 2.1.1 低层 filter（ImuFilterConfig），给驱动用
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

            // 2.2 实时预处理配置（ImuRtPreprocessConfig）
            //
            // 优先读取 imu.preprocess；若没有则兼容老字段 imu.rt_preproc。
            YAML::Node pre_n = get_child(imu_n, "preprocess");
            if (!pre_n) {
                pre_n = get_child(imu_n, "rt_preproc");
            }
            if (pre_n) {
                auto& pcfg = out_cfg.imu.rt_preproc;

                // 2.2.1 basic 有效性过滤（ImuFilterConfig），挂在 preprocess.basic 下面
                //
                //   imu:
                //     preprocess:
                //       basic:
                //         enable_accel: true
                //         ...
                YAML::Node basic_n = get_child(pre_n, "basic");
                if (basic_n) {
                    auto& b = pcfg.basic;
                    load_scalar(basic_n, "enable_accel",  b.enable_accel);
                    load_scalar(basic_n, "enable_gyro",   b.enable_gyro);
                    load_scalar(basic_n, "enable_euler",  b.enable_euler);
                    load_scalar(basic_n, "max_abs_accel", b.max_abs_accel);
                    load_scalar(basic_n, "max_abs_gyro",  b.max_abs_gyro);
                    load_scalar(basic_n, "max_euler_abs", b.max_euler_abs);
                }

                // 2.2.2 静止窗 / bias 估计参数
                load_scalar(pre_n, "static_window_s",          pcfg.static_window_s);
                load_scalar(pre_n, "static_max_abs_gyro",      pcfg.static_max_abs_gyro);
                load_scalar(pre_n, "static_max_abs_accel_dev", pcfg.static_max_abs_accel_dev);
                load_scalar(pre_n, "static_max_samples",       pcfg.static_max_samples);

                // 2.2.3 重力参考与补偿
                load_scalar(pre_n, "g_ref_m_s2",                   pcfg.g_ref_m_s2);
                load_scalar(pre_n, "enable_gravity_compensation",  pcfg.enable_gravity_compensation);

                // 2.2.4 yaw 轴滤波 + deadband
                load_scalar(pre_n, "yaw_lpf_cutoff_hz",  pcfg.yaw_lpf_cutoff_hz);
                load_scalar(pre_n, "yaw_deadband_rad_s", pcfg.yaw_deadband_rad_s);

                // 2.2.5 全轴 deadband
                load_scalar(pre_n, "gyro_deadband_rad_s",  pcfg.gyro_deadband_rad_s);
                load_scalar(pre_n, "accel_deadband_m_s2",  pcfg.accel_deadband_m_s2);

                // 2.2.6 roll/pitch 来源开关
                load_scalar(pre_n, "use_imu_rp_from_sample", pcfg.use_imu_rp_from_sample);
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

        // 3.1 DVL 串口驱动配置
        YAML::Node drv_n = get_child(dvl_n, "driver");
        if (drv_n) {
            auto& dcfg = out_cfg.dvl.driver;

            load_scalar(drv_n, "port",              dcfg.port);
            load_scalar(drv_n, "baud",              dcfg.baud);
            load_scalar(drv_n, "auto_reconnect",    dcfg.auto_reconnect);
            load_scalar(drv_n, "send_startup_cmds", dcfg.send_startup_cmds);
            YAML::Node binding_n = get_child(drv_n, "binding");
            if (binding_n) {
                auto& b = dcfg.binding;
                load_string_list(binding_n, "candidate_paths", b.candidate_paths);
                load_scalar(binding_n, "expected_by_id_substring", b.expected_by_id_substring);
                load_scalar(binding_n, "expected_vid", b.expected_vid);
                load_scalar(binding_n, "expected_pid", b.expected_pid);
                load_scalar(binding_n, "expected_serial", b.expected_serial);
                load_scalar(binding_n, "offline_timeout_ms", b.offline_timeout_ms);
                load_scalar(binding_n, "reconnect_backoff_ms", b.reconnect_backoff_ms);
            }

            // 新版字段：ping_rate / avg_count
            // （如 YAML 中缺失，则保持结构体默认值）
            load_scalar(drv_n, "ping_rate",         dcfg.ping_rate);
            load_scalar(drv_n, "avg_count",         dcfg.avg_count);

            // ★ 旧版 driver.filter.* 已移除，对应逻辑搬到 DvlRtPreprocessConfig，
            //   这里不再解析 "filter" 子节点。
        } else {
            log_warn("section 'dvl.driver' missing, DVL will use defaults and may be disabled");
        }

        // 3.2 DVL 实时预处理配置（BI/BE 分流、噪声地板等）
        YAML::Node pre_n = get_child(dvl_n, "preprocess");
        if (!pre_n) {
            pre_n = get_child(dvl_n, "rt_preproc");
        }
        if (pre_n) {
            auto& pcfg = out_cfg.dvl.rt_preproc;  // 按你现有的配置结构命名

            load_scalar(pre_n, "max_gap_s",           pcfg.max_gap_s);
            load_scalar(pre_n, "enable_gating",       pcfg.enable_gating);

            load_scalar(pre_n, "min_corr",            pcfg.min_corr);
            load_scalar(pre_n, "min_good_beams",      pcfg.min_good_beams);

            load_scalar(pre_n, "min_alt_m",           pcfg.min_alt_m);
            load_scalar(pre_n, "max_alt_m",           pcfg.max_alt_m);

            load_scalar(pre_n, "max_abs_speed_mps",   pcfg.max_abs_speed_mps);
            load_scalar(pre_n, "dvl_noise_floor_mps", pcfg.dvl_noise_floor_mps);

            load_scalar(pre_n, "require_bottom_lock", pcfg.require_bottom_lock);
            load_scalar(pre_n, "status_mask_ok",      pcfg.status_mask_ok);

            load_scalar(pre_n, "mount_yaw_rad",       pcfg.mount_yaw_rad);
            load_scalar(pre_n, "use_be_as_enu",       pcfg.use_be_as_enu);
           }
    } else {
           log_warn("section 'dvl' missing, DVL will use defaults and may be disabled");
       }
    }

    // ============================ 4. estimator ============================
    {
        YAML::Node est_n = get_child(root, "estimator");
        if (est_n) {
            auto& est_cfg = out_cfg.estimator;

            // 4.1 健康监控总开关（新字段：enable_health）
            //
            // 说明：
            //   - true  时，如果编译时开启了 NAV_CORE_ENABLE_GRAPH 并实现了 NavHealthMonitor，
            //            则运行时可以根据健康状态做告警 / 降级；
            //   - false 时，健康监控逻辑关闭，仅输出导航结果。
            load_scalar(est_n, "enable_health", est_cfg.enable_health);

            // 4.2 健康监控详细参数（NavHealthConfig）
            //
            // 当前 NavHealthConfig 已精简，不再包含旧版的
            //   min_baseline_duration_s / min_baseline_samples /
            //   max_rms_xy_ok_m / max_rms_xy_degraded_m /
            //   max_rms_yaw_ok_rad / max_rms_yaw_degraded_rad /
            //   max_drift_rate_ok_m_per_min / max_drift_rate_degraded_m_per_min
            //
            // 如需在 YAML 中配置新的参数，可以在这里按实际 struct 成员名补充。
            YAML::Node h_n = get_child(est_n, "health");
            if (h_n) {
            // 由于当前 NavHealthConfig 中尚未定义新的可配置字段，
            // 这里先预留接口。如果未来在 nav_health_monitor.hpp /
            // 相关头文件中补充了成员，例如：
            //
            //   double max_xy_drift_m;
            //   double max_yaw_drift_rad;
            //
            // 则可以按如下方式添加：
            //
            // load_scalar(h_n, "max_xy_drift_m",     hcfg.max_xy_drift_m);
            // load_scalar(h_n, "max_yaw_drift_rad",  hcfg.max_yaw_drift_rad);
            //
            // 现在先不从 YAML 中加载任何数值，全部使用结构体默认值，
            // 避免访问不存在的成员导致编译错误。
            }
        } else {
            log_warn("section 'estimator' missing, estimator config will use defaults");
        }
    }

    // ============================ 5. logging ============================
    {
        YAML::Node log_n = get_child(root, "logging");
        if (log_n) {
            load_scalar(log_n, "enable",        out_cfg.logging.enable);
            load_scalar(log_n, "base_dir",      out_cfg.logging.base_dir);
            load_scalar(log_n, "split_by_date", out_cfg.logging.split_by_date);

            // 新版多路日志开关
            load_scalar(log_n, "log_imu_raw",        out_cfg.logging.log_imu_raw);
            load_scalar(log_n, "log_dvl_raw",        out_cfg.logging.log_dvl_raw);
            load_scalar(log_n, "log_imu_processed",  out_cfg.logging.log_imu_processed);
            load_scalar(log_n, "log_dvl_processed",  out_cfg.logging.log_dvl_processed);
            load_scalar(log_n, "log_eskf_state",     out_cfg.logging.log_eskf_state);
            load_scalar(log_n, "log_eskf_update",    out_cfg.logging.log_eskf_update);
            load_scalar(log_n, "log_health_report",  out_cfg.logging.log_health_report);

            // 兼容老字段（如果你还在用旧版 yaml，可以临时复用）
            bool old_log_imu = false, old_log_dvl = false, old_log_eskf = false;
            if (log_n["log_imu"]) {
                load_scalar(log_n, "log_imu", old_log_imu);
                if (!log_n["log_imu_raw"] && !log_n["log_imu_processed"]) {
                    out_cfg.logging.log_imu_raw       = old_log_imu;
                    out_cfg.logging.log_imu_processed = old_log_imu;
                }
            }
            if (log_n["log_dvl"]) {
                load_scalar(log_n, "log_dvl", old_log_dvl);
                if (!log_n["log_dvl_raw"] && !log_n["log_dvl_processed"]) {
                    out_cfg.logging.log_dvl_raw       = old_log_dvl;
                    out_cfg.logging.log_dvl_processed = old_log_dvl;
                }
            }
            if (log_n["log_eskf"]) {
                load_scalar(log_n, "log_eskf", old_log_eskf);
                if (!log_n["log_eskf_state"]) {
                    out_cfg.logging.log_eskf_state = old_log_eskf;
                }
            }
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
