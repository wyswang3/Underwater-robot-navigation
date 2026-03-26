// nav_core/src/nav_core/estimator/eskf_config_yaml.cpp
//
// @file  eskf_config_yaml.cpp
// @brief EskfConfig 的 YAML 加载实现，对齐 Python 版 ESKF 2D 配置结构。

#include "nav_core/estimator/eskf_config_yaml.hpp"

#include <iostream>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

namespace nav_core::estimator {

namespace {

// 简单日志工具（你看着改成自己的 log 宏也行）
inline void log_warn(const std::string& msg)
{
    std::cerr << "[eskf_config_yaml][WARN] " << msg << std::endl;
}

inline void log_info(const std::string& msg)
{
    std::cerr << "[eskf_config_yaml][INFO] " << msg << std::endl;
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

} // anonymous namespace

// ---------------------------------------------------------------------
// 主实现：从文件加载 EskfConfig
// ---------------------------------------------------------------------

bool load_eskf_config_from_yaml_file(const std::string& yaml_path,
                                     EskfConfig&        cfg,
                                     std::string*       err_msg)
{
    YAML::Node root;
    try {
        root = YAML::LoadFile(yaml_path);
    } catch (const std::exception& e) {
        if (err_msg) {
            *err_msg = "failed to load ESKF yaml file '" + yaml_path +
                       "': " + e.what();
        }
        log_warn("cannot open or parse ESKF yaml file: " + yaml_path +
                 " : " + e.what());
        return false;
    }

    if (!root || !root.IsMap()) {
        if (err_msg) {
            *err_msg = "ESKF yaml root is not a map: " + yaml_path;
        }
        log_warn("ESKF yaml root is not a map: " + yaml_path);
        return false;
    }

    log_info("loading EskfConfig from: " + yaml_path);

    // ============================ 1. time ============================
    {
        YAML::Node time_n = get_child(root, "time");
        if (time_n) {
            load_scalar(time_n, "dt_max_s", cfg.dt_max_s);
            load_scalar(time_n, "dt_min_s", cfg.dt_min_s);
            load_scalar(time_n, "yaw_wrap", cfg.yaw_wrap);
        } else {
            log_warn("section 'time' missing, using EskfConfig defaults for time");
        }
    }

    // ============================ 2. init ============================
    {
        YAML::Node init_n = get_child(root, "init");
        if (init_n) {
            // yaw_source: "imu" | "config"
            load_scalar(init_n, "yaw_source", cfg.init_yaw_source);

            // ENU 初值（2D 配置只有 E/N，U 使用 EskfConfig 默认）
            load_scalar(init_n, "E_m",   cfg.init_E_m);
            load_scalar(init_n, "N_m",   cfg.init_N_m);
            // 如果 YAML 以后加 U_m，可一起读
            load_scalar(init_n, "U_m",   cfg.init_U_m);

            load_scalar(init_n, "vE_mps", cfg.init_vE_mps);
            load_scalar(init_n, "vN_mps", cfg.init_vN_mps);
            load_scalar(init_n, "vU_mps", cfg.init_vU_mps);

            load_scalar(init_n, "yaw_rad",     cfg.init_yaw_rad);
            load_scalar(init_n, "bgz_rad_s",   cfg.init_bgz_rad_s);

            // P0 子节点
            YAML::Node P0_n = get_child(init_n, "P0");
            if (P0_n) {
                load_scalar(P0_n, "pos_m2",       cfg.P0_pos_m2);
                load_scalar(P0_n, "vel_m2s2",     cfg.P0_vel_m2s2);
                load_scalar(P0_n, "yaw_rad2",     cfg.P0_yaw_rad2);
                load_scalar(P0_n, "bgz_rad2s2",   cfg.P0_bgz_rad2s2);

                // 3D 额外项：如果以后在 YAML 里加，就自动覆盖
                load_scalar(P0_n, "U_m2",         cfg.P0_U_m2);
                load_scalar(P0_n, "vU_m2s2",      cfg.P0_vU_m2s2);
                load_scalar(P0_n, "att_rad2",     cfg.P0_att_rad2);
                load_scalar(P0_n, "ba_m2s4",      cfg.P0_ba_m2s4);
                load_scalar(P0_n, "bg_rad2s2",    cfg.P0_bg_rad2s2);
            }
        } else {
            log_warn("section 'init' missing, using EskfConfig defaults for init");
        }
    }

    // ============================ 3. imu ============================
    {
        YAML::Node imu_n = get_child(root, "imu");
        if (imu_n) {
            load_scalar(imu_n, "acc_is_linear", cfg.imu_acc_is_linear);
            load_scalar(imu_n, "gravity_mps2",  cfg.gravity_mps2);
            load_scalar(imu_n, "yaw_sign",      cfg.yaw_sign);
            load_scalar(imu_n, "yaw_offset_rad", cfg.yaw_offset_rad);
        } else {
            log_warn("section 'imu' missing, using EskfConfig defaults for imu");
        }
    }

    // ====================== 4. process_noise =========================
    {
        YAML::Node proc_n = get_child(root, "process_noise");
        if (proc_n) {
            load_scalar(proc_n, "sigma_acc_mps2",        cfg.sigma_acc_mps2);
            load_scalar(proc_n, "sigma_gyro_z_rad_s",    cfg.sigma_gyro_z_rad_s);
            // 3D gyro 噪声量级：若没写，就用之前的默认；写了就同步
            load_scalar(proc_n, "sigma_gyro_xyz_rad_s",  cfg.sigma_gyro_xyz_rad_s);

            load_scalar(proc_n, "sigma_bgz_rw",          cfg.sigma_bgz_rw);

            // 3D bias RW（可选）
            load_scalar(proc_n, "sigma_ba_rw_mps2_sqrt_s",
                        cfg.sigma_ba_rw_mps2_sqrt_s);
            load_scalar(proc_n, "sigma_bg_rw_rad_s_sqrt_s",
                        cfg.sigma_bg_rw_rad_s_sqrt_s);

            load_scalar(proc_n, "q_vel_extra_mps2",      cfg.q_vel_extra_mps2);
        } else {
            log_warn("section 'process_noise' missing, using EskfConfig defaults");
        }
    }

    // ============================ 5. dvl =============================
    {
        YAML::Node dvl_n = get_child(root, "dvl");
        if (dvl_n) {
            // 对应：sigma_xy_mps → sigma_dvl_xy_mps
            if (dvl_n["sigma_xy_mps"]) {
                load_scalar(dvl_n, "sigma_xy_mps", cfg.sigma_dvl_xy_mps);
            } else {
                load_scalar(dvl_n, "sigma_dvl_xy_mps", cfg.sigma_dvl_xy_mps);
            }

            load_scalar(dvl_n, "zupt_speed_mps",  cfg.zupt_speed_mps);

            // 对应：sigma_zupt_mps → sigma_dvl_zupt_mps
            if (dvl_n["sigma_zupt_mps"]) {
                load_scalar(dvl_n, "sigma_zupt_mps", cfg.sigma_dvl_zupt_mps);
            } else {
                load_scalar(dvl_n, "sigma_dvl_zupt_mps", cfg.sigma_dvl_zupt_mps);
            }

            load_scalar(dvl_n, "meas_speed_eps_mps", cfg.meas_speed_eps_mps);

            // 若以后在 dvl 部分加垂向配置，也可以顺便覆盖：
            load_scalar(dvl_n, "sigma_z_mps",      cfg.sigma_dvl_z_mps);
            load_scalar(dvl_n, "max_abs_z_mps",    cfg.max_abs_dvl_z_mps);
            load_scalar(dvl_n, "enable_dvl_z_update", cfg.enable_dvl_z_update);
        } else {
            log_warn("section 'dvl' missing, using EskfConfig defaults for dvl");
        }
    }

    // ============================ 6. gating ==========================
    {
        YAML::Node gat_n = get_child(root, "gating");
        if (gat_n) {
            load_scalar(gat_n, "nis_soft",   cfg.nis_soft);
            load_scalar(gat_n, "nis_target", cfg.nis_target);
            load_scalar(gat_n, "nis_hard",   cfg.nis_hard);

            load_scalar(gat_n, "ratio_soft", cfg.ratio_soft);
            load_scalar(gat_n, "ratio_hard", cfg.ratio_hard);

            load_scalar(gat_n, "r_inflate_max", cfg.r_inflate_max);
            load_scalar(gat_n, "post_inflate_hard_reject",
                        cfg.post_inflate_hard_reject);

            load_scalar(gat_n, "reject_huge_residual",
                        cfg.reject_huge_residual);
            load_scalar(gat_n, "nis_abs_hard", cfg.nis_abs_hard);
        } else {
            log_warn("section 'gating' missing, using EskfConfig defaults");
        }
    }

    // ========================= 7. stabilizer =========================
    {
        YAML::Node stab_n = get_child(root, "stabilizer");
        if (stab_n) {
            load_scalar(stab_n, "vel_leak_1ps",   cfg.vel_leak_1ps);

            // 对应 v_hard_max_mps（水平）和 vU_hard_max_mps（可用同一值）
            if (stab_n["v_hard_max_mps"]) {
                load_scalar(stab_n, "v_hard_max_mps", cfg.v_hard_max_mps);
                // 若 vU 没单独写，就用同一数值
                if (!stab_n["vU_hard_max_mps"]) {
                    cfg.vU_hard_max_mps = cfg.v_hard_max_mps;
                }
            }
            load_scalar(stab_n, "vU_hard_max_mps", cfg.vU_hard_max_mps);
        } else {
            log_warn("section 'stabilizer' missing, using EskfConfig defaults");
        }
    }

    // ========================== 8. numerics ==========================
    {
        YAML::Node num_n = get_child(root, "numerics");
        if (num_n) {
            load_scalar(num_n, "meas_jitter", cfg.meas_jitter);
            load_scalar(num_n, "S_jitter",    cfg.S_jitter);
        } else {
            log_warn("section 'numerics' missing, using EskfConfig defaults");
        }
    }

    // ================== 9. 其他段：output / diagnostics ==============
    //
    // 这些主要是离线 Python ESKF 用来控制输出/诊断文件的；
    // 对 C++ 在线 ESKF 来说可以直接忽略，因此这里不做映射。
    // 需要的话可以在将来单独扩展。

    log_info("EskfConfig loaded successfully.");
    return true;
}

} // namespace nav_core::estimator
