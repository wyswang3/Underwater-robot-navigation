// nav_core/include/nav_core/app/nav_daemon_config.hpp
//
// @file  nav_daemon_config.hpp
// @brief 导航守护进程 nav_daemon 的整体配置结构。
// 
// 设计定位：
//   - 作为 YAML / JSON 等配置文件在 C++ 侧的“镜像结构”；
//   - 把 IMU / DVL / 在线 ESKF / 健康监测 / 日志 / SHM 发布
//     等模块的配置集中在一个顶层 NavDaemonConfig 下；
//   - 不做任何 IO（不依赖 YAML 解析库），具体加载逻辑放在 .cpp 中实现；
//   - app 层只 include 本头文件，就能拿到所有子系统的配置。

#pragma once

#include <string>

#include "nav_core/drivers/imu_driver_wit.hpp"        // drivers::ImuConfig
#include "nav_core/drivers/dvl_driver.hpp"            // drivers::DvlConfig

// ★ 新版实时预处理头文件
#include "nav_core/preprocess/imu_rt_preprocessor.hpp" // preprocess::ImuRtPreprocessConfig (或类似)
#include "nav_core/preprocess/dvl_rt_preprocessor.hpp" // preprocess::DvlRtPreprocessConfig (或类似)

#include "nav_core/estimator/online_estimator.hpp"    // estimator::OnlineEstimatorConfig（内部可用 ESKF）
#include "nav_core/estimator/nav_health_monitor.hpp"  // estimator::NavHealthConfig / NavHealthMonitor
#include "nav_core/io/nav_state_publisher.hpp"        // io::NavStatePublisherConfig

namespace nav_core {

// 1) IMU driver 配置别名：drivers::ImuConfig → nav_core::ImuConfig
using ImuConfig = drivers::ImuConfig;

// 2) DVL driver 配置别名
using DvlConfig = drivers::DvlConfig;

// 3) NavStatePublisher 配置别名
using NavStatePublisherConfig = io::NavStatePublisherConfig;

} // namespace nav_core

namespace nav_core::app {

// ============================ 1. Loop / 时间相关配置 ============================

struct LoopTimingConfig {
    double nav_loop_hz{20.0};   ///< 导航主循环频率 (Hz)
    double max_imu_age_s{0.2};  ///< IMU 数据最大允许延迟 (s)
    double max_dvl_age_s{3.0};  ///< DVL 数据最大允许延迟 (s)
};


// ============================ 2. IMU 部分配置 ============================

/**
 * @brief IMU 子系统配置（驱动 + 实时预处理）。
 *
 * 流程：
 *   ImuDriverWit (原始帧)
 *     → ImuRtPreprocessor (重力补偿 + 坐标统一到 B=FRD + 滤波)
 *     → 预处理 ImuSample 进入 OnlineEstimator/ESKF
 */
// IMU 小节配置：驱动 + 实时预处理
struct ImuSectionConfig {
    bool enable{true};  ///< 是否启用 IMU 链路

    // 串口 / Modbus 驱动配置
    //
    // 类型来自 imu_driver_wit.hpp：
    //   namespace nav_core::drivers {
    //       struct ImuConfig { ... };
    //   }
    nav_core::drivers::ImuConfig driver{};

    // 实时预处理配置：
    //   - basic: ImuFilterConfig（尖刺 / NaN / 超量程过滤）
    //   - static_window: 静止窗 zero-bias 估计参数
    //   - g_ref / 重力补偿开关
    //   - yaw_lpf / deadband 等
    nav_core::preprocess::ImuRtPreprocessConfig rt_preproc{};
};



// ============================ 3. DVL 部分配置 ============================

/**
 * @brief DVL 子系统配置（驱动 + 实时预处理）。
 *
 * 流程：
 *   DvlDriver (原始 BI/BE 等)
 *     → DvlRtPreprocessor (BI/BE 分流 + ENU 统一 + gating)
 *     → 预处理 DvlSample 进入 OnlineEstimator/ESKF
 */
struct DvlSectionConfig {
    bool              enable{true}; ///< 是否启用 DVL 链路
    nav_core::DvlConfig driver;     ///< 串口 & 基本过滤配置

    // ★ 预处理配置（负责 BI/BE 分开、噪声地板、质量门控等）
    nav_core::preprocess::DvlRtPreprocessConfig rt_preproc;
};

// ============================ 3.5. Volt32 部分配置 ============================

/**
 * @brief Volt32 电压电流板采集配置（只负责日志落盘，不参与导航融合）。
 */
struct VoltSectionConfig {
    bool        enable{false};
    std::string port{"/dev/ttyUSB1"};
    int         baud{115200};
    int         channels{16};
};


// ============================ 4. 估计器部分配置（在线 ESKF + 健康监测） ============================

struct EstimatorSectionConfig {
    bool enable_online{true}; ///< 是否启用在线导航估计（ESKF）
    bool enable_health{true}; ///< 是否启用健康监测

    nav_core::estimator::OnlineEstimatorConfig online; ///< 在线估计（ESKF）配置
    nav_core::estimator::NavHealthConfig       health; ///< 健康监测配置
    std::string eskf_config_path;  ///< 指向上面这份 ESKF 专用 YAML
    
};


// ============================ 5. 日志写入配置 ============================

struct LoggingSectionConfig {
    bool        enable{true};                   ///< 总开关
    std::string base_dir{"./data/nav"};         ///< 日志根目录
    bool        split_by_date{true};            ///< 是否按日期分子目录
    std::string sensor_data_root{"../data"};    ///< 传感器 CSV 根目录（默认 data/YYYY-MM-DD/<sensor>/）
    bool        log_sensor_csv{true};           ///< 是否落盘传感器原始 CSV

    // 1) 传感器原始数据（驱动输出）
    bool log_imu_raw{true};                     ///< ImuLogPacket（原始 IMU）
    bool log_dvl_raw{true};                     ///< DvlLogPacket（原始 DVL）
    bool log_volt_raw{true};                    ///< Volt32 原始采集 CSV

    // 2) 预处理后的传感器数据（进入 ESKF 的版本）
    bool log_imu_processed{true};               ///< 预处理 IMU (B=FRD, 已重力补偿)
    bool log_dvl_processed{true};               ///< 预处理 DVL（BI/BE gating 后）

    // 3) 在线估计器 / ESKF 相关
    bool log_eskf_state{true};                  ///< ESKF 状态 (EskfLogPacket)
    bool log_eskf_update{true};                 ///< ESKF 更新 diag（NIS, 观测类型等）

    // 4) 健康监测
    bool log_health_report{true};               ///< NavHealthReport 概要（可单独 packet）
};


// ============================ 6. SHM / NavState 发布配置 ============================

struct PublisherSectionConfig {
    nav_core::NavStatePublisherConfig shm; ///< SHM 发布配置
};


// ============================ 7. 顶层 NavDaemonConfig ============================

struct NavDaemonConfig {
    LoopTimingConfig       loop;       ///< 主循环与时间阈值
    ImuSectionConfig       imu;        ///< IMU 子系统
    DvlSectionConfig       dvl;        ///< DVL 子系统
    VoltSectionConfig      volt;       ///< Volt32 采集（日志用途）
    EstimatorSectionConfig estimator;  ///< 在线 ESKF + 健康监测
    LoggingSectionConfig   logging;    ///< 二进制日志写入
    PublisherSectionConfig publisher;  ///< 导航状态发布
};


// ============================ 8. 配置加载接口声明 ============================

bool load_nav_daemon_config_from_yaml(const std::string& yaml_path,
                                      NavDaemonConfig&   out_cfg,
                                      std::string*       err_msg = nullptr);

} // namespace nav_core::app
