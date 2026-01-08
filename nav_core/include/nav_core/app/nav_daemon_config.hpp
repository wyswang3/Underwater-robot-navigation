// nav_core/include/nav_core/app/nav_daemon_config.hpp
//
// @file  nav_daemon_config.hpp
// @brief 导航守护进程 nav_daemon 的整体配置结构。
// 
// 设计定位：
//   - 作为 YAML / JSON 等配置文件在 C++ 侧的“镜像结构”；
//   - 把 IMU / DVL / 在线估计 / 因子图平滑 / 健康监测 / 日志 / SHM 发布
//     等模块的配置集中在一个顶层 NavDaemonConfig 下；
//   - 不做任何 IO（不依赖 YAML 解析库），具体加载逻辑放在 .cpp 中实现；
//   - app 层只 include 本头文件，就能拿到所有子系统的配置。
// 
// 典型使用：
//   - nav_daemon.cpp：
//       1) NavDaemonConfig cfg;
//       2) load_nav_daemon_config_from_yaml(path, cfg, &err);
//       3) 用 cfg.imu / cfg.dvl / cfg.estimator 等初始化各模块。

#pragma once

#include <string>

#include "nav_core/drivers/imu_driver_wit.hpp"       // drivers::ImuConfig
#include "nav_core/drivers/dvl_driver.hpp"           // drivers::DvlConfig
#include "nav_core/filters/imu_rt_filter.hpp"        // filters::ImuRtFilterConfig
#include "nav_core/filters/filter_common.hpp"        // 噪声/阈值配置（若有）
#include "nav_core/estimator/online_estimator.hpp"   // estimator::OnlineEstimatorConfig
#include "nav_core/estimator/graph_smoother_2d.hpp"  // estimator::GraphSmoother2DConfig
#include "nav_core/estimator/nav_health_monitor.hpp" // estimator::NavHealthMonitor
#include "nav_core/io/nav_state_publisher.hpp"       // io::NavStatePublisherConfig

namespace nav_core {

// 1) IMU driver 配置：drivers::ImuConfig 映射成 nav_core::ImuConfig
using ImuConfig = drivers::ImuConfig;

} // namespace nav_core

// 2) NavHealthMonitor 配置：这里定义一个独立的配置结构体，供 app 使用
namespace nav_core::estimator {

struct NavHealthMonitorConfig {
    // 先给一个占位字段，后续根据需要在 .cpp 中扩展
    double max_pos_rms_m{0.5};   ///< 允许的最大位置均方根误差（示例）
    double max_vel_rms_mps{0.5}; ///< 允许的最大速度均方根误差（示例）
    bool   enable_stop_on_fault{false}; ///< 是否在严重故障时建议停机
};

} // namespace nav_core::estimator

namespace nav_core {

// 3) NavStatePublisher 配置：io::NavStatePublisherConfig → NavStatePublisherConfig
using NavStatePublisherConfig = io::NavStatePublisherConfig;

} // namespace nav_core


namespace nav_core::app {

// ============================ 1. Loop / 时间相关配置 ============================

/**
 * @brief 导航主循环 & 异步窗口相关配置。
 *
 * 说明：
 *   - nav_loop_hz:
 *       导航主循环频率（估计 / 发布 / 健康监测的节拍），
 *       与 IMU 采样频率不同（IMU 由驱动线程按自身节奏推送）。
 *   - max_imu_age_s:
 *       当最新 IMU 样本距离当前时间超过该阈值，认为 IMU 掉线 / 不可信；
 *   - max_dvl_age_s:
 *       同理，用于判断 DVL 是否“有效在线”，可和 DvlDriver::isAlive 配合；
 *   - smoother_window_s:
 *       离线因子图平滑使用的时间窗口长度（例如最近 600s）。
 */
struct LoopTimingConfig {
    double nav_loop_hz{20.0};        ///< 导航主循环频率 (Hz)
    double max_imu_age_s{0.2};       ///< IMU 数据最大允许延迟 (s)
    double max_dvl_age_s{3.0};       ///< DVL 数据最大允许延迟 (s)

    double smoother_window_s{600.0}; ///< 因子图平滑默认窗口长度 (s)，可选
};


// ============================ 2. IMU 部分配置 ============================

/**
 * @brief IMU 子系统配置（驱动 + 实时滤波）。
 *
 * 结构拆分：
 *   - enable:
 *       是否启用 IMU 链路（一般为 true）；若 false，nav_daemon 可以降级运行； 
 *   - driver:
 *       传给 ImuDriverWit 的底层串口 / 从站配置（port/baud/slave_addr 等）；
 *   - rt_filter:
 *       传给 ImuRtFilter 的静止窗 / bias / deadband 参数。
 */
struct ImuSectionConfig {
    bool                          enable{true};      ///< 是否启用 IMU
    nav_core::ImuConfig           driver;            ///< 串口 & Modbus 配置
    nav_core::filters::ImuRtFilterConfig rt_filter;  ///< 实时滤波配置
};


// ============================ 3. DVL 部分配置 ============================

/**
 * @brief DVL 子系统配置（驱动 + 协议级过滤）。
 *
 * 结构拆分：
 *   - enable:
 *       是否启用 DVL 链路；调试 IMU-only 模式时可置 false；
 *   - driver:
 *       DvlDriver 的配置（port / baud / filter）；其中 filter 内部已有
 *       DVL 速度/高度范围等基本门限；
 *   - ping 控制策略通常由更高一层逻辑实现（下水后发送 CS，上岸前 CZ），
 *     不放在这里做自动“上电即启动”，以避免空气中 ping。
 */
struct DvlSectionConfig {
    bool enable{true};

    nav_core::drivers::DvlConfig driver; ///< 串口 & 基本过滤配置
};


// ============================ 4. 估计器部分配置（在线 + 因子图 + 健康监测） ============================

/**
 * @brief 在线导航 / 因子图平滑 / 健康监测的统一配置。
 *
 * 拆分为三个子模块：
 *   - online:
 *       在线估计器（OnlineEstimator）：IMU yaw + DVL 速度 → NavState；
 *   - smoother:
 *       2D 因子图平滑（GraphSmoother2D）：基于 DVL 轨迹做离线/准在线重建；
 *   - health:
 *       导航健康监测（NavHealthMonitor）：在线 vs 平滑参考，输出质量等级、
 *       误差统计与停机建议。
 */
struct EstimatorSectionConfig {
    bool enable_online{true};   ///< 是否启用在线导航估计
    bool enable_smoother{true}; ///< 是否启用因子图平滑（可按需关闭）
    bool enable_health{true};   ///< 是否启用健康监测

    nav_core::estimator::OnlineEstimatorConfig  online;   ///< 在线估计配置
    nav_core::estimator::GraphSmoother2DConfig  smoother; ///< 因子图平滑配置
    nav_core::estimator::NavHealthMonitorConfig health;   ///< 健康监测配置
};


// ============================ 5. 日志写入配置 ============================

/**
 * @brief 二进制日志写入相关配置。
 *
 * 说明：
 *   - base_dir:
 *       日志根目录，例如 "/home/wys/data/nav"；
 *       实际写入一般按日期 / 会话再分子目录；
 *   - enable_*:
 *       控制各类 packet 是否写盘，方便调试时只开部分。
 */
struct LoggingSectionConfig {
    bool        enable{true};                  ///< 总开关
    std::string base_dir{"./data/nav"};        ///< 日志根目录

    bool split_by_date{true};                  ///< 是否按日期分子目录

    bool log_imu{true};                        ///< 是否记录 ImuLogPacket
    bool log_dvl{true};                        ///< 是否记录 DvlLogPacket
    bool log_nav_state{true};                  ///< 是否记录 NavState（若有对应 packet）
    bool log_eskf{false};                      ///< 是否记录 EskfLogPacket（如启用 ESKF）

    // 未来可以增加：max_file_size_mb / rotate_policy 等
};


// ============================ 6. SHM / NavState 发布配置 ============================

/**
 * @brief 导航状态对外发布配置（共享内存 / 未来可扩展 UDP）。
 *
 * 当前版本只封装了共享内存发布（NavStatePublisher），
 * 控制程序在另一进程中做 seqlock 式读取即可。
 */
struct PublisherSectionConfig {
    nav_core::NavStatePublisherConfig shm; ///< SHM 发布配置
    // 未来可扩展 UDP / ZeroMQ 等发布选项
};


// ============================ 7. 顶层 NavDaemonConfig ============================

/**
 * @brief 导航守护进程 nav_daemon 的顶层配置。
 *
 * 逻辑上对应一份 YAML / JSON 配置文件的结构。
 */
struct NavDaemonConfig {
    LoopTimingConfig       loop;       ///< 主循环与窗口时序
    ImuSectionConfig       imu;        ///< IMU 子系统
    DvlSectionConfig       dvl;        ///< DVL 子系统
    EstimatorSectionConfig estimator;  ///< 在线估计 + 因子图 + 健康监测
    LoggingSectionConfig   logging;    ///< 二进制日志写入
    PublisherSectionConfig publisher;  ///< 导航状态发布
};


// ============================ 8. 配置加载接口声明 ============================

/**
 * @brief 从 YAML 文件加载 NavDaemonConfig（在 .cpp 中实现）。
 *
 * 说明：
 *   - 本函数声明仅为方便 nav_daemon.cpp 使用；
 *   - 具体实现可以使用 yaml-cpp，在 nav_core/app/nav_daemon_config.cpp 中完成；
 *   - err_msg 若非空，在解析失败时写入错误信息。
 */
bool load_nav_daemon_config_from_yaml(const std::string& yaml_path,
                                      NavDaemonConfig&   out_cfg,
                                      std::string*       err_msg = nullptr);

} // namespace nav_core::app
