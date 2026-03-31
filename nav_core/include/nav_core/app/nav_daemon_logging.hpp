// nav_core/include/nav_core/app/nav_daemon_logging.hpp
//
// @file  nav_daemon_logging.hpp
// @brief nav_daemon 的低频事件日志、二进制日志和 NavState 发布初始化接口。
//
// 角色：
//   - 封装 nav_events.csv 这类“面向现场排障”的低频结构化日志；
//   - 统一 BinLogger / NavStatePublisher 的初始化与写包入口；
//   - 把日志和发布相关逻辑从主循环中拆出，降低 nav_daemon_runner.cpp 的体积和耦合度。
//
// 边界：
//   - 本模块不维护设备连接状态机；
//   - 不负责协议解析，只负责把外部传入的诊断结果结构化落盘。
//
#pragma once

#include <cstdint>
#include <fstream>
#include <string>

#include "nav_core/app/device_binding.hpp"
#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/sample_timing.hpp"
#include "nav_core/core/types.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/io/log_packets.hpp"
#include "nav_core/io/nav_state_publisher.hpp"

#include "shared/msg/nav_state.hpp"

namespace nav_core::app {

/**
 * @brief NavState 低频状态快照。
 *
 * 用途：
 *  - 仅用于 `nav_events.csv` 去重和状态变化记录；
 *  - 避免每个循环都重复写同一类状态事件。
 */
struct NavPublishSnapshot {
    bool valid{false};
    bool stale{false};
    bool degraded{false};
    std::uint16_t fault_code{0};
    std::uint16_t nav_status_flags{0};

    bool operator==(const NavPublishSnapshot& rhs) const noexcept;
};

/**
 * @brief nav_events.csv 写入器。
 *
 * 目标：
 *  - 记录设备绑定变化、串口打开失败、协议诊断、发布状态变化等低频事件；
 *  - 让现场排障优先看结构化摘要，而不是只依赖 stderr 滚屏。
 */
class NavEventCsvLogger {
public:
    bool init(const NavDaemonConfig& cfg);

    void log_device_bind_state_changed(MonoTimeNs mono_ns,
                                       const DeviceBindingStatus& status);
    void log_serial_open_failed(MonoTimeNs mono_ns,
                                const char* device_label,
                                const std::string& device_path,
                                int baud,
                                const char* reason);
    void log_sensor_update_rejected(MonoTimeNs mono_ns,
                                    const char* sensor_id,
                                    const char* reason_class,
                                    std::uint32_t sample_age_ms,
                                    std::uint16_t fault_code);
    void log_nav_publish_state_changed(MonoTimeNs mono_ns,
                                       const shared::msg::NavState& nav);
    void log_protocol_diagnostic(MonoTimeNs mono_ns,
                                 const char* device_label,
                                 const char* signature,
                                 const std::string& summary,
                                 const std::string& preview_text,
                                 const std::string& preview_hex);

private:
    void log_row(MonoTimeNs mono_ns,
                 const char* event,
                 const char* level,
                 std::uint16_t fault_code,
                 const NavPublishSnapshot& nav_snapshot,
                 const std::string& device_label,
                 const std::string& device_path,
                 const std::string& state,
                 const std::string& reason,
                 const std::string& sensor_id,
                 const std::string& reason_class,
                 std::uint32_t sample_age_ms,
                 const std::string& message);

    std::ofstream ofs_{};
    std::string   run_id_{};
};

/// 根据配置初始化共享内存 NavState 发布器。
bool init_nav_state_publisher(const NavDaemonConfig& cfg,
                              io::NavStatePublisher& pub);

/// 用统一规则初始化某个命名二进制日志文件。
bool init_named_bin_logger(const NavDaemonConfig& cfg,
                           const char* filename,
                           nav_core::BinLogger& logger);

/// 向 timing binlog 写一条标准化 timing trace。
void write_timing_trace(nav_core::BinLogger* logger,
                        io::TimingTraceKind kind,
                        const SampleTiming& timing,
                        MonoTimeNs publish_mono_ns,
                        std::uint32_t age_ms,
                        std::uint16_t flags,
                        shared::msg::NavFaultCode fault_code =
                            shared::msg::NavFaultCode::kNone);

} // namespace nav_core::app
