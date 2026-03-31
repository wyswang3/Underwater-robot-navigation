// nav_core/include/nav_core/app/nav_daemon_devices.hpp
//
// @file  nav_daemon_devices.hpp
// @brief nav_daemon 中“设备接入与在线监督”模块的公共接口。
//
// 角色：
//   - 维护共享传感器缓存，给主循环提供最新 IMU/DVL 数据入口；
//   - 负责 IMU/DVL 驱动的启动、停机、掉线检测和重新绑定；
//   - 把与设备生命周期相关的逻辑从 nav_daemon_runner.cpp 拆出来，保持主循环只关心业务编排。
//
// 边界：
//   - 本模块不做 ESKF 估计逻辑；
//   - 不做具体串口协议解析，协议解析仍在各 driver 内部；
//   - 只在 nav_daemon 主线程中驱动状态机，SharedSensorState 只负责跨线程交换最新样本。
//
#pragma once

#include <cstdint>
#include <mutex>
#include <optional>

#include "nav_core/app/device_binding.hpp"
#include "nav_core/app/nav_daemon_config.hpp"
#include "nav_core/app/nav_daemon_logging.hpp"
#include "nav_core/core/types.hpp"
#include "nav_core/drivers/dvl_driver.hpp"
#include "nav_core/drivers/imu_driver_wit.hpp"
#include "nav_core/io/bin_logger.hpp"
#include "nav_core/preprocess/dvl_rt_preprocessor.hpp"

namespace nav_core::app {

/**
 * @brief nav_daemon 内部共享的“最新样本缓存”。
 *
 * 用途：
 *  - 驱动线程把最新 IMU/DVL 样本写入这里；
 *  - 主循环在每个 tick 拉取一份快照做预处理与融合。
 */
struct SharedSensorState {
    std::mutex m;

    std::optional<ImuFrame> last_imu;
    MonoTimeNs              last_imu_mono_ns{0};

    std::optional<preprocess::DvlRawSample> last_dvl_raw;
    MonoTimeNs                              last_dvl_mono_ns{0};

    std::uint64_t loop_counter{0};
};

/**
 * @brief 单个设备运行时状态封装。
 *
 * 包含：
 *  - `binder`：负责“选哪个设备 / 当前设备处于什么连接状态”；
 *  - `last_reported_*`：用于低频日志去重，避免每个循环重复刷相同状态。
 */
struct ManagedDeviceRuntime {
    DeviceBinder          binder;
    DeviceConnectionState last_reported_state{DeviceConnectionState::DISCONNECTED};
    MonoTimeNs            last_reported_transition_ns{-1};
};

/// 清空共享 IMU 缓存，通常在设备掉线或重连前调用。
void clear_imu_shared_state(SharedSensorState& shared_state);
/// 清空共享 DVL 缓存，通常在设备掉线或重连前调用。
void clear_dvl_shared_state(SharedSensorState& shared_state);

/// 生成 IMU 驱动回调，把驱动线程产出的最新帧写入 SharedSensorState。
drivers::ImuDriverWit::FrameCallback make_imu_frame_callback(SharedSensorState& shared_state);
/// 生成 DVL 驱动回调，把驱动线程产出的最新原始样本写入 SharedSensorState。
drivers::DvlDriver::RawCallback make_dvl_raw_callback(SharedSensorState& shared_state);

/**
 * @brief 维护 IMU 设备生命周期。
 *
 * 负责：
 *  - 设备探测与启动；
 *  - “串口开了但没帧”“驱动线程停了”“帧超时”等离线判定；
 *  - 掉线后的 stop / clear / reconnect。
 *
 * 返回值：
 *  - `true` 表示 IMU 连通性发生变化，上层应重置相关处理链。
 */
bool service_imu_device(const NavDaemonConfig& cfg,
                        SharedSensorState& shared_state,
                        drivers::ImuDriverWit& imu_driver,
                        ManagedDeviceRuntime& runtime,
                        nav_core::BinLogger* timing_logger,
                        NavEventCsvLogger* event_logger,
                        MonoTimeNs now_mono_ns);

/**
 * @brief 维护 DVL 设备生命周期。
 *
 * 返回值：
 *  - `true` 表示 DVL 连通性发生变化，上层应清理依赖 DVL freshness 的缓存。
 */
bool service_dvl_device(const NavDaemonConfig& cfg,
                        SharedSensorState& shared_state,
                        drivers::DvlDriver& dvl_driver,
                        ManagedDeviceRuntime& runtime,
                        nav_core::BinLogger* timing_logger,
                        NavEventCsvLogger* event_logger,
                        MonoTimeNs now_mono_ns);

} // namespace nav_core::app
