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

struct SharedSensorState {
    std::mutex m;

    std::optional<ImuFrame> last_imu;
    MonoTimeNs              last_imu_mono_ns{0};

    std::optional<preprocess::DvlRawSample> last_dvl_raw;
    MonoTimeNs                              last_dvl_mono_ns{0};

    std::uint64_t loop_counter{0};
};

struct ManagedDeviceRuntime {
    DeviceBinder          binder;
    DeviceConnectionState last_reported_state{DeviceConnectionState::DISCONNECTED};
    MonoTimeNs            last_reported_transition_ns{-1};
};

void clear_imu_shared_state(SharedSensorState& shared_state);
void clear_dvl_shared_state(SharedSensorState& shared_state);

drivers::ImuDriverWit::FrameCallback make_imu_frame_callback(SharedSensorState& shared_state);
drivers::DvlDriver::RawCallback make_dvl_raw_callback(SharedSensorState& shared_state);

bool service_imu_device(const NavDaemonConfig& cfg,
                        SharedSensorState& shared_state,
                        drivers::ImuDriverWit& imu_driver,
                        ManagedDeviceRuntime& runtime,
                        nav_core::BinLogger* timing_logger,
                        NavEventCsvLogger* event_logger,
                        MonoTimeNs now_mono_ns);

bool service_dvl_device(const NavDaemonConfig& cfg,
                        SharedSensorState& shared_state,
                        drivers::DvlDriver& dvl_driver,
                        ManagedDeviceRuntime& runtime,
                        nav_core::BinLogger* timing_logger,
                        NavEventCsvLogger* event_logger,
                        MonoTimeNs now_mono_ns);

} // namespace nav_core::app
