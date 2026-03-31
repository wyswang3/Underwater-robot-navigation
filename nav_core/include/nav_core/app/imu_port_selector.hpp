#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "nav_core/app/device_binding.hpp"
#include "nav_core/drivers/imu_driver_wit.hpp"

namespace nav_core::app {

struct ImuPortSelectionOptions {
    int passive_baud{115200};
    int passive_observe_ms{150};
    int active_reply_timeout_ms{80};
    int active_probe_attempts{2};
};

DeviceProbeDecision select_imu_device_port(
    const std::vector<SerialPortIdentity>& devices,
    const drivers::ImuConfig&              imu_cfg,
    const drivers::SerialBindingConfig&    binding_cfg = {},
    const std::string&                     preferred_path = {},
    ImuPortSelectionOptions                options = {});

DeviceBinder::SelectFn make_imu_port_selector(const drivers::ImuConfig& imu_cfg,
                                              ImuPortSelectionOptions   options = {});

} // namespace nav_core::app
