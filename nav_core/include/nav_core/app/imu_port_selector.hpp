// nav_core/include/nav_core/app/imu_port_selector.hpp
//
// @file  imu_port_selector.hpp
// @brief IMU 串口自动识别模块。
//
// 角色：
//   - 在 nav_daemon 的设备绑定阶段，从扫描到的多个串口候选中找出真正的 IMU 口；
//   - 利用“Volt32 会被动持续输出文本、IMU 需要主机主动发 Modbus 读请求才回复”的行为差异做判别；
//   - 把端口选择逻辑独立出 runner/binder，避免主流程继续膨胀。
//
// 当前策略：
//   1) 先按 binding/preferred_path 做软排序，不直接把它们当作唯一真值；
//   2) 对候选口做一次被动观察，若出现 `CHn:` 文本则判为 Volt32 类设备；
//   3) 对剩余候选口主动发送 WIT Modbus 读寄存器请求；
//   4) 只有收到合法 Modbus 回复的串口，才会被选为 IMU。
//
// 注意：
//   - 本模块只负责“选哪个串口”，不负责长期保活和断连监督；
//   - 长期连接状态仍由 DeviceBinder + ImuDriverWit + nav_daemon_devices 负责。
//
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "nav_core/app/device_binding.hpp"
#include "nav_core/drivers/imu_driver_wit.hpp"

namespace nav_core::app {

/**
 * @brief IMU 串口探测阶段的时间窗口参数。
 *
 * 这些参数只影响“绑定时如何识别设备”，
 * 不影响 IMU 驱动线程的正式采样频率。
 */
struct ImuPortSelectionOptions {
    int passive_baud{115200};
    int passive_observe_ms{150};
    int active_reply_timeout_ms{80};
    int active_probe_attempts{2};
};

/**
 * @brief 在一组串口候选中选出真正的 IMU 端口。
 *
 * 输入：
 *  - `devices` 是一次扫描得到的串口身份列表；
 *  - `imu_cfg` 提供 IMU 侧 slave_addr / baud 等主动探测参数；
 *  - `binding_cfg` / `preferred_path` 只参与排序和诊断，不再硬性决定结果。
 *
 * 返回：
 *  - 找到可确认的 IMU 口时，返回 `CONNECTING + selected_device`；
 *  - 找不到时，返回 `MISMATCH` 或 `DISCONNECTED`，并给出原因摘要。
 */
DeviceProbeDecision select_imu_device_port(
    const std::vector<SerialPortIdentity>& devices,
    const drivers::ImuConfig&              imu_cfg,
    const drivers::SerialBindingConfig&    binding_cfg = {},
    const std::string&                     preferred_path = {},
    ImuPortSelectionOptions                options = {});

/**
 * @brief 把 IMU 端口识别逻辑包装成 DeviceBinder 可注入的选择器。
 *
 * 用途：
 *  - 让 IMU 设备绑定继续复用 DeviceBinder 的状态机；
 *  - 但把“如何在多个候选口中挑出 IMU”交给本模块实现。
 */
DeviceBinder::SelectFn make_imu_port_selector(const drivers::ImuConfig& imu_cfg,
                                              ImuPortSelectionOptions   options = {});

} // namespace nav_core::app
