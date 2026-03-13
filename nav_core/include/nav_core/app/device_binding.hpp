#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "nav_core/core/types.hpp"
#include "nav_core/drivers/serial_binding.hpp"

namespace nav_core::app {

enum class DeviceConnectionState : std::uint8_t {
    DISCONNECTED = 0,
    PROBING = 1,
    CONNECTING = 2,
    ONLINE = 3,
    MISMATCH = 4,
    ERROR_BACKOFF = 5,
    RECONNECTING = 6,
};

/**
 * @brief 扫描到的串口设备身份信息。
 *
 * 约定：
 *  - `path` 是本次扫描暴露给上层尝试连接的路径，可能是 by-id symlink；
 *  - `canonical_path` 是实际 tty 设备路径，便于去重；
 *  - `vendor_id/product_id/serial` 来自 sysfs，缺失时为空字符串。
 */
struct SerialPortIdentity {
    std::string path{};
    std::string canonical_path{};
    std::string vendor_id{};
    std::string product_id{};
    std::string serial{};
};

/**
 * @brief 设备绑定状态快照。
 *
 * 线程边界：
 *  - 设计给 nav_daemon 主线程单线程读写；
 *  - 不在 driver 线程中直接访问。
 */
struct DeviceBindingStatus {
    DeviceConnectionState state{DeviceConnectionState::DISCONNECTED};
    std::string label{};
    std::string active_path{};
    std::string reason{};
    MonoTimeNs  last_transition_ns{0};
    MonoTimeNs  next_action_ns{0};
    SerialPortIdentity active_identity{};
};

std::vector<SerialPortIdentity> scan_serial_port_identities();

bool matches_binding_identity(const SerialPortIdentity&          device,
                              const drivers::SerialBindingConfig& cfg,
                              const std::string&                  preferred_path) noexcept;

class DeviceBinder {
public:
    using DiscoverFn = std::function<std::vector<SerialPortIdentity>()>;

    /**
     * @brief 创建一个主线程使用的串口设备绑定器。
     *
     * 输入：
     *  - `label` 用于日志与诊断；
     *  - `preferred_path` 是 driver config 的主路径，优先尝试 by-id 或自定义 symlink；
     *  - `cfg` 定义候选端口、身份约束和重试参数；
     *  - `discover` 可注入 fake discovery，便于单元测试。
     *
     * 线程边界：
     *  - 只应在 nav 主线程中调用；
     *  - 不做内部加锁。
     */
    DeviceBinder(std::string                        label,
                 std::string                        preferred_path,
                 drivers::SerialBindingConfig       cfg = {},
                 DiscoverFn                         discover = {});

    const DeviceBindingStatus& status() const noexcept { return status_; }

    bool should_probe(MonoTimeNs now_ns) const noexcept;

    /**
     * @brief 执行一次探测并选择连接目标。
     *
     * 返回：
     *  - 有匹配设备时，状态切到 CONNECTING 并返回选中的身份信息；
     *  - 无设备时，状态切到 DISCONNECTED；
     *  - 有设备但身份不匹配或路径歧义时，状态切到 MISMATCH。
     */
    std::optional<SerialPortIdentity> probe(MonoTimeNs now_ns);

    void mark_connect_success(MonoTimeNs now_ns, const SerialPortIdentity& device);
    void mark_connect_failure(MonoTimeNs now_ns, const std::string& reason);
    void mark_disconnected(MonoTimeNs now_ns, const std::string& reason);

private:
    void transition_(MonoTimeNs               now_ns,
                     DeviceConnectionState    next_state,
                     std::string              reason,
                     std::optional<SerialPortIdentity> identity = std::nullopt);

    std::vector<SerialPortIdentity> discover_() const;

    std::string                   preferred_path_{};
    drivers::SerialBindingConfig  cfg_{};
    DiscoverFn                    discover_fn_{};
    DeviceBindingStatus           status_{};
};

const char* device_state_name(DeviceConnectionState state) noexcept;

} // namespace nav_core::app
