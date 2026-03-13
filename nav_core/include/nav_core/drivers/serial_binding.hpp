#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace nav_core::drivers {

/**
 * @brief 串口设备绑定配置。
 *
 * 语义：
 *  - `port` 仍由各 driver config 自己保存，作为首选稳定路径；
 *  - `candidate_paths` 作为备选路径列表，可包含自定义 udev symlink；
 *  - `expected_by_id_substring` / `expected_vid` / `expected_pid` / `expected_serial`
 *    至少命中其一时，才允许把扫描到的设备当作目标设备；
 *  - `offline_timeout_ms` 用于监督“驱动线程虽然还在，但设备其实已经不再出数”；
 *  - `reconnect_backoff_ms` 用于控制重探测节奏，避免忙轮询。
 */
struct SerialBindingConfig {
    std::vector<std::string> candidate_paths{};
    std::string expected_by_id_substring{};
    std::string expected_vid{};
    std::string expected_pid{};
    std::string expected_serial{};

    std::uint32_t offline_timeout_ms{1000};
    std::uint32_t reconnect_backoff_ms{500};
};

} // namespace nav_core::drivers
