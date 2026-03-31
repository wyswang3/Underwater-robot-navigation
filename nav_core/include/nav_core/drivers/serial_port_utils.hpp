// nav_core/include/nav_core/drivers/serial_port_utils.hpp
//
// @file  serial_port_utils.hpp
// @brief POSIX 串口打开与 raw 配置的公共工具。
//
// 角色：
//   - 给 IMU/DVL/串口探测模块提供统一的 termios raw 配置入口；
//   - 保证关键参数一致，例如 8N1、关闭流控、设置 VMIN/VTIME；
//   - 避免各个 driver/tool 各自复制一套 termios 代码后出现细节漂移。
//
// 当前主要用途：
//   - 修正 IMU C++ 驱动与 Python 行为不一致的问题；
//   - 供端口自动探测阶段临时打开串口做被动观察和主动探测。
//
#pragma once

#include <string>

namespace nav_core::drivers {

/**
 * @brief 把一个已打开的 fd 配置成 raw 串口模式。
 *
 * 约定：
 *  - 使用 8N1；
 *  - 关闭软件/硬件流控；
 *  - `read_timeout_ds` 以 0.1s 为单位写入 `VTIME`。
 */
bool configure_serial_port_raw(int fd,
                               int baud,
                               int read_timeout_ds,
                               std::string* error = nullptr);

/**
 * @brief 打开串口路径并立即应用统一的 raw 配置。
 *
 * 返回：
 *  - 成功时返回有效 fd；
 *  - 失败时返回 `-1`，并把错误摘要写入 `error`。
 */
int open_serial_port_raw(const std::string& path,
                         int baud,
                         int read_timeout_ds,
                         std::string* error = nullptr);

} // namespace nav_core::drivers
