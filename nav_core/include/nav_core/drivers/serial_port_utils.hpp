#pragma once

#include <string>

namespace nav_core::drivers {

bool configure_serial_port_raw(int fd,
                               int baud,
                               int read_timeout_ds,
                               std::string* error = nullptr);

int open_serial_port_raw(const std::string& path,
                         int baud,
                         int read_timeout_ds,
                         std::string* error = nullptr);

} // namespace nav_core::drivers
