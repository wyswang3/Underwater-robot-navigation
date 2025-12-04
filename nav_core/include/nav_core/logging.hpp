// include/nav_core/logging.hpp
#pragma once

#include <iostream>
#include <string_view>

namespace uwnav::core {

inline void log_info(std::string_view msg) {
    std::cerr << "[INFO ] " << msg << '\n';
}
inline void log_warn(std::string_view msg) {
    std::cerr << "[WARN ] " << msg << '\n';
}
inline void log_error(std::string_view msg) {
    std::cerr << "[ERROR] " << msg << '\n';
}

}  // namespace uwnav::core

#define UWNAV_LOG_INFO(msg)  ::uwnav::core::log_info(msg)
#define UWNAV_LOG_WARN(msg)  ::uwnav::core::log_warn(msg)
#define UWNAV_LOG_ERROR(msg) ::uwnav::core::log_error(msg)
