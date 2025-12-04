// include/nav_core/status.hpp
#pragma once

#include <string>

namespace uwnav::core {

enum class StatusCode {
    kOk = 0,
    kInvalidArgument,
    kIoError,
    kTimeout,
    kConfigError,
    kInternalError,
};

class Status {
public:
    Status() : code_(StatusCode::kOk) {}
    Status(StatusCode code, std::string msg)
        : code_(code), msg_(std::move(msg)) {}

    static Status Ok() { return Status(); }

    bool ok() const { return code_ == StatusCode::kOk; }
    StatusCode code() const { return code_; }
    const std::string& message() const { return msg_; }

private:
    StatusCode  code_;
    std::string msg_;
};

}  // namespace uwnav::core
