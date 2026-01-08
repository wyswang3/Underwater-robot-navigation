// nav_core/include/nav_core/status.hpp
//
// @file status.hpp
// @brief nav_core 内部通用“状态码 + 文本信息”返回类型。
//
// 用途：
//  - 为各个模块（drivers / filters / estimator / io / app）提供统一的错误返回机制；
//  - 代替裸 int / bool 返回值，使调用方可以同时拿到：
//      * 状态类别（StatusCode）
//      * 可读错误信息（std::string）
//  - 适合函数签名：Status foo(...);  调用方通过 status.ok() 判断是否成功。
//    若需要返回值，可采用 pair/struct 或在调用方传入输出引用。
//
// 设计原则：
//  - 轻量、无异常（不抛出）；
//  - header-only，无需对应 cpp 文件；
//  - 不依赖 nav_core 其他模块，可被任何子模块直接使用。

#pragma once

#include <string>
#include <utility>  // for std::move

namespace nav_core::core {

/// @brief 通用状态码枚举。
///
/// 说明：
///  - 这些状态码覆盖 nav_core 常见错误场景；
///  - 可以根据项目演进增量扩展，但应尽量保持语义稳定。
enum class StatusCode {
    kOk = 0,           ///< 成功，无错误。
    kInvalidArgument,  ///< 调用参数非法（范围、格式、空指针等）。
    kIoError,          ///< IO 错误（串口/文件/网络读写失败等）。
    kTimeout,          ///< 操作超时（等待设备响应或数据超时）。
    kConfigError,      ///< 配置错误（缺失、格式错误、字段不合法等）。
    kInternalError,    ///< 内部错误（逻辑错误、未预期状态等）。
};

/// @brief 轻量状态类：封装 StatusCode + 文本消息。
///
/// 用法示例：
/// @code
/// nav_core::core::Status init_result = driver.init(cfg);
/// if (!init_result.ok()) {
///     std::cerr << "DVL init failed: " << init_result.message() << std::endl;
///     return init_result;
/// }
/// @endcode
///
/// 设计要点：
///  - 默认构造为 Ok；
///  - 支持静态工厂方法：Status::Ok() / Status::InvalidArgument(...) 等；
///  - 不抛异常，所有信息通过返回值显式传递；
///  - 可作为函数返回值在整个 nav_core 中统一使用。
class Status {
public:
    /// @brief 默认构造：表示成功（kOk）。
    Status() : code_(StatusCode::kOk) {}

    /// @brief 使用状态码 + 文本消息构造。
    Status(StatusCode code, std::string msg)
        : code_(code), msg_(std::move(msg)) {}

    /// @brief 构造一个成功状态。
    static Status Ok() { return Status(); }

    /// @brief 是否为成功状态。
    bool ok() const { return code_ == StatusCode::kOk; }

    /// @brief 便捷转换：if (status) 等价于 if (status.ok())。
    explicit operator bool() const { return ok(); }

    /// @brief 获取状态码。
    StatusCode code() const { return code_; }

    /// @brief 获取错误消息说明。
    const std::string& message() const { return msg_; }

    // ---------- 便捷工厂函数（可选使用） ----------

    static Status InvalidArgument(std::string msg) {
        return Status(StatusCode::kInvalidArgument, std::move(msg));
    }

    static Status IoError(std::string msg) {
        return Status(StatusCode::kIoError, std::move(msg));
    }

    static Status Timeout(std::string msg) {
        return Status(StatusCode::kTimeout, std::move(msg));
    }

    static Status ConfigError(std::string msg) {
        return Status(StatusCode::kConfigError, std::move(msg));
    }

    static Status InternalError(std::string msg) {
        return Status(StatusCode::kInternalError, std::move(msg));
    }

private:
    StatusCode  code_;
    std::string msg_;
};

}  // namespace nav_core::core
