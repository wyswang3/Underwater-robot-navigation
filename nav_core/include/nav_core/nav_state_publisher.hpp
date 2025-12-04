#pragma once

/**
 * @file   nav_state_publisher.hpp
 * @brief  导航状态共享内存发布器（OrangePi 导航进程 → 控制进程）
 */

#include <cstddef>
#include <cstdint>
#include <string>

#include <nav_core/types.hpp>

// 这里只需要前向声明，不需要完整定义
namespace shared {
namespace msg {
struct NavState;
} // namespace msg
} // namespace shared

namespace nav_core {

/**
 * @brief 导航状态发布配置
 */
struct NavStatePublisherConfig {
    /// 是否启用发布（false 时所有函数退化为 no-op，便于调试）
    bool enable{true};

    /// POSIX 共享内存名称（必须以 '/' 开头，不能含其它 '/'; 进程间需保持一致）
    std::string shm_name{"/rov_nav_state_v1"};

    /// 显式 shm 大小（字节）。0 表示使用内部计算的 sizeof(ShmLayout)。
    std::size_t shm_size{0};
};

/**
 * @brief 导航状态共享内存发布器
 */
class NavStatePublisher {
public:
    NavStatePublisher() = default;
    ~NavStatePublisher() noexcept;

    NavStatePublisher(const NavStatePublisher&)            = delete;
    NavStatePublisher& operator=(const NavStatePublisher&) = delete;

    NavStatePublisher(NavStatePublisher&& other) noexcept;
    NavStatePublisher& operator=(NavStatePublisher&& other) noexcept;

    /// 初始化共享内存
    bool init(const NavStatePublisherConfig& cfg);

    /// 释放共享内存资源
    void shutdown() noexcept;

    /// 发布一帧导航状态
    bool publish(const shared::msg::NavState& state);

    /// 是否已成功初始化并处于正常工作状态
    bool ok() const noexcept { return initialized_ && !error_flag_; }

    /// 最近一次 publish 是否出现持久错误（例如 shm 被删除）
    bool hasError() const noexcept { return error_flag_; }

private:
    struct ShmHeader {
        std::uint32_t magic;    // 'NAV1'
        std::uint32_t version;  // 当前版本 1
        std::uint64_t seq;      // 序号
        MonoTimeNs    mono_ns;  // 单调时钟（ns）
        SysTimeNs     wall_ns;  // 墙钟时间（ns）
    };

    struct ShmLayout;           // 在 cpp 里给出完整定义

    bool init_shm(const NavStatePublisherConfig& cfg);
    void close_shm() noexcept;

private:
    bool        enabled_{false};
    bool        initialized_{false};
    bool        error_flag_{false};

    std::string shm_name_;
    std::size_t shm_size_{0};

#ifdef _WIN32
    void*      shm_handle_{nullptr};
    ShmLayout* shm_ptr_{nullptr};
#else
    int        shm_fd_{-1};
    ShmLayout* shm_ptr_{nullptr};
#endif
};

} // namespace nav_core
