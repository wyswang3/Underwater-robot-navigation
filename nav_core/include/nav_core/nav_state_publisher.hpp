#pragma once
/**
 * @file   nav_state_publisher.hpp
 * @brief  导航状态共享内存发布器（OrangePi 导航进程 → 控制进程）
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <atomic>   // 新增：必须

#include <nav_core/types.hpp>

// 这里只需要前向声明，不需要完整定义
namespace shared {
namespace msg {
struct NavState;
} // namespace msg
} // namespace shared

namespace nav_core {

struct NavStatePublisherConfig {
    bool enable{true};
    std::string shm_name{"/rov_nav_state_v1"};
    std::size_t shm_size{0};
};

class NavStatePublisher {
public:
    NavStatePublisher() = default;
    ~NavStatePublisher() noexcept;

    NavStatePublisher(const NavStatePublisher&)            = delete;
    NavStatePublisher& operator=(const NavStatePublisher&) = delete;

    NavStatePublisher(NavStatePublisher&& other) noexcept;
    NavStatePublisher& operator=(NavStatePublisher&& other) noexcept;

    bool init(const NavStatePublisherConfig& cfg);
    void shutdown() noexcept;
    bool publish(const shared::msg::NavState& state);

    bool ok() const noexcept { return initialized_ && !error_flag_; }
    bool hasError() const noexcept { return error_flag_; }

private:
    struct ShmHeader {
        std::uint32_t magic   = 0;  // 'NAV1'
        std::uint32_t version = 0;  // 当前版本 1

        MonoTimeNs mono_ns = 0;     // 单调时钟（ns）
        SysTimeNs  wall_ns = 0;     // 墙钟时间（ns）

        // seqlock: odd=writing, even=stable
        alignas(8) std::atomic<std::uint64_t> seq{0};
    };

    struct ShmLayout; // cpp 里给出完整定义

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
