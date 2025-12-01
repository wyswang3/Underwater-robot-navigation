#include "nav_core/nav_state_publisher.h"

#include <chrono>
#include <cstring>   // std::memset, std::memcpy
#include <iostream>

#ifndef _WIN32
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace nav_core {

namespace {

// 组合成 'N' 'A' 'V' '1'
constexpr std::uint32_t NAV_STATE_MAGIC =
    (static_cast<std::uint32_t>('N') << 24) |
    (static_cast<std::uint32_t>('A') << 16) |
    (static_cast<std::uint32_t>('V') << 8)  |
    (static_cast<std::uint32_t>('1'));

constexpr std::uint32_t NAV_STATE_VERSION = 1;

/// 获取单调时钟 ns（用于控制端判断“有没有在更新”）
inline std::int64_t now_mono_ns()
{
    using Clock = std::chrono::steady_clock;
    const auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

/// 获取墙钟 ns（用于日志/调试，人能看懂的时间基）
inline std::int64_t now_wall_ns()
{
    using Clock = std::chrono::system_clock;
    const auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

} // anonymous namespace

NavStatePublisher::NavStatePublisher()
{
}

NavStatePublisher::~NavStatePublisher()
{
    shutdown();
}

NavStatePublisher::NavStatePublisher(NavStatePublisher&& other) noexcept
{
    *this = std::move(other);
}

NavStatePublisher& NavStatePublisher::operator=(NavStatePublisher&& other) noexcept
{
    if (this == &other) return *this;

    shutdown();

    enabled_     = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;
    shm_name_    = std::move(other.shm_name_);
    shm_size_    = other.shm_size_;

#ifndef _WIN32
    shm_fd_  = other.shm_fd_;
    shm_ptr_ = other.shm_ptr_;
    other.shm_fd_  = -1;
    other.shm_ptr_ = nullptr;
#else
    shm_handle_ = other.shm_handle_;
    shm_ptr_    = other.shm_ptr_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_    = nullptr;
#endif

    other.initialized_ = false;
    other.error_flag_  = false;

    return *this;
}

bool NavStatePublisher::init(const NavStatePublisherConfig& cfg)
{
    // 显式禁用：所有操作退化为 no-op，但返回 true，方便在 nav_daemon 里统一处理
    enabled_ = cfg.enable;
    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

    return init_shm(cfg);
}

void NavStatePublisher::shutdown()
{
    close_shm();
    initialized_ = false;
    error_flag_  = false;
    enabled_     = false;
}

bool NavStatePublisher::publish(const shared::msg::NavState& state)
{
    if (!enabled_) {
        // 调试模式不启用 IPC 时，publish 视为成功
        return true;
    }
    if (!initialized_ || error_flag_ || !shm_ptr_) {
        return false;
    }

    ShmLayout* layout = shm_ptr_;

    // 简单写：先更新 header，再 memcpy state
    // 单生产者场景下，这样的顺序满足控制进程的读取需求。
    layout->hdr.seq       += 1;
    layout->hdr.mono_ns    = now_mono_ns();
    layout->hdr.wall_ns    = now_wall_ns();
    layout->hdr.magic      = NAV_STATE_MAGIC;
    layout->hdr.version    = NAV_STATE_VERSION;

    // NavState 必须是 POD/TriviallyCopyable，这也是 shared/msg/nav_state.hpp 设计时要注意的
    std::memcpy(&layout->state, &state, sizeof(shared::msg::NavState));

    // 若需要更严格的可见性保证，可以在此添加 std::atomic_thread_fence(std::memory_order_release);
    return true;
}

bool NavStatePublisher::init_shm(const NavStatePublisherConfig& cfg)
{
#ifdef _WIN32
    // Windows 暂不实现，直接标记初始化失败或作为 no-op
    std::cerr << "[NavStatePublisher] Shared memory is not implemented on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rov_nav_state_v1" : cfg.shm_name;

    if (shm_name_.empty() || shm_name_[0] != '/') {
        std::cerr << "[NavStatePublisher] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // 计算共享内存大小
    const std::size_t min_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0 || cfg.shm_size < min_size) ? min_size : cfg.shm_size;

    // 打开/创建共享内存对象
    shm_fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[NavStatePublisher] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // 调整大小
    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[NavStatePublisher] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    // 映射到本进程地址空间
    void* addr = ::mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (addr == MAP_FAILED) {
        std::perror("[NavStatePublisher] mmap failed");
        ::close(shm_fd_);
        shm_fd_ = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);

    // 若是首次创建，可以做一次 header 初始化（但不强依赖）
    if (shm_ptr_->hdr.magic != NAV_STATE_MAGIC ||
        shm_ptr_->hdr.version != NAV_STATE_VERSION) {
        std::memset(shm_ptr_, 0, shm_size_);
        shm_ptr_->hdr.magic   = NAV_STATE_MAGIC;
        shm_ptr_->hdr.version = NAV_STATE_VERSION;
        shm_ptr_->hdr.seq     = 0;
        shm_ptr_->hdr.mono_ns = now_mono_ns();
        shm_ptr_->hdr.wall_ns = now_wall_ns();
    }

    initialized_ = true;
    error_flag_  = false;
    return true;
#endif
}

void NavStatePublisher::close_shm()
{
#ifdef _WIN32
    // 暂无实现，留空
    shm_ptr_    = nullptr;
    shm_handle_ = nullptr;
#else
    if (shm_ptr_) {
        ::munmap(static_cast<void*>(shm_ptr_), shm_size_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
#endif
}

} // namespace nav_core
