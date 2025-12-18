// nav_core/src/nav_state_publisher.cpp

#include <nav_core/nav_state_publisher.hpp>
#include "shared/msg/nav_state.hpp"  // 完整定义 NavState，只在实现文件里用

#include <chrono>
#include <cstring>
#include <iostream>

#ifndef _WIN32
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
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

/// 获取单调时钟 ns（steady_clock）
inline std::int64_t now_mono_ns()
{
    using Clock = std::chrono::steady_clock;
    const auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

/// 获取墙钟 ns（system_clock）
inline std::int64_t now_wall_ns()
{
    using Clock = std::chrono::system_clock;
    const auto tp = Clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(tp).count();
}

} // anonymous namespace

// 在实现文件里给出 ShmLayout 的完整定义
struct NavStatePublisher::ShmLayout {
    ShmHeader             hdr;
    shared::msg::NavState state;
};

// ============================================================================
//                         生命周期与移动语义
// ============================================================================

NavStatePublisher::~NavStatePublisher() noexcept
{
    shutdown();
}

NavStatePublisher::NavStatePublisher(NavStatePublisher&& other) noexcept
{
    *this = std::move(other);
}

NavStatePublisher&
NavStatePublisher::operator=(NavStatePublisher&& other) noexcept
{
    if (this == &other) {
        return *this;
    }

    shutdown();

    enabled_     = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;
    shm_name_    = std::move(other.shm_name_);
    shm_size_    = other.shm_size_;

#ifndef _WIN32
    shm_fd_      = other.shm_fd_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_fd_  = -1;
    other.shm_ptr_ = nullptr;
#else
    shm_handle_  = other.shm_handle_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_    = nullptr;
#endif

    other.enabled_     = false;
    other.initialized_ = false;
    other.error_flag_  = false;

    return *this;
}

// ============================================================================
//                         公共接口
// ============================================================================

bool NavStatePublisher::init(const NavStatePublisherConfig& cfg)
{
    enabled_ = cfg.enable;
    if (!enabled_) {
        initialized_ = false;
        error_flag_  = false;
        return true;
    }

    return init_shm(cfg);
}

void NavStatePublisher::shutdown() noexcept
{
    close_shm();
    enabled_     = false;
    initialized_ = false;
    error_flag_  = false;
}

bool NavStatePublisher::publish(const shared::msg::NavState& state)
{
    if (!enabled_) return true;
    if (!initialized_ || error_flag_ || !shm_ptr_) return false;

    ShmLayout* layout = shm_ptr_;
    auto& seq = layout->hdr.seq;

    // 置为奇数：写入开始
    std::uint64_t old = seq.load(std::memory_order_relaxed);
    std::uint64_t start = (old & 1u) ? (old + 2u) : (old + 1u);
    seq.store(start, std::memory_order_relaxed);

    layout->hdr.mono_ns = now_mono_ns();
    layout->hdr.wall_ns = now_wall_ns();
    layout->hdr.magic   = NAV_STATE_MAGIC;
    layout->hdr.version = NAV_STATE_VERSION;

    std::memcpy(&layout->state, &state, sizeof(shared::msg::NavState));

    // 发布完成：release，保证 state/header 对读端可见
    seq.store(start + 1u, std::memory_order_release);
    return true;
}


// ============================================================================
//                         内部：共享内存初始化
// ============================================================================

bool NavStatePublisher::init_shm(const NavStatePublisherConfig& cfg)
{
#ifdef _WIN32
    std::cerr << "[NavStatePublisher] Shared memory publisher "
                 "is not implemented on Windows.\n";
    initialized_ = false;
    error_flag_  = true;
    return false;
#else
    shm_name_ = cfg.shm_name.empty() ? "/rov_nav_state_v1" : cfg.shm_name;

    if (shm_name_.empty() || shm_name_.front() != '/') {
        std::cerr << "[NavStatePublisher] Invalid shm_name: " << shm_name_
                  << " (must start with '/')\n";
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    const std::size_t min_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0 || cfg.shm_size < min_size)
                ? min_size
                : cfg.shm_size;

    shm_fd_ = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (shm_fd_ < 0) {
        std::perror("[NavStatePublisher] shm_open failed");
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    if (::ftruncate(shm_fd_, static_cast<off_t>(shm_size_)) != 0) {
        std::perror("[NavStatePublisher] ftruncate failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    void* addr = ::mmap(nullptr,
                        shm_size_,
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED,
                        shm_fd_,
                        0);
    if (addr == MAP_FAILED) {
        std::perror("[NavStatePublisher] mmap failed");
        ::close(shm_fd_);
        shm_fd_      = -1;
        initialized_ = false;
        error_flag_  = true;
        return false;
    }

    shm_ptr_ = static_cast<ShmLayout*>(addr);

    if (shm_ptr_->hdr.magic   != NAV_STATE_MAGIC ||
        shm_ptr_->hdr.version != NAV_STATE_VERSION)
    {
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

// ============================================================================
//                         内部：共享内存关闭
// ============================================================================

void NavStatePublisher::close_shm() noexcept
{
#ifdef _WIN32
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
