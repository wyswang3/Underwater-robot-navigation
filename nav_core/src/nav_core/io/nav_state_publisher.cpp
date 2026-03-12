// nav_core/src/io/nav_state_publisher.cpp
//
// 导航状态共享内存发布器实现（POSIX shm + seqlock）
//
// 布局：
//   struct ShmLayout {
//       ShmHeader             header;   // magic / version / timestamps / seq
//       shared::msg::NavState nav;      // 最新的导航状态快照
//   }
//
// 写端（本类）：
//   publish(state):
//     seq += 1 (odd, 写入中)
//     写 mono_ns / wall_ns
//     拷贝 NavState
//     seq += 1 (even, 稳定)
//
// 读端（控制进程）：
//   do {
//       seq1 = header.seq (atomic, acquire)
//       读 header + nav
//       seq2 = header.seq (atomic, acquire)
//   } while (seq1 != seq2 || (seq1 & 1) != 0);
//
//   若 seq 为偶数且两次相同，则 nav 是一致快照。

#include "nav_core/io/nav_state_publisher.hpp"

#include <algorithm>
#include <cstring>   // std::memset, std::memcpy
#include <utility>   // std::swap

#include "shared/msg/nav_state.hpp"
#include "shared/shm/nav_state_shm.hpp"

#ifndef _WIN32
#  include <fcntl.h>      // shm_open
#  include <sys/mman.h>   // mmap, munmap
#  include <sys/stat.h>   // S_IRWXU, etc.
#  include <unistd.h>     // ftruncate, close
#  include <time.h>       // clock_gettime
#else
#  include <windows.h>
#  include <chrono>
#endif

namespace nav_core::io {

// ============================ 构造 / 析构 / 移动 ============================

NavStatePublisher::~NavStatePublisher() noexcept
{
    shutdown();
}

NavStatePublisher::NavStatePublisher(NavStatePublisher&& other) noexcept
{
    *this = std::move(other);
}

NavStatePublisher& NavStatePublisher::operator=(NavStatePublisher&& other) noexcept
{
    if (this == &other) {
        return *this;
    }

    // 先释放自身资源
    shutdown();

    enabled_     = other.enabled_;
    initialized_ = other.initialized_;
    error_flag_  = other.error_flag_;

    shm_name_    = std::move(other.shm_name_);
    shm_size_    = other.shm_size_;

#ifdef _WIN32
    shm_handle_  = other.shm_handle_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_handle_ = nullptr;
    other.shm_ptr_    = nullptr;
#else
    shm_fd_      = other.shm_fd_;
    shm_ptr_     = other.shm_ptr_;
    other.shm_fd_  = -1;
    other.shm_ptr_ = nullptr;
#endif

    other.enabled_     = false;
    other.initialized_ = false;
    // error_flag_ 留给对方原值即可（移动后其他对象不再使用）

    return *this;
}

// ============================ 公共接口实现 ============================

bool NavStatePublisher::init(const NavStatePublisherConfig& cfg)
{
    // 1) 配置层开关：关闭时直接进入“禁用模式”
    enabled_     = cfg.enable;
    error_flag_  = false;
    initialized_ = false;

    if (!enabled_) {
        // 禁用模式：不做任何 shm 操作，但视作 init 成功，publish() 将 no-op
        shm_name_.clear();
        shm_size_ = 0;
#ifdef _WIN32
        shm_handle_ = nullptr;
        shm_ptr_    = nullptr;
#else
        shm_fd_     = -1;
        shm_ptr_    = nullptr;
#endif
        initialized_ = true;
        return true;
    }

    // 2) 启用模式：尝试初始化共享内存
    if (!init_shm(cfg)) {
        error_flag_  = true;
        initialized_ = false;
        return false;
    }

    initialized_ = true;
    return true;
}

void NavStatePublisher::shutdown() noexcept
{
    // 即使 init 尚未成功，多次调用也安全
    close_shm();
    initialized_ = false;
}

bool NavStatePublisher::publish(const shared::msg::NavState& state)
{
    // 1) 快速检查：未启用或未初始化时 no-op
    if (!enabled_ || !initialized_) {
        return false;
    }
    if (error_flag_) {
        return false;
    }
    if (!shm_ptr_) {
        error_flag_ = true;
        return false;
    }

    ShmLayout* layout = shm_ptr_;
    ShmHeader& hdr    = layout->hdr;

    // 2) 获取当前时间（mono_ns / wall_ns）
    MonoTimeNs mono_ns = 0;
    SysTimeNs  wall_ns = 0;

#ifndef _WIN32
    {
        timespec ts_mono{};
        timespec ts_wall{};

        if (clock_gettime(CLOCK_MONOTONIC, &ts_mono) == 0) {
            mono_ns = static_cast<MonoTimeNs>(ts_mono.tv_sec) * 1000000000LL
                    + static_cast<MonoTimeNs>(ts_mono.tv_nsec);
        }

        if (clock_gettime(CLOCK_REALTIME, &ts_wall) == 0) {
            wall_ns = static_cast<SysTimeNs>(ts_wall.tv_sec) * 1000000000LL
                    + static_cast<SysTimeNs>(ts_wall.tv_nsec);
        }
    }
#else
    {
        using namespace std::chrono;
        const auto now_steady = steady_clock::now().time_since_epoch();
        const auto now_sys    = system_clock::now().time_since_epoch();

        mono_ns = static_cast<MonoTimeNs>(
            duration_cast<nanoseconds>(now_steady).count()
        );
        wall_ns = static_cast<SysTimeNs>(
            duration_cast<nanoseconds>(now_sys).count()
        );
    }
#endif

    // 3) seqlock 写入：seq = odd → 写 header+nav → seq = even
    //
    // 使用 fetch_add(1) 确保跨进程可见性；
    // memory_order：
    //   - 写入前：acq_rel，避免乱序；
    //   - 写入后：release，使 nav 内容对读端可见。
    hdr.seq.fetch_add(1, std::memory_order_acq_rel); // 变为奇数：写入中

    // 写入时间戳
    hdr.mono_ns = mono_ns;
    hdr.wall_ns = wall_ns;

    // 拷贝 NavState（trivially copyable，可用 memcpy）
    std::memcpy(&layout->payload, &state, sizeof(shared::msg::NavState));

    hdr.seq.fetch_add(1, std::memory_order_release); // 变为偶数：稳定快照

    return true;
}

// ============================ 内部 shm 实现 ============================

bool NavStatePublisher::init_shm(const NavStatePublisherConfig& cfg)
{
    shm_name_ = cfg.shm_name;
    if (shm_name_.empty()) {
        // 名字不合法
        return false;
    }

    // 计算共享内存大小：若未指定，则使用完整布局大小
    const std::size_t layout_size = sizeof(ShmLayout);
    shm_size_ = (cfg.shm_size == 0) ? layout_size
                                    : std::max(cfg.shm_size, layout_size);

#ifndef _WIN32
    // ---------- POSIX 分支 ----------
    // 打开或创建共享内存对象
    int fd = ::shm_open(shm_name_.c_str(), O_CREAT | O_RDWR, 0666);
    if (fd < 0) {
        return false;
    }

    // 设置大小
    if (::ftruncate(fd, static_cast<off_t>(shm_size_)) != 0) {
        ::close(fd);
        return false;
    }

    // 映射到本进程地址空间
    void* addr = ::mmap(nullptr, shm_size_,
                        PROT_READ | PROT_WRITE,
                        MAP_SHARED,
                        fd, 0);
    if (addr == MAP_FAILED) {
        ::close(fd);
        return false;
    }

    shm_fd_  = fd;
    shm_ptr_ = static_cast<ShmLayout*>(addr);

    // 初始化整个映射区，随后写入 canonical header metadata。
    std::memset(static_cast<void*>(shm_ptr_), 0, shm_size_);

    ShmHeader& hdr = shm_ptr_->hdr;
    hdr.magic         = shared::shm::kNavStateMagic;
    hdr.layout_ver    = shared::shm::kNavStateLayoutVersion;
    hdr.payload_ver   = shared::shm::kNavStatePayloadVersion;
    hdr.payload_size  = static_cast<std::uint32_t>(sizeof(shared::msg::NavState));
    hdr.payload_align = static_cast<std::uint32_t>(alignof(shared::msg::NavState));
    hdr.seq.store(0, std::memory_order_relaxed);
    hdr.mono_ns = 0;
    hdr.wall_ns = 0;

    return true;

#else
    // ---------- Windows 分支 ----------
    // 使用 CreateFileMapping + MapViewOfFile 实现类似 shm
    const std::string mapped_name = shm_name_;

    HANDLE hMap = ::CreateFileMappingA(
        INVALID_HANDLE_VALUE,
        nullptr,
        PAGE_READWRITE,
        0,
        static_cast<DWORD>(shm_size_),
        mapped_name.c_str()
    );
    if (!hMap) {
        return false;
    }

    void* addr = ::MapViewOfFile(
        hMap,
        FILE_MAP_ALL_ACCESS,
        0, 0,
        shm_size_
    );
    if (!addr) {
        ::CloseHandle(hMap);
        return false;
    }

    shm_handle_ = hMap;
    shm_ptr_    = static_cast<ShmLayout*>(addr);

    std::memset(static_cast<void*>(shm_ptr_), 0, shm_size_);

    ShmHeader& hdr = shm_ptr_->hdr;
    hdr.magic         = shared::shm::kNavStateMagic;
    hdr.layout_ver    = shared::shm::kNavStateLayoutVersion;
    hdr.payload_ver   = shared::shm::kNavStatePayloadVersion;
    hdr.payload_size  = static_cast<std::uint32_t>(sizeof(shared::msg::NavState));
    hdr.payload_align = static_cast<std::uint32_t>(alignof(shared::msg::NavState));
    hdr.seq.store(0, std::memory_order_relaxed);
    hdr.mono_ns = 0;
    hdr.wall_ns = 0;

    return true;
#endif
}

void NavStatePublisher::close_shm() noexcept
{
#ifndef _WIN32
    if (shm_ptr_) {
        ::munmap(static_cast<void*>(shm_ptr_), shm_size_);
        shm_ptr_ = nullptr;
    }
    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }
#else
    if (shm_ptr_) {
        ::UnmapViewOfFile(static_cast<void*>(shm_ptr_));
        shm_ptr_ = nullptr;
    }
    if (shm_handle_) {
        ::CloseHandle(shm_handle_);
        shm_handle_ = nullptr;
    }
#endif
    shm_size_ = 0;
}

} // namespace nav_core::io
