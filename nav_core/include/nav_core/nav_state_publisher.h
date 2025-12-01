#pragma once

/**
 * @file   nav_state_publisher.h
 * @brief  导航状态共享内存发布器（OrangePi 导航进程 → 控制进程）
 *
 * 设计目标：
 *   - 由导航守护进程 uwnav_navd 周期性写入最新 NavState；
 *   - 控制进程（香橙派上的 pwm_control_program 等）以只读方式 mmap；
 *   - 单生产者 / 多消费者，简单、低延迟、语言无关（C/C++/Python 都可读）。
 *
 * 当前实现：
 *   - Linux/OrangePi：基于 POSIX shared memory (shm_open + mmap)
 *   - Windows：退化为 no-op（publish() 始终返回 true，但不做任何事情）
 *
 * 共享内存布局：
 *   struct ShmLayout {
 *       uint32_t magic;          // 'N','A','V','1'
 *       uint32_t version;        // 协议版本，当前 = 1
 *       uint64_t seq;            // 序号，每次 publish 自增
 *       int64_t  mono_ns;        // 写入时的单调时钟（ns）
 *       int64_t  wall_ns;        // 写入时的墙钟时间（ns）
 *       NavState state;          // 实际导航状态（shared/msg/nav_state.hpp）
 *   };
 */

#include <cstdint>
#include <string>

#include "shared/msg/nav_state.hpp"  // D:\UnderwaterRobotSystem\shared 下的公共消息定义

namespace nav_core {

/**
 * @brief 导航状态发布配置
 */
struct NavStatePublisherConfig {
    /// 是否启用发布（false 时所有函数退化为 no-op，便于调试）
    bool enable = true;

    /// POSIX 共享内存名称（必须以 '/' 开头，不能含其它 '/'; 进程间需保持一致）
    std::string shm_name = "/rov_nav_state_v1";

    /// 显式 shm 大小（字节）。0 表示使用内部计算的 sizeof(ShmLayout)。
    std::size_t shm_size = 0;
};

/**
 * @brief 导航状态共享内存发布器
 *
 * 生命周期：
 *   - 构造后处于未初始化状态；
 *   - 调用 init(cfg) 完成 shm_open/ftruncate/mmap；
 *   - 周期性调用 publish(nav_state) 写入；
 *   - 程序退出前调用 shutdown()，或依赖析构自动清理。
 */
class NavStatePublisher {
public:
    NavStatePublisher();
    ~NavStatePublisher();

    /// 禁止拷贝，仅允许单实例或显式管理
    NavStatePublisher(const NavStatePublisher&) = delete;
    NavStatePublisher& operator=(const NavStatePublisher&) = delete;

    /// 允许移动（可选）
    NavStatePublisher(NavStatePublisher&& other) noexcept;
    NavStatePublisher& operator=(NavStatePublisher&& other) noexcept;

    /**
     * @brief 初始化共享内存
     *
     * @return true 初始化成功或显式禁用（cfg.enable=false）
     *         false 初始化失败（shm_open/mmap 等出错）
     */
    bool init(const NavStatePublisherConfig& cfg);

    /**
     * @brief 释放共享内存（关闭 fd，munmap）
     */
    void shutdown();

    /**
     * @brief 发布一帧导航状态
     *
     * @param state 最新的导航状态（通常来自 ESKF）
     *
     * 语义：
     *   - 若未启用（cfg.enable=false），直接返回 true（无副作用）；
     *   - 若 init 失败/未调用，则返回 false；
     *   - 正常情况下，写入共享内存并自增 seq；
     */
    bool publish(const shared::msg::NavState& state);

    /// 是否已成功初始化并处于正常工作状态
    bool ok() const { return initialized_ && !error_flag_; }

    /// 最近一次 publish 是否出现持久错误（例如 shm 被删除）
    bool hasError() const { return error_flag_; }

private:
    struct ShmHeader {
        std::uint32_t magic;      // 'NAV1'
        std::uint32_t version;    // 当前版本 1
        std::uint64_t seq;        // 序号
        std::int64_t  mono_ns;    // 单调时钟
        std::int64_t  wall_ns;    // 墙钟时间
    };

    struct ShmLayout {
        ShmHeader            hdr;
        shared::msg::NavState state;
    };

    bool init_shm(const NavStatePublisherConfig& cfg);
    void close_shm();

    bool          enabled_      = false;
    bool          initialized_  = false;
    bool          error_flag_   = false;
    std::string   shm_name_;
    std::size_t   shm_size_     = 0;

#ifdef _WIN32
    // Windows 下暂不实现共享内存，留空
    void*         shm_handle_   = nullptr;
    ShmLayout*    shm_ptr_      = nullptr;
#else
    int           shm_fd_       = -1;
    ShmLayout*    shm_ptr_      = nullptr;
#endif
};

} // namespace nav_core
