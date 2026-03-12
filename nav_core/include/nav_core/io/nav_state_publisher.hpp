// nav_core/include/nav_core/io/nav_state_publisher.hpp
//
// @file   nav_state_publisher.hpp
// @brief  导航状态共享内存发布器
//
// 职责：
//   - 将在线导航解算得到的 shared::msg::NavState 写入一块共享内存，
//     供控制进程 / 其他进程以零拷贝方式读取；
//   - 统一在 OrangePi 上作为「导航进程 → 控制进程」的 NavState 出口；
//   - 使用 seqlock 方案：写端递增 seq，读端在 seq 稳定时拷贝数据，
//     保证读到的是一个内部一致的 NavState 快照；
//   - 同时在 ShmHeader 中写入当前 MonoTimeNs / SysTimeNs 作为元信息，
//     便于控制侧估计“观测延迟”和做时间对齐。
//
// 设计原则：
//   - 不依赖非标准库，只用 POSIX shm（或 Windows 对等实现）+ <atomic>；
//   - 头文件只暴露配置与接口，具体 shm 打开/映射逻辑放在 .cpp 中；
//   - 保持与旧版 ABI 兼容：
//       * NavStatePublisherConfig 字段不删、接口名不改（init/shutdown/publish）
//       * ShmHeader 字段保持类型与顺序不变（magic/version/mono_ns/wall_ns/seq）。
//

#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include <nav_core/core/types.hpp>  // MonoTimeNs / SysTimeNs

#include "shared/shm/nav_state_shm.hpp"

// 如果你暂时不想引入 io 命名空间，也可以保留 namespace nav_core，
// 但从架构上更推荐放在 nav_core::io 下。
namespace nav_core::io {

/// @brief NavState 共享内存发布器的配置。
///
/// 约定：
///   - enable == false 时，init() 会直接返回 true，但 publish() 为 no-op；
///   - shm_name 建议以 "/" 开头，便于在不同进程之间按名字打开；
///   - shm_size 若为 0，则在 init() 内部使用 sizeof(ShmLayout) 自动计算。
struct NavStatePublisherConfig {
    bool        enable{true};                 ///< 是否启用共享内存发布
    std::string shm_name{"/rov_nav_state_v1"};///< SHM 名称（跨进程约定）
    std::size_t shm_size{0};                  ///< 总大小；0=由实现自动推导
};

class NavStatePublisher {
public:
    NavStatePublisher() = default;
    ~NavStatePublisher() noexcept;

    NavStatePublisher(const NavStatePublisher&)            = delete;
    NavStatePublisher& operator=(const NavStatePublisher&) = delete;

    NavStatePublisher(NavStatePublisher&& other) noexcept;
    NavStatePublisher& operator=(NavStatePublisher&& other) noexcept;

    /**
     * @brief 初始化共享内存发布器。
     *
     * 典型流程：
     *   - 若 cfg.enable == false，则标记为“禁用模式”，init 返回 true；
     *   - 打开或创建名为 cfg.shm_name 的共享内存；
     *   - 将其映射到本进程地址空间，布局为 ShmLayout；
     *   - 初始化 ShmHeader（magic/version/时间戳/seq 等）。
     *
     * @return true  初始化成功，可以调用 publish()
     * @return false 初始化失败（shm 打开/映射出错等），hasError() 为 true
     */
    bool init(const NavStatePublisherConfig& cfg);

    /**
     * @brief 关闭共享内存并释放资源。
     *
     * 可重复调用；析构函数内部也会调用一次。
     */
    void shutdown() noexcept;

    /**
     * @brief 将一帧 NavState 写入共享内存。
     *
     * 语义：
     *   - 若 config.enable == false 或未成功 init，则返回 false（no-op）； 
     *   - 写入流程（seqlock）大致为：
     *       1) seq 自增为奇数（标记“写入中”）；
     *       2) 写入 ShmHeader::mono_ns / wall_ns（可选）；
     *       3) 拷贝 NavState 到共享体；
///       4) seq 自增为偶数（标记“稳定快照”）。
     *   - 读端只在 seq 读取为偶数且两次读取相同的情况下，认为快照有效。
     *
     * @param state 当前估计的导航状态（shared::msg::NavState）
     * @return true  发布成功（共享内存处于“已写入”的稳定状态）
     * @return false 发布失败（未初始化 / shm 异常等）
     */
    bool publish(const shared::msg::NavState& state);

    /// @brief 发布器是否处于“工作正常”状态。
    ///
    /// 条件：已初始化（init 成功）且未设置 error_flag_。
    bool ok() const noexcept { return initialized_ && !error_flag_; }

    /// @brief 是否出现错误（例如 init 失败 / publish 过程中检测到错误）。
    bool hasError() const noexcept { return error_flag_; }

private:
    // ==================== 共享内存布局 ====================
    //
    // 导航 SHM 契约已统一定义在 shared/shm/nav_state_shm.hpp；
    // 发布器与消费端必须共用这一份定义，避免布局漂移。
    using ShmHeader = shared::shm::ShmHeader;
    using ShmLayout = shared::shm::ShmLayout;

    // shm 初始化 / 关闭的内部实现。
    bool init_shm(const NavStatePublisherConfig& cfg);
    void close_shm() noexcept;

private:
    bool        enabled_{false};      ///< 配置层开关
    bool        initialized_{false};  ///< 是否已成功 init
    bool        error_flag_{false};   ///< 是否检测到错误（仅累积，不自动清除）

    std::string shm_name_;            ///< 共享内存名称
    std::size_t shm_size_{0};         ///< 共享内存大小（字节）

#ifdef _WIN32
    void*      shm_handle_{nullptr};
    ShmLayout* shm_ptr_{nullptr};
#else
    int        shm_fd_{-1};
    ShmLayout* shm_ptr_{nullptr};
#endif
};

} // namespace nav_core::io
