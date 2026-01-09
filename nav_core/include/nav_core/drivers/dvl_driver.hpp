// nav_core/include/nav_core/drivers/dvl_driver.hpp
//
// @file  dvl_driver.hpp
// @brief Hover H1000 (PD6/EPD6) DVL 串口驱动：
//        串口 IO + 协议级解析桥接 + 基本过滤 + 命令发送 + 连接健康判定。
//
// 设计目标：
//   - 对上层（在线导航 / 离线因子图）暴露统一的 DVL 接口：
//       * DvlFrame（过滤后的“导航速度帧”，见 nav_core/types.hpp）；
//       * DvlRawData（带时间戳的协议级字段，用于日志/分析）；
//   - 内部负责：
//       * 串口阻塞 read()，按行读取 PD6/EPD6 文本；
//       * 调用 nav_core::dvl_protocol 解析为协议字段；
//       * 根据 DvlFilterConfig 做质量门控，生成 DvlFrame；
//       * 提供命令发送接口（CZ/CS 等 16 字节命令）；
//       * 提供“连接存活”判定（基于最近一帧接收时间）。
//
// 职责边界：
//   - 协议细节（行格式解析、16 字节命令打包）在 dvl_protocol.hpp/.cpp 中实现；
//   - 本驱动只做：IO + 时间戳 + 基本过滤 + 回调 + 健康判定；
//   - ZUPT / 噪声地板 / 因子图等高层逻辑统一放在 estimator 层或 math.hpp 中完成。
//

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "nav_core/core/types.hpp"          // MonoTimeNs / SysTimeNs / DvlFrame / DvlFilterConfig
#include "nav_core/drivers/dvl_protocol.hpp"   // TrackType / CoordFrame / ParsedLine / 命令打包

namespace nav_core::drivers {

// ============================ 1. 类型别名与 Raw 结构 ============================

/// 复用协议层的枚举与解析结果类型，避免重复定义。
using DvlTrackType  = dvl_protocol::TrackType;
using DvlCoordFrame = dvl_protocol::CoordFrame;
using DvlParsedLine = dvl_protocol::ParsedLine;
using CommandBytes = nav_core::dvl_protocol::CommandBytes;

/**
 * @brief 带时间戳的 DVL 协议级原始数据（供日志 / 调试 / 离线分析使用）
 *
 * 结构分两部分：
 *   - 时间信息：
 *       * mono_ns : 本机单调时钟（steady_clock）时间 [ns]；
 *       * est_ns  : 经延迟补偿后的“估计 UNIX 时间基” [ns]（可选，用于统一日志）；
 *   - 协议字段：
 *       * parsed  : 来自 dvl_protocol::parse_pd6_line 的解析结果（帧类型、坐标系、
 *                   速度、位移、A/V 有效标志、底跟踪/水团分类等）。
 *
 * 注意：
 *   - DvlRawData 不做任何“过滤”和“体 → ENU 旋转”，只是“这一行协议 + 时间戳”；
 *   - 在线导航通常使用 DvlFrame（types.hpp）作为过滤后、整合后的统一接口。
 */
struct DvlRawData {
    MonoTimeNs     mono_ns{0};
    SysTimeNs      est_ns{0};
    DvlParsedLine  parsed;

    // 便捷访问函数（避免上层频繁写 parsed.xxx）
    DvlTrackType  track_type() const noexcept { return parsed.track_type; }
    DvlCoordFrame coord_frame() const noexcept { return parsed.coord_frame; }

    bool is_bottom_track() const noexcept {
        return parsed.track_type == dvl_protocol::TrackType::BottomTrack;
    }
    bool is_water_track() const noexcept {
        return parsed.track_type == dvl_protocol::TrackType::WaterTrack;
    }

    double ve_mps() const noexcept { return parsed.ve_mps(); }
    double vn_mps() const noexcept { return parsed.vn_mps(); }
    double vu_mps() const noexcept { return parsed.vu_mps(); }
};


// ============================ 2. 驱动配置 ============================

/// @brief DVL 驱动配置。
///
/// 说明：
///   - port          : 串口路径，如 "/dev/ttyACM0"；
///   - baud          : 串口波特率；
///   - filter        : 协议级基本过滤配置（类型见 DvlFilterConfig）；
///   - auto_reconnect: 预留“自动重连”策略开关（当前仅占位，未实现具体策略）。
struct DvlConfig {
    std::string     port;             ///< 串口路径，如 "/dev/ttyACM0"
    int             baud{115200};     ///< 波特率
    DvlFilterConfig filter{};         ///< 过滤配置（来自 types.hpp）

    bool auto_reconnect{false};       ///< 预留：是否启用自动重连（当前未实现）
    // ★ 新增：CS 命令参数（呼率 & 平均次数）
    int cs_ping_rate{10};             ///< CS 呼率参数，默认 10
    int cs_avg_count{10};             ///< CS 平均次数，默认 10
};


// ============================ 3. DVL 驱动类 ============================

/**
 * @brief Hover H1000 DVL 串口驱动（IO + 协议桥接 + 基本过滤 + 命令发送）
 *
 * 职责：
 *  - 串口打开/关闭与后台读线程管理；
 *  - 从串口按行读取 PD6/EPD6 文本，调用 dvl_protocol::parse_pd6_line 解析为协议字段；
 *  - 为每一帧补充 Mono/Sys 时间戳，生成 DvlRawData；
 *  - 根据 DvlFilterConfig 对速度/高度等做基本门控；
 *  - 为“用于导航的底跟踪帧”构造 DvlFrame，并通过 FrameCallback 回调给上层；
 *  - 对所有协议级原始帧（包括水团帧等）通过 RawCallback 输出（可选）；
 *  - 提供 DVL 命令发送（CZ/CS 等 16 字节命令）的线程安全接口；
 *  - 提供简单的“连接健康”判定接口（基于最近一帧接收时间）。
 *
 * 约定：
 *  - FrameCallback 通常只针对“用于在线导航的底跟踪 Earth-frame 帧”（例如 BE/BD）；
///  - 水团帧 / BI/BS 体速等原始信息建议通过 RawCallback 单独记录，用于离线因子图、调参、诊断；
///  - 更高层的导航策略（ZUPT、噪声地板等）不在本驱动中实现。
 */
class DvlDriver {
public:
    /// @brief 过滤后的导航速度帧回调类型。
    using FrameCallback = std::function<void(const DvlFrame&)>;

    /// @brief 协议级原始帧回调类型（含解析字段与时间戳）。
    using RawCallback   = std::function<void(const DvlRawData&)>;

    /// @brief 默认构造（不做初始化，需显式调用 init()）。
    DvlDriver();

    /// @brief 使用配置 + 回调构造（内部调用 init，但不会自动 start）。
    DvlDriver(const DvlConfig& cfg,
              FrameCallback    on_frame,
              RawCallback      on_raw = nullptr);

    /// @brief 兼容旧接口的构造函数（port + baud + filter）。
    DvlDriver(const std::string&     port,
              int                    baud,
              const DvlFilterConfig& filter,
              FrameCallback          on_frame,
              RawCallback            on_raw = nullptr);

    ~DvlDriver();

    DvlDriver(const DvlDriver&)            = delete;
    DvlDriver& operator=(const DvlDriver&) = delete;
    DvlDriver(DvlDriver&&)                 = delete;
    DvlDriver& operator=(DvlDriver&&)      = delete;

    // ==================== 生命周期管理 ====================

    /**
     * @brief 初始化驱动（配置 + 回调），不启动线程。
     *
     * @param cfg       串口与过滤配置
     * @param on_frame  过滤后导航速度帧回调（必需，不能为空）
     * @param on_raw    协议级原始帧回调（可选，用于日志/调试）
     * @return true     初始化成功（配置已缓存，可以 start）
     * @return false    初始化失败（配置非法等）
     */
    bool init(const DvlConfig& cfg,
              FrameCallback    on_frame,
              RawCallback      on_raw = nullptr);

    /// @brief 启动后台读取线程。
    ///
    /// @return true  启动成功（线程已经开始阻塞读串口）
    /// @return false 启动失败（串口打开失败等）
    bool start();

    /// @brief 请求停止并等待后台线程退出（可重复调用）。
    void stop() noexcept;

    /// @brief 当前是否在运行（后台线程处于 active 状态）。
    [[nodiscard]] bool running() const noexcept { return running_.load(); }

    // ==================== 运行时配置 ====================

    /// @brief 更新过滤配置（线程安全：内部拷贝一份）。
    void setFilterConfig(const DvlFilterConfig& cfg);

    /// @brief 获取当前过滤配置（按值拷贝返回）。
    [[nodiscard]] DvlFilterConfig filterConfig() const;

    // ==================== 连接健康判定 ====================

    /// @brief 返回最近一次成功解析到 DVL 帧的 mono 时间（ns）。
    ///
    /// 若尚未收到任何帧，则返回 0。
    [[nodiscard]] MonoTimeNs lastRxMonoNs() const noexcept {
        return last_rx_mono_ns_.load();
    }

    /// @brief 简单“连接是否存活”判定。
    ///
    /// @param now_mono_ns   当前 mono 时间（ns），由上层调用时提供
    /// @param timeout_ns    判定超时时间（ns），例如 2 秒 = 2e9 ns
    /// @return true         若在 (now_mono_ns - timeout_ns, now_mono_ns] 内有数据；
/// @return false        若超时未收到数据，可视为链路异常 / DVL 掉线。
    [[nodiscard]] bool isAlive(MonoTimeNs now_mono_ns,
                               std::int64_t timeout_ns) const noexcept
    {
        const auto last = last_rx_mono_ns_.load();
        if (last <= 0) return false;
        return (now_mono_ns - last) <= timeout_ns;
    }

    // ==================== 命令发送（CZ/CS 等 16 字节） ====================

    /// @brief 发送原始命令（内部通过 dvl_protocol 组装为 16 字节并写串口）。
    ///
    /// 说明：
    ///   - 本函数会调用 dvl_protocol::make_command_from_ascii(ascii_cmd)，
///     将人类可读命令（如 "CZ"、"CS 10 10"）打包为 Hover 要求的固定
///     长度 16 字节命令（包括结尾 '\r'）；
///   - 然后通过 writeBytes() 一次性写入串口。
    ///
    /// @param ascii_cmd 人类可读命令字符串（不必是 16 字节）
/// @return true       发送成功（底层 writeBytes 返回 16）
/// @return false      发送失败（串口未打开 / write 错误等）
    bool sendRawCommand16(const std::string& ascii_cmd);

    /// @brief 发送 CZ 命令：强制停止 ping（安全停机）。
    ///
    /// 实现说明：
    ///   - 内部调用 dvl_protocol::make_command_cz() 生成 16 字节命令；
///   - 再通过 writeBytes 写入串口。
    bool sendCommandCZ();

    /// @brief 发送 CS 命令：启动 ping，并设置呼率 / 平均次数。
    ///
    /// @param ping_rate     呼率参数（具体含义按 Hover 手册，如 Hz 或内部码）
/// @param avg_count      平均次数（例如 10）
/// @return true          发送成功
/// @return false         发送失败
    ///
    /// 实现说明：
    ///   - 内部调用 dvl_protocol::make_command_cs(ping_rate, avg_count)；
///   - 再通过 writeBytes 写入串口。
    bool sendCommandCS();
    //设置发射呼率和平均次数
    bool sendCommandPR(int ping_rate); // 新增声明
    bool sendCommandPM(int avg_count); // 新增声明

private:
    // ==================== 后台线程与串口 ====================

    /// @brief 后台线程主函数。
    ///
    /// 典型流程（在 .cpp 中实现）：
    ///   - openPort()；
///   - 循环阻塞读取一行文本（遇到 '\n' 结束）；
///   - 调用 dvl_protocol::parse_pd6_line(line, parsed) 完成协议解析；
///   - 组装带时间戳的 DvlRawData 对象；
///   - 更新 last_rx_mono_ns_；
///   - 调用 handleRawSample() 执行过滤与回调。
    void threadFunc();

    /// @brief 打开串口（仅在 start() 或后台线程内部调用）。
    bool openPort();

    /// @brief 关闭串口（若已打开）。
    void closePort() noexcept;

    // ==================== 过滤 & 回调 ====================

    /// @brief 处理一帧原始数据：
    ///        1) 可选回调 on_raw_；
    ///        2) 根据过滤规则决定是否生成导航 DvlFrame，并调用 on_frame_。
    void handleRawSample(const DvlRawData& raw);

    /// @brief 判断 raw 是否通过过滤，并在通过时填充 out_frame。
    ///
    /// 典型策略（在 .cpp 中实现）：
    ///   - 只对“底跟踪 + Earth 坐标系”的帧产生 DvlFrame；
///   - 根据 DvlFilterConfig 检查速度/高度/quality；
///   - 对无效速度（A/V 标志为无效）按你的策略置零但保留时间戳。
    bool passFilter(const DvlRawData& raw, DvlFrame& out_frame);

    // ==================== 内部命令发送实现 ====================

    /// @brief 实际执行串口 write 的内部函数（不做 16 字节调整）。
    ///
    /// 说明：
    ///   - 调用前应由 dvl_protocol::* 已经构造好完整的 16 字节命令；
///   - 本函数只负责写入，不解析命令内容。
    ///
    /// @param buf 原始缓冲区指针
    /// @param len 字节数
    /// @return 写入成功字节数；若 <0 表示错误。
    int writeBytes(const std::uint8_t* buf, std::size_t len);

private:
    // ==================== 配置与状态 ====================

    DvlConfig cfg_{};             ///< 基础配置快照（原始 cfg）

    // 展开后的关键字段，便于实现使用。
    std::string     port_;        ///< 串口路径
    int             baud_{0};     ///< 波特率

    int fd_{-1};                  ///< 串口文件描述符（未打开时为 -1）

    // 线程与状态。
    std::thread       th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};

    // 回调。
    FrameCallback on_frame_{};
    RawCallback   on_raw_{};

    // 过滤配置（运行时可更新）。
    mutable std::mutex filter_mutex_;
    DvlFilterConfig    filter_cfg_{};

    // 统计量（方便调试与日志）。
    std::atomic<std::uint64_t> n_lines_{0};
    std::atomic<std::uint64_t> n_parsed_ok_{0};
    std::atomic<std::uint64_t> n_parsed_fail_{0};
    std::atomic<std::uint64_t> n_filtered_out_{0};

    // 最近一次成功解析帧的 mono 时间（用于 isAlive 判定）。
    std::atomic<MonoTimeNs> last_rx_mono_ns_{0};
    // 最近一次 BD 解析结果（前面我们已经加过）
    mutable std::mutex last_bd_mutex_{};
    bool               has_last_bd_{false};
    DvlParsedLine      last_bd_{};

    // === 最近一次 ENU 距离 / 深度缓存，用于滤波 ===
    double last_dist_enu_m_[3]{0.0, 0.0, 0.0};  // [E, N, U] 或至少用到 [0]
    double last_depth_m_{0.0};

    bool   has_last_dist_{false};
    bool   has_last_depth_{false};
    /// 发送一条 DVL 命令：负责写串口 + 基本长度检查
    bool sendCommand(const CommandBytes& bytes, const char* tag);
};

} // namespace nav_core::drivers
