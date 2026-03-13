// nav_core/include/nav_core/drivers/dvl_driver.hpp
//
// @file  dvl_driver.hpp
// @brief Hover H1000 (PD6/EPD6) DVL 串口驱动：
//        串口 IO + 协议级解析桥接 + 命令发送 + 连接健康判定。
//        不再在本层做“导航过滤 / 体 → ENU 旋转 / ESKF 输入”等逻辑。
//
// 当前定位（配合新架构）：
//   - 本驱动只负责：
//       * 串口打开/关闭 + 后台读线程；
//       * 调用 nav_core::dvl_protocol 解析每一行 PD6/EPD6 报文；
//       * 补充 Mono/Sys 时间戳，形成 DvlRawData；
//       * 通过 RawCallback 把所有合法 PD6/EPD6 帧推给上层；
//       * 提供 CZ/CS/PR/PM 命令发送接口，配合 DVL_logger 使用；
//       * 提供简单的 isAlive() 判定。
//   - 分类保存（BI/BE/BD 拆分成不同 CSV）、工程级预处理（质量门限、窗口重采样、
//     ENU 统一等）全部在“预处理模块”/DVL_logger/ESKF 前级完成。

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include "nav_core/core/types.hpp"            // MonoTimeNs / SysTimeNs
#include "nav_core/drivers/dvl_protocol.hpp"  // TrackType / CoordFrame / ParsedLine / CommandBytes

// ★ 前向声明：预处理层的 DvlRawSample（避免头文件之间强耦合）
namespace nav_core::preprocess {
struct DvlRawSample;
}

namespace nav_core::drivers {

// ============================ 1. 类型别名与 Raw 结构 ============================

using DvlTrackType   = dvl_protocol::TrackType;
using DvlCoordFrame  = dvl_protocol::CoordFrame;
using DvlParsedLine  = dvl_protocol::ParsedLine;
using DvlCommandBytes = dvl_protocol::CommandBytes;

/**
 * @brief 带时间戳的 DVL 协议级原始数据（供记录 / 预处理 / 离线分析使用）
 *
 * 说明：
 *   - DvlRawData 不做任何工程过滤，也不转换坐标系；
 *   - 仅仅是：“这一行 PD6/EPD6 报文” + “采样/接收时间元数据”；
 *   - 上层（例如 DVL_logger / 预处理模块）负责：
 *       * 根据 parsed.src / coord_frame 分类为 BI/BE/BD；
///       * 做质量门限 / 速度单位转换 / ENU 统一；
///       * 输出 CSV 供 Python 或 ESKF 使用。
 */
struct DvlRawData {
    MonoTimeNs    sensor_time_ns{0};   ///< 样本对应采样时刻（steady ns）
    MonoTimeNs    recv_mono_ns{0};     ///< 驱动线程收到/解码该帧时刻（steady ns）
    MonoTimeNs    mono_ns{0};          ///< 规范化样本时间（steady ns）
    SysTimeNs     est_ns{0};           ///< 兼容字段：当前与 mono_ns 保持一致
    DvlParsedLine parsed;       ///< 协议字段（来自 dvl_protocol::parse_pd6_line）

    // 便捷访问函数（避免上层频繁写 parsed.xxx）
    DvlTrackType  track_type() const noexcept { return parsed.track_type; }
    DvlCoordFrame coord_frame() const noexcept { return parsed.coord_frame; }

    bool is_bottom_track() const noexcept { return parsed.is_bottom_track(); }
    bool is_water_track()  const noexcept { return parsed.is_water_track();  }

    double ve_mps() const noexcept { return parsed.ve_mps(); }
    double vn_mps() const noexcept { return parsed.vn_mps(); }
    double vu_mps() const noexcept { return parsed.vu_mps(); }
};


// ============================ 2. 驱动配置 ============================

/**
 * @brief DVL 驱动配置。
 *
 * 说明：
 *   - port            : 串口路径，如 "/dev/ttyACM0"；
 *   - baud            : 串口波特率；
 *   - auto_reconnect  : 预留“自动重连”策略开关（当前仅占位，未实现具体策略）；
///   - send_startup_cmds:
 *       * 若为 true，start() 成功打开串口后会自动发送一组安全命令：
 *           - CZ  （确保上电后先停机，防止空气 ping）；
 *           - PR  / PM （设置呼率 & 平均次数）；
 *           - CS  （启动 ping）。
 *       * 若为 false，则仅打开串口和读线程，不主动下发命令；
///         由上层显式调用 sendCommand*()。
 */
struct DvlConfig {
    std::string port;              ///< 串口路径，如 "/dev/ttyUSB0"
    int         baud{115200};      ///< 串口波特率

    bool auto_reconnect{false};    ///< 预留：是否启用自动重连（当前未实现）
    bool send_startup_cmds{true};  ///< 是否在 start() 时自动下发 CZ/PR/PM/CS

    int ping_rate{10};             ///< PR/CS 呼率参数（参见 Hover 手册）
    int avg_count{10};             ///< PM 平均次数参数
};


// ============================ 3. DVL 驱动类 ============================

/**
 * @brief Hover H1000 DVL 串口驱动（IO + 协议桥接 + 命令发送）
 *
 * 核心职责：
 *  - 串口打开/关闭与后台读线程管理；
 *  - 从串口按行读取文本，调用 dvl_protocol::parse_pd6_line 解析；
 *  - 为每一帧补充 Mono/Sys 时间戳，生成 DvlRawData；
 *  - 通过 RawCallback 将所有成功解析的 PD6/EPD6 帧推给上层；
 *  - 提供 DVL 命令发送接口（CZ/CS/PR/PM）；
 *  - 提供简单的“连接健康”判定接口。
 *
 * 不再承担：
 *  - DvlFrame 导出、导航级过滤、坐标系转换、ZUPT、噪声地板等高层逻辑。
 */
class DvlDriver {
public:
    /// @brief 协议级原始帧回调类型（含解析字段与时间戳）。
    using RawCallback = std::function<void(const DvlRawData&)>;

    /// @brief 默认构造（需显式 init()）。
    DvlDriver();

    /// @brief 使用配置 + 回调构造（内部调用 init，但不会自动 start）。
    DvlDriver(const DvlConfig& cfg,
              RawCallback      on_raw);

    /// @brief 兼容旧接口：仅提供 port + baud，其他配置使用默认值。
    DvlDriver(const std::string& port,
              int                baud,
              RawCallback        on_raw);

    ~DvlDriver();

    DvlDriver(const DvlDriver&)            = delete;
    DvlDriver& operator=(const DvlDriver&) = delete;
    DvlDriver(DvlDriver&&)                 = delete;
    DvlDriver& operator=(DvlDriver&&)      = delete;

    // ==================== 生命周期管理 ====================

    /**
     * @brief 初始化驱动（配置 + 回调），不启动线程。
     *
     * @param cfg       串口与启动命令配置
     * @param on_raw    协议级原始帧回调（必需，不能为空）
     * @return true     初始化成功（配置已缓存，可以 start）
     * @return false    初始化失败（配置非法等）
     */
    bool init(const DvlConfig& cfg,
              RawCallback      on_raw);

    /// @brief 启动后台读取线程。
    ///
    /// 典型行为：
    ///   - 打开串口；
    ///   - 若 cfg_.send_startup_cmds == true，则按顺序发送：
    ///       CZ → PR(ping_rate) → PM(avg_count) → CS；
    ///   - 启动阻塞读取线程，从串口按行读文本，解析为 PD6/EPD6。
    ///
    /// @return true  启动成功（线程已经开始阻塞读串口）
    /// @return false 启动失败（串口打开失败等）
    bool start();

    /// @brief 请求停止并等待后台线程退出（可重复调用）。
    void stop() noexcept;

    /// @brief 当前是否在运行（后台线程处于 active 状态）。
    [[nodiscard]] bool running() const noexcept { return running_.load(); }

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

    // ==================== 命令发送（CZ/CS/PR/PM） ====================

    /// @brief 发送一条 16 字节命令（内部不再修改缓冲区）。
    ///
    /// @param bytes  已由 dvl_protocol 打包好的 16 字节命令
    /// @param tag    用于日志的标签字符串（如 "CZ" / "CS" / "PR"）
    /// @return true  写入成功（长度为 16）
    /// @return false 写入失败（串口未打开 / write 错误等）
    bool sendCommand(const DvlCommandBytes& bytes, const char* tag);

    /// @brief 发送 CZ 命令：强制停止 ping。
    bool sendCommandCZ();

    /// @brief 发送 CS 命令：启动 ping（不携带参数，呼率由 PR/PM 决定）。
    bool sendCommandCS();

    /// @brief 设置发射呼率（PR 命令）。
    ///
    /// @param ping_rate 呼率参数（0..99，内部按 format_pr_pm_arg_2d 规范化）
    bool sendCommandPR(int ping_rate);

    /// @brief 设置平均次数（PM 命令）。
    ///
    /// @param avg_count 平均次数参数（0..99）
    bool sendCommandPM(int avg_count);

private:
    // ==================== 后台线程与串口 ====================

    /// @brief 后台线程主函数。
    ///
    /// 典型流程（在 dvl_driver.cpp 中实现）：
    ///   - openPort()；
    ///   - （可选）根据 cfg_.send_startup_cmds 发送 CZ/PR/PM/CS；
    ///   - 循环阻塞读取一行文本（遇到 '\n' 结束）；
///   - 调用 dvl_protocol::parse_pd6_line(line, parsed) 完成协议解析；
///   - 组装带时间戳的 DvlRawData 对象；
///   - 更新 last_rx_mono_ns_；
///   - 调用 handleRawSample() 执行回调。
    void threadFunc();

    /// @brief 打开串口（仅在 start() 或后台线程内部调用）。
    bool openPort();

    /// @brief 关闭串口（若已打开）。
    void closePort() noexcept;

    /// @brief 实际执行串口 write 的内部函数。
    ///
    /// @param buf 原始缓冲区指针
    /// @param len 字节数（通常为 16）
    /// @return 写入成功字节数；若 <0 表示错误。
    int writeBytes(const std::uint8_t* buf, std::size_t len);

    /// @brief 处理一帧成功解析的原始数据，调用 on_raw_ 回调。
    void handleRawSample(const DvlRawData& raw);

private:
    // ==================== 配置与状态 ====================

    DvlConfig cfg_{};              ///< 配置快照

    std::string port_;             ///< 串口路径
    int         baud_{0};          ///< 波特率

    int fd_{-1};                   ///< 串口文件描述符（未打开时为 -1）

    std::thread       th_;         ///< 后台读线程
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};

    RawCallback on_raw_{};         ///< 原始帧回调

    // 统计量（方便调试与日志）。
    std::atomic<std::uint64_t> n_lines_{0};        ///< 读到的总行数
    std::atomic<std::uint64_t> n_parsed_ok_{0};    ///< parse_pd6_line 成功数
    std::atomic<std::uint64_t> n_parsed_fail_{0};  ///< parse_pd6_line 失败数

    // 最近一次成功解析帧的 mono 时间（用于 isAlive 判定）。
    std::atomic<MonoTimeNs> last_rx_mono_ns_{0};
};
/// @brief 将驱动输出的 DvlRawData 桥接为预处理模块使用的 DvlRawSample。
///
/// 仅做：
///   - 时间戳 / bottom_lock / 质量指标的拷贝；
///   - 根据 src / coord_frame 填充 vel_inst_mps / vel_earth_mps；
/// 不做：
///   - 门控 / 噪声地板 / 限幅 / ENU 旋转等高层导航逻辑。
///
/// @param in   DvlDriver 输出的原始数据（含 ParsedLine）
/// @param out  预处理层的 DvlRawSample（若返回 true，则填充有效）
/// @return true  支持的帧类型（BI/BE/BS/BD），已成功填充 out
/// @return false 不支持的帧类型（例如 WI/WE/WD），上层可跳过该帧
bool makeDvlRawSample(const DvlRawData& in,
                      ::nav_core::preprocess::DvlRawSample& out);

} // namespace nav_core::drivers
