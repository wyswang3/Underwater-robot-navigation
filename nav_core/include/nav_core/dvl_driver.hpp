// nav_core/include/nav_core/dvl_driver.hpp
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>

#include <nav_core/types.hpp>

namespace nav_core {

/**
 * @brief 原始 DVL 解析数据结构（协议级）
 *
 * 对应 Hover H1000 PD6 / EPD6 解析后的字段：
 *  - mono_ns    : 本机单调时钟（steady_clock）时间 [ns]
 *  - est_ns     : 经延迟补偿后的“估计时间基” [ns]
 *  - timestamp_s: DVL 内部时间戳 [s]（如有）
 *  - src        : 帧类型标识（例如 "SA", "TS" 等）
 *  - ve_mm/vn_mm/vu_mm : ENU 速度分量，单位 mm/s，可为占位值
 *  - depth_m/e_m/n_m/u_m: 深度与位移（某些帧才包含）
 *  - valid      : 1 = A(可用)，0 = V(无效)，-1 = 未知/未设置
 *
 * 用途：
 *  - 主要用于日志、调试、离线分析；
 *  - 实时状态估计通常使用 types.hpp 中的 DvlFrame（过滤后）。
 */
struct DvlRawData {
    MonoTimeNs mono_ns{0};      ///< steady-clock 时间基
    SysTimeNs  est_ns{0};       ///< 延迟补偿后的时间基（可选）

    double     timestamp_s{0.0};///< DVL 自身时间（若无可为 0）

    std::string src;            ///< 帧类型字符串（如 "SA"）

    double ve_mm{0.0};          ///< 东向速度 [mm/s]
    double vn_mm{0.0};          ///< 北向速度 [mm/s]
    double vu_mm{0.0};          ///< 上向速度 [mm/s]

    double depth_m{0.0};        ///< 深度 [m]
    double e_m{0.0};            ///< 东向位移 [m]
    double n_m{0.0};            ///< 北向位移 [m]
    double u_m{0.0};            ///< 上向位移 [m]

    int    valid{-1};           ///< 1: A, 0: V, -1: unknown

    [[nodiscard]] inline double ve_mps() const noexcept { return ve_mm * 1e-3; }
    [[nodiscard]] inline double vn_mps() const noexcept { return vn_mm * 1e-3; }
    [[nodiscard]] inline double vu_mps() const noexcept { return vu_mm * 1e-3; }
};

/**
 * @brief DVL 驱动配置
 */
struct DvlConfig {
    std::string     port;          ///< 串口路径，如 "/dev/ttyUSB2"
    int             baud{115200};  ///< 波特率
    DvlFilterConfig filter{};      ///< 过滤配置（来自 types.hpp）
};

/**
 * @brief DVL 串口驱动（串口 IO + 文本协议解析 + 基本过滤）
 *
 * 线程模型：
 *  - 内部线程阻塞式 read() 串口，按行读取 PD6/EPD6 文本；
 *  - 每行解析为 DvlRawData；
 *  - 根据 filter_cfg_ 做质量过滤，生成 DvlFrame（nav_core::DvlFrame）；
 *  - 回调 on_frame_ / on_raw_ 供上层使用。
 */
class DvlDriver {
public:
    using FrameCallback = std::function<void(const DvlFrame&)>;   ///< 过滤后速度帧
    using RawCallback   = std::function<void(const DvlRawData&)>; ///< 协议级原始数据

    /// 默认构造（需要后续调用 init()）
    DvlDriver();

    /// 使用配置 + 回调构造（内部调用 init，但不会自动 start）
    DvlDriver(const DvlConfig& cfg,
              FrameCallback    on_frame,
              RawCallback      on_raw = nullptr);

    /// 兼容旧接口的构造函数（port + baud + filter）
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

    /**
     * @brief 初始化驱动（配置 + 回调），不启动线程
     *
     * @param cfg       串口与过滤配置
     * @param on_frame  过滤后速度帧回调
     * @param on_raw    原始解析帧回调
     * @return true     初始化成功
     * @return false    初始化失败
     */
    bool init(const DvlConfig& cfg,
              FrameCallback    on_frame,
              RawCallback      on_raw = nullptr);

    /// 启动后台读取线程（成功返回 true）
    bool start();

    /// 请求停止并等待后台线程退出（可重复调用）
    void stop() noexcept;

    /// 当前是否在运行
    [[nodiscard]] bool running() const noexcept { return running_.load(); }

    /// 更新过滤配置（内部拷贝一份，线程安全）
    void setFilterConfig(const DvlFilterConfig& cfg);

    /// 获取当前过滤配置（拷贝一份）
    [[nodiscard]] DvlFilterConfig filterConfig() const;

private:
    /// 后台线程主函数
    void threadFunc();

    /// 打开/关闭串口（仅在内部线程或 start() 调用）
    bool openPort();
    void closePort() noexcept;

    /// 将一行 PD6/EPD6 文本解析为 DvlRawData
    /// @return 解析成功返回 true，失败返回 false
    bool parseLine(const std::string& line, DvlRawData& out_raw);

    /// 根据当前过滤配置，处理一帧原始数据
    void handleRawSample(const DvlRawData& raw);

    /// 判断 raw 是否通过过滤，并在通过时填充 out_frame
    bool passFilter(const DvlRawData& raw, DvlFrame& out_frame);

private:
    // 基础配置快照（原始 cfg）
    DvlConfig cfg_{};

    // 展开后的关键字段，便于实现层使用
    std::string     port_;       ///< 串口路径（来自 cfg_.port）
    int             baud_{0};    ///< 波特率（来自 cfg_.baud）

    int fd_{-1};                 ///< 串口文件描述符（未打开时为 -1）

    // 线程与状态
    std::thread       th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};

    // 回调
    FrameCallback on_frame_{};
    RawCallback   on_raw_{};

    // 过滤配置（运行时可更新）
    mutable std::mutex filter_mutex_;
    DvlFilterConfig    filter_cfg_{};

    // 统计量（方便调试与日志）
    std::atomic<std::uint64_t> n_lines_{0};
    std::atomic<std::uint64_t> n_parsed_ok_{0};
    std::atomic<std::uint64_t> n_parsed_fail_{0};
    std::atomic<std::uint64_t> n_filtered_out_{0};
};

} // namespace nav_core
