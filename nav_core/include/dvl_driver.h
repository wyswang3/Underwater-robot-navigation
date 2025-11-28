#ifndef NAV_CORE_DVL_DRIVER_H
#define NAV_CORE_DVL_DRIVER_H

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>
#include <vector>

namespace nav_core {

/**
 * @brief 原始 DVL 解析数据结构（类似 Python DVLData）
 *
 * 对应 PD6/EPD6 解析后的字段：
 *  - timestamp_s: 本机 Unix 时间戳（秒）
 *  - src        : 帧类型（BE/BS/BI/BD/SA/TS ...）
 *  - ve_mm, vn_mm, vu_mm: 东/北/上 速度，单位 mm/s，可能为 NaN 或占位
 *  - depth_m, e_m, n_m, u_m: 深度/位移（部分协议帧才有）
 *  - valid     : A/V/None 对应 True/False/Unknown
 *
 * 说明：
 *  - 这是“协议级”的原始结构，供日志、调试或进一步过滤使用；
 *  - 实时融合一般使用经过过滤的 DvlFrame。
 */
struct DvlRawData {
    int64_t mono_ns = 0;   // steady-clock
    int64_t est_ns  = 0;   // latency-corrected timebase
    double  timestamp_s = 0.0;  // DVL internal time

    std::string src;
    double ve_mm = 0.0;
    double vn_mm = 0.0;
    double vu_mm = 0.0;
    double depth_m = 0.0;
    double e_m = 0.0;
    double n_m = 0.0;
    double u_m = 0.0;
    int valid = -1;

    inline double ve_mps() const { return ve_mm * 1e-3; }
    inline double vn_mps() const { return vn_mm * 1e-3; }
    inline double vu_mps() const { return vu_mm * 1e-3; }
};




/**
 * @brief DVL 速度样本（供 ESKF 使用的“精简 + 过滤后”结构）
 *
 *  - mono_ns / est_ns: 统一时间基
 *  - vel[3]: ENU 速度 [m/s]
 *  - valid : 是否通过本驱动内部过滤
 *  - quality: 质量指标（可来自底锁标志/协议字段）
 */
struct DvlFrame {
    int64_t mono_ns = 0;
    int64_t est_ns  = 0;
    float   vel[3]  = {0.0f, 0.0f, 0.0f};  ///< [vE, vN, vU], m/s
    bool    valid   = false;
    int     quality = 0;                   ///< 质量指标（0=未知/差，1=好...）
};


/**
 * @brief DVL 过滤配置（对应 Python MinimalSpeedWriter + 质量门控）
 *
 *  - only_valid_flag: 仅接受 valid==A 的速度帧
 *  - require_all_axes: 三轴速度必须都是有限数（非 NaN/Inf）
 *  - max_abs_vel_mps: 三轴速度绝对值上限（防止异常尖刺），<=0 表示不启用
 *  - min_quality    : 最小质量阈值，quality < min_quality 则认为无效
 */
struct DvlFilterConfig {
    bool   only_valid_flag  = true;
    bool   require_all_axes = true;
    double max_abs_vel_mps  = 5.0;   ///< 例如 5 m/s，若 <=0 则不检查
    int    min_quality      = 0;     ///< 若 quality < min_quality 则丢弃

    // 上层可以根据实验调整这几个参数
};


/**
 * @brief DVL 串口驱动（含协议解析 + 基本过滤）
 *
 * 典型用法：
 *  - 创建 DvlDriver(port, baud, filter_cfg, on_frame, on_raw)
 *  - 调用 start() 启动读取线程
 *  - 在回调中接收 DvlFrame（已按 filter_cfg 过滤）
 *
 * 线程模型：
 *  - 内部开启一个线程阻塞 read() 串口
 *  - 每读到一行 PD6/EPD6 文本，解析成 DvlRawData
 *  - 判断是否满足过滤条件，若满足则生成 DvlFrame 并调用 on_frame 回调
 *  - 可选调用 on_raw 回调输出原始文本（用于日志）
 */
class DvlDriver {
public:
    using FrameCallback = std::function<void(const DvlFrame&)>;
    using RawCallback   = std::function<void(const DvlRawData&)>;

    /**
     * @brief 构造函数
     * @param port      串口路径，如 "/dev/ttyUSB2"
     * @param baud      波特率，如 115200
     * @param filter    过滤配置（可后续 setFilterConfig 调整）
     * @param on_frame  过滤后速度帧回调（供 ESKF 使用）
     * @param on_raw    原始解析帧回调（可为 nullptr）
     */
    DvlDriver(const std::string& port,
              int baud,
              const DvlFilterConfig& filter,
              FrameCallback on_frame,
              RawCallback   on_raw = nullptr);

    ~DvlDriver();

    /// 启动后台读取线程
    bool start();

    /// 请求停止并等待后台线程退出
    void stop();

    /// 当前是否在运行
    bool running() const { return running_.load(); }

    /// 更新过滤配置（线程安全）
    void setFilterConfig(const DvlFilterConfig& cfg);

    /// 获取当前过滤配置（拷贝）
    DvlFilterConfig filterConfig() const;

private:
    /// 后台线程主函数
    void threadFunc();

    /// 打开/关闭串口（内部使用）
    bool openPort();
    void closePort();

    /// 将一行 PD6/EPD6 文本解析为 DvlRawData（类似 Python parse_lines + DVLData）
    /// 若解析失败返回 false
    bool parseLine(const std::string& line, DvlRawData& out_raw);

    /// 根据 raw + filter_cfg 决定是否生成 DvlFrame，并调用 on_frame
    void handleRawSample(const DvlRawData& raw);

    /// 基于当前过滤配置判断 raw 是否“可用”
    bool passFilter(const DvlRawData& raw, DvlFrame& out_frame);

private:
    std::string port_;
    int         baud_ = 115200;
    int         fd_   = -1;                ///< 串口文件描述符

    std::thread       th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};

    FrameCallback    on_frame_;
    RawCallback      on_raw_;
    DvlFilterConfig  filter_cfg_;

    // 可选：一些统计量，便于调试
    std::atomic<uint64_t> n_lines_{0};
    std::atomic<uint64_t> n_parsed_ok_{0};
    std::atomic<uint64_t> n_parsed_fail_{0};
    std::atomic<uint64_t> n_filtered_out_{0};
};

} // namespace nav_core

#endif // NAV_CORE_DVL_DRIVER_H
