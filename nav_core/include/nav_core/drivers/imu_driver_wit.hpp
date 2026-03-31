// nav_core/include/nav_core/drivers/imu_driver_wit.hpp
//
// @file  imu_driver_wit.hpp
// @brief WitMotion HWT9073-485 (Modbus-RTU) IMU 驱动封装（导航项目专用）。
//
// 设计目标：
//   - 将厂家 Wit C SDK（全局寄存器数组 + C 回调）的细节封装在 .cpp 中；
//   - 对上层（nav_core::estimator / nav_core::io）暴露统一的 IMU 采样接口：
//       * ImuFrame / ImuSample（见 nav_core/core/types.hpp）；
//       * ImuRawRegs（可选，用于调试寄存器更新）；
//   - 提供“线程化串口采集 + 回调模式”：
//       * 驱动内部启动一个采集线程，按固定频率轮询寄存器并喂给 Wit SDK；
//       * Wit SDK 通过 C 回调通知寄存器更新，由驱动解析成 ImuFrame 并调用上层回调；
//   - 当前实现只支持单实例（受 Wit SDK 全局状态限制）。
//
// 统一单位约定（与 nav_core/core/types.hpp 保持一致）：
//   - ImuFrame::lin_acc: m/s^2   （线加速度，[ax, ay, az]，IMU 自身坐标系）
//   - ImuFrame::ang_vel: rad/s   （角速度，[wx, wy, wz]，IMU 自身坐标系）
//   - ImuFrame::euler:   rad     （欧拉角，[roll, pitch, yaw]）
//
// 厂家原始寄存器 sReg[] 的典型换算关系（默认配置）：
//   - 加速度寄存器：sReg[AX..AZ] ∈ [-32768, 32767]
//       acc_g = sReg * (acc_range_g / 32768)
//       acc_mps2 = acc_g * raw_g_to_mps2
//   - 角速度寄存器：sReg[GX..GZ] ∈ [-32768, 32767]
//       gyro_deg_s = sReg * (gyro_range_dps / 32768)
//       gyro_rad_s = gyro_deg_s * π / 180
//   - 姿态角寄存器：sReg[Roll..Yaw] ∈ [-32768, 32767]
//       angle_deg = sReg * 180 / 32768
//       angle_rad = angle_deg * π / 180
//
// 这些换算会在 ImuDriverWit 内部完成，对外层（ESKF / 因子图等）完全屏蔽
// 厂家“counts / g / deg”的语义差异，上层统一消费 SI 单位。
//

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "nav_core/drivers/serial_binding.hpp"
#include "nav_core/drivers/imu_serial_diagnostics.hpp"
#include "nav_core/core/types.hpp"  // ImuFrame / ImuRawRegs / ImuFilterConfig

namespace nav_core::drivers {

struct ImuConfig {
    std::string  port;
    int          baud{230400};
    std::uint8_t slave_addr{0x50};
    SerialBindingConfig binding{};

    ImuFilterConfig filter{};

    // ===== 单位转换相关配置 =====
    float acc_range_g{16.0f};       ///< 加速度量程 ±acc_range_g g
    float gyro_range_dps{2000.0f};  ///< 角速度量程 ±gyro_range_dps °/s
    float raw_g_to_mps2{9.78f};     ///< 1 g → m/s²（与 Python 工程保持一致）
};

class ImuDriverWit {
public:
    using FrameCallback = std::function<void(const ImuFrame&)>;
    using RawCallback   = std::function<void(const ImuRawRegs&)>;

    ImuDriverWit();
    ImuDriverWit(const ImuConfig& cfg,
                 FrameCallback    on_frame,
                 RawCallback      on_raw = nullptr);
    ImuDriverWit(const std::string&      port,
                 int                     baud,
                 std::uint8_t            slave_addr,
                 const ImuFilterConfig&  filter,
                 FrameCallback           on_frame,
                 RawCallback             on_raw = nullptr);
    ~ImuDriverWit();

    ImuDriverWit(const ImuDriverWit&)            = delete;
    ImuDriverWit& operator=(const ImuDriverWit&) = delete;

    // ==================== 生命周期管理 ====================

    bool init(const ImuConfig& cfg,
              FrameCallback    on_frame,
              RawCallback      on_raw = nullptr);

    bool start();
    void stop();

    // ==================== 运行时配置 ====================

    void         setFilterConfig(const ImuFilterConfig& cfg);
    ImuFilterConfig filterConfig() const;

    // ==================== 运行状态与健康检查 ====================

    /// @brief 采集线程是否在运行（start 成功且尚未 stop）。
    bool isRunning() const noexcept;

    /// @brief 串口是否已成功打开。
    ///
    /// 说明：
    ///   - 返回 true 表示 openPort() 成功，底层 fd_ 有效；
    ///   - 若返回 false，说明当前无法与 IMU 通信（端口不存在/权限不足等）。
    bool isPortOpen() const noexcept;

    /// @brief 自 start() 以来是否至少收到过一帧 ImuFrame（无论 valid 与否）。
    ///
    /// 用途：
    ///   - 区分“串口打开了但没数据”（IMU 掉线 / 线路问题）
    ///   - 与 isPortOpen() 联合用于启动阶段自检：
    ///       * isPortOpen()==true && hasEverReceivedFrame()==true
    ///         ⇒ 串口确实连着一个在说话的 IMU。
    bool hasEverReceivedFrame() const noexcept;

    /// @brief 自 start() 以来是否至少收到过一帧有效 ImuFrame（valid==true）。
    ///
    /// 用途：
    ///   - 快速判断“有没有收到通过基本过滤的 IMU 数据”，
    ///   - 如果长时间为 false，可认为当前 IMU 输出异常或过滤过严。
    bool hasEverReceivedValidFrame() const noexcept;

    /// @brief 最近一帧 ImuFrame 的单调时间戳（ns）。
    ///
    /// 说明：
    ///   - 若尚未收到任何帧，返回 0；
    ///   - 上层可用（now_ns - lastFrameMonoNs()）对比某个超时时间，
    ///     判定 IMU 是否“近期在线”。
    MonoTimeNs lastFrameMonoNs() const noexcept;

    /// @brief 返回当前串口捕获到的诊断快照。
    ///
    /// 仅用于“串口已打开但 IMU 没有回调”的现场排障，
    /// 不参与导航解算语义。
    ImuSerialDebugSnapshot serialDebugSnapshot() const;

private:
    // ==================== 串口管理 ====================

    bool openPort();
    void closePort();

    // ==================== 线程主循环 ====================

    void threadFunc();

    // ==================== Wit SDK 回调桥接（C → C++） ====================

    static ImuDriverWit* s_instance_;

    static void serialWriteBridge(std::uint8_t* data, std::uint32_t len);
    static void regUpdateBridge(std::uint32_t start_reg, std::uint32_t num);

    void onSerialWrite(std::uint8_t* data, std::uint32_t len);
    void onRegUpdate(std::uint32_t start_reg, std::uint32_t num);

    /// @brief 根据 sReg[] 更新区间解析 ImuFrame（完成单位转换）。
    void makeFrameFromRegs(std::uint32_t start_reg,
                           std::uint32_t num,
                           ImuFrame&     out);

    // ==================== 单位转换辅助函数（实现放在 .cpp） ====================

    float rawAccCountToG(std::int16_t raw) const;
    float rawAccCountToMps2(std::int16_t raw) const;
    float rawGyroCountToDegPerSec(std::int16_t raw) const;
    float rawGyroCountToRadPerSec(std::int16_t raw) const;
    float rawAngleCountToDeg(std::int16_t raw) const;
    float rawAngleCountToRad(std::int16_t raw) const;

private:
    // ==================== 配置与状态 ====================

    ImuConfig       cfg_{};

    std::string     port_{};
    int             baud_{230400};
    std::uint8_t    slave_addr_{0x50};
    ImuFilterConfig filter_cfg_{};

    int  fd_{-1};
    bool sdk_inited_{false};

    FrameCallback on_frame_{};
    RawCallback   on_raw_{};

    std::thread       th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};

    // ===== 健康检查相关状态（由线程与回调维护） =====

    /// @brief 串口是否打开成功（openPort 成功置 true，closePort 置 false）。
    std::atomic<bool> port_open_{false};

    /// @brief 自 start() 以来是否收到过至少一帧 ImuFrame（无论 valid）。
    std::atomic<bool> have_any_frame_{false};

    /// @brief 自 start() 以来是否收到过至少一帧 valid==true 的 ImuFrame。
    std::atomic<bool> have_valid_frame_{false};

    /// @brief 最近一帧 ImuFrame 的单调时间戳（ns），未收到则为 0。
    std::atomic<MonoTimeNs> last_frame_mono_ns_{0};

    // 记录连接窗口内的原始串口特征，便于把“错口/错协议/错波特率”落成可读诊断。
    ImuSerialDiagnostics serial_diag_{};

    // 现场排障用：只打印少量前几帧 TX，确认“驱动是否真的在发 Modbus 请求”。
    std::atomic<int> tx_debug_budget_{0};
};

} // namespace nav_core::drivers
