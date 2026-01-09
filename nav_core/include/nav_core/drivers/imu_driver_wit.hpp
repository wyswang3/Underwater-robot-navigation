// nav_core/include/nav_core/drivers/imu_driver_wit.hpp
//
// @file  imu_driver_wit.hpp
// @brief WitMotion HWT9073-485 (Modbus-RTU) IMU 驱动封装（导航项目专用）。
//
// 设计目标：
//   - 将厂家 Wit C SDK（全局寄存器数组 + C 回调）的细节封装在 .cpp 中；
//   - 对上层（nav_core::estimator / nav_core::io）暴露统一的 IMU 采样接口：
//       * ImuFrame / ImuSample（见 nav_core/types.hpp）；
//       * ImuRawRegs（可选，用于调试寄存器更新）；
//   - 提供“线程化串口采集 + 回调模式”：
//       * 驱动内部启动一个采集线程，按固定频率轮询寄存器并喂给 Wit SDK；
//       * Wit SDK 通过 C 回调通知寄存器更新，由驱动解析成 ImuFrame 并调用上层回调；
//   - 当前实现只支持单实例（受 Wit SDK 全局状态限制）。
//
// 典型使用流程：
// @code
// using namespace nav_core;
// using namespace nav_core::drivers;
//
// ImuConfig cfg;
// cfg.port       = "/dev/ttyUSB0";
// cfg.baud       = 230400;
// cfg.slave_addr = 0x50;
// cfg.filter     = ImuFilterConfig{};  // 可按需调整阈值
//
// ImuDriverWit imu;
// bool ok = imu.init(cfg,
//                    [](const ImuFrame& frame) {
//                        // 处理标准化 IMU 帧（用于导航 / 记录）
//                    },
//                    nullptr  // 若不需要原始寄存器回调，可传 nullptr
// );
// if (!ok) {
//     // 处理初始化失败（串口/SDK 等）
// }
//
// if (!imu.start()) {
//     // 处理启动失败
// }
//
// // ... 程序运行中，回调会被持续调用 ...
//
// imu.stop();  // 安全结束：停止采集线程，关闭串口
// @endcode
//
// 实现说明：
//   - 具体串口读写 / Wit SDK 调用 / sReg[] 解析逻辑在 .cpp 中实现；
//   - 本头文件只定义：配置结构、回调类型、类接口与线程模型约定。
//

#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "nav_core/core/types.hpp"  // ImuFrame / ImuRawRegs / ImuFilterConfig

namespace nav_core::drivers {

/// @brief IMU 驱动配置（串口+协议+过滤）。
///
/// 说明：
///   - port: 串口设备路径（如 "/dev/ttyUSB0"、"/dev/ttyS1"）；
///   - baud: 串口波特率（HWT9073-485 通常为 230400）；
///   - slave_addr: Modbus 从机地址（厂家一般默认 0x50）；
///   - filter: 基本过滤配置（见 ImuFilterConfig），用于去除明显异常帧。
struct ImuConfig {
    std::string  port;                  ///< 串口设备路径，例如 "/dev/ttyUSB0"
    int          baud{230400};          ///< 波特率
    std::uint8_t slave_addr{0x50};      ///< Modbus 从机地址

    ImuFilterConfig filter{};           ///< IMU 基本过滤配置
};

/// @brief 基于 WitMotion HWT9073-485 (Modbus-RTU) 的 IMU 驱动。
///
/// 职责：
///   - 管理串口打开/关闭；
///   - 封装 Wit C SDK 初始化、寄存器回调、串口写回调；
///   - 在后台线程中循环：
///       * 向 IMU 发送读寄存器命令；
///       * 从串口读回数据并喂给 WitSerialDataIn；
///       * 在寄存器更新时解析为 ImuFrame，并调用上层回调；
///   - 对外只暴露统一的 ImuFrame / ImuRawRegs，不暴露 sReg[] 与原始 C 接口。
///
/// 限制：
///   - Wit SDK 使用全局寄存器数组 sReg[] 与全局回调，因此当前实现为
///     “单实例设计”：
///       * 在一个进程中只应创建一个 ImuDriverWit 对象；
///       * 内部通过静态指针 s_instance_ 将 C 回调桥接到当前实例；
///   - 非线程安全：不支持多个线程同时调用 init/start/stop，
///     需由上层串行管理生命周期。
class ImuDriverWit {
public:
    /// @brief 标准化 IMU 帧回调类型。
    using FrameCallback = std::function<void(const ImuFrame&)>;

    /// @brief 原始寄存器更新信息回调类型（可选，用于调试/统计）。
    using RawCallback   = std::function<void(const ImuRawRegs&)>;

    /// @brief 默认构造（不做任何初始化，需显式调用 init()）。
    ImuDriverWit();

    /// @brief 使用配置 + 回调构造（内部调用 init，但不会自动 start）。
    ///
    /// 等价于：
    /// @code
    /// ImuDriverWit imu;
    /// imu.init(cfg, on_frame, on_raw);
    /// @endcode
    ImuDriverWit(const ImuConfig& cfg,
                 FrameCallback on_frame,
                 RawCallback   on_raw = nullptr);

    /// @brief 兼容旧接口的构造函数。
    ///
    /// 等价于构造 ImuConfig 后调用 ImuDriverWit(cfg, ...)：
    /// @code
    /// ImuConfig cfg{port, baud, slave_addr, filter};
    /// ImuDriverWit imu(cfg, on_frame, on_raw);
    /// @endcode
    ImuDriverWit(const std::string&      port,
                 int                     baud,
                 std::uint8_t           slave_addr,
                 const ImuFilterConfig& filter,
                 FrameCallback          on_frame,
                 RawCallback            on_raw = nullptr);

    /// @brief 析构函数：若线程仍在运行，将自动调用 stop()。
    ~ImuDriverWit();

    // 禁止拷贝，允许移动（如有需要可后续实现移动构造/赋值）
    ImuDriverWit(const ImuDriverWit&)            = delete;
    ImuDriverWit& operator=(const ImuDriverWit&) = delete;

    // ==================== 生命周期管理 ====================

    /**
     * @brief 初始化驱动（配置串口与回调，不启动采集线程）。
     *
     * @param cfg       串口与过滤配置
     * @param on_frame  标准化 IMU 帧回调（必需，不能为空）
     * @param on_raw    可选：原始寄存器更新信息回调（用于调试/统计）
     * @return true     初始化成功（配置缓存就绪，可以 start）
     * @return false    初始化失败（参数非法 / 回调为空等）
     *
     * 注意：
     *   - 不会打开串口、不调用 Wit SDK，只是保存配置与回调；
     *   - 若需检查串口是否存在，请在 start() 中实现具体打开与错误报告。
     */
    bool init(const ImuConfig& cfg,
              FrameCallback    on_frame,
              RawCallback      on_raw = nullptr);

    /// @brief 启动 IMU 驱动线程（打开串口，初始化 Wit SDK，开始轮询）。
    ///
    /// @return true  启动成功（线程已开始运行）
    /// @return false 启动失败（例如串口打开失败 / SDK 初始化失败）
    ///
    /// 线程模型：
    ///   - 内部创建一个后台线程，执行 threadFunc() 主循环；
    ///   - 该线程会定期向 IMU 发送查询命令，并处理串口返回数据；
    ///   - 在寄存器更新时，通过回调向上层推送 ImuFrame / ImuRawRegs。
    bool start();

    /// @brief 停止 IMU 驱动线程（安全关停）。
    ///
    /// 行为：
    ///   - 通知线程退出（设置 stop_requested_ / running_ 标志）；
    ///   - 等待采集线程结束（join）；
///   - 关闭串口，释放 Wit SDK 相关资源；
///   - 在析构函数中会自动调用，无需重复调用。
    void stop();

    // ==================== 运行时配置 ====================

    /// @brief 设置过滤配置（线程安全：内部按值拷贝）。
    ///
    /// 说明：
    ///   - 可在运行过程中动态调整基本过滤参数（max_abs_accel / max_abs_gyro 等），
///     - 内部会在采集线程中按最新配置执行过滤逻辑。
    void setFilterConfig(const ImuFilterConfig& cfg);

    /// @brief 获取当前过滤配置（按值返回）。
    ImuFilterConfig filterConfig() const;

private:
    // ==================== 串口管理（仅在 .cpp 中实现） ====================

    /// @brief 打开串口并配置波特率等参数。
    ///
    /// @return true  串口打开成功
    /// @return false 串口打开失败（路径不存在 / 权限不足等）
    bool openPort();

    /// @brief 关闭串口（若已打开）。
    void closePort();

    // ==================== 线程主循环 ====================

    /// @brief 采集线程主函数。
    ///
    /// 典型逻辑（在 .cpp 中实现）：
    ///   - 调用 openPort() / Wit SDK 初始化；
    ///   - 循环：
    ///       * 发送读寄存器指令；
    ///       * 从串口读取数据并喂给 WitSerialDataIn；
    ///       * 根据 WitRegisterCallBack 通知解析 ImuFrame 并调用 on_frame_；
    ///   - 循环结束后，关闭串口，反初始化 SDK。
    void threadFunc();

    // ==================== Wit SDK 回调桥接（C → C++） ====================

    /// @brief 静态实例指针，用于 C 回调桥接到当前对象。
    ///
    /// 限制：
    ///   - 单实例：在 init/start 前应保证没有其他 ImuDriverWit 实例正在使用；
    ///   - 实际的赋值/清理逻辑在 .cpp 中管理。
    static ImuDriverWit* s_instance_;

    /// @brief 提供给 WitSerialWriteRegister 的静态回调。
    ///
    /// SDK 会通过该函数向串口发送数据，本函数内部会转发到当前实例的
    /// onSerialWrite() 成员函数。
    static void serialWriteBridge(std::uint8_t* data, std::uint32_t len);

    /// @brief 提供给 WitRegisterCallBack 的静态回调。
    ///
    /// SDK 在寄存器更新时调用本函数，本函数内部会转发到当前实例的
    /// onRegUpdate() 成员函数。
    static void regUpdateBridge(std::uint32_t start_reg, std::uint32_t num);

    /// @brief 实例级串口写实现（供 serialWriteBridge 调用）。
    void onSerialWrite(std::uint8_t* data, std::uint32_t len);

    /// @brief 实例级寄存器更新处理（供 regUpdateBridge 调用）。
    ///
    /// 典型操作：
    ///   - 构造 ImuRawRegs 并触发 RawCallback（若存在）；
///   - 从 sReg[] 解析出 ImuFrame，并依据过滤配置做基本筛选；
///   - 若帧有效，调用 FrameCallback。
    void onRegUpdate(std::uint32_t start_reg, std::uint32_t num);

    /// @brief 根据 sReg[] + 起始寄存器范围解析出 ImuFrame。
    ///
    /// @param start_reg   本次更新的起始寄存器地址
    /// @param num         连续寄存器数量
    /// @param out         输出的 ImuFrame（时间戳与物理量在 .cpp 中赋值）
    ///
    /// 说明：
    ///   - 仅负责“寄存器→物理量”的解码逻辑；
///   - 不做过滤，过滤逻辑由调用方按 ImuFilterConfig 处理。
    void makeFrameFromRegs(std::uint32_t start_reg,
                           std::uint32_t num,
                           ImuFrame&     out);

private:
    // ==================== 配置与状态 ====================

    ImuConfig       cfg_{};          ///< 原始配置（供调试/查询）

    // 展开字段，方便 .cpp 中使用并保持与 cfg_ 同步。
    std::string     port_{};
    int             baud_{230400};
    std::uint8_t    slave_addr_{0x50};
    ImuFilterConfig filter_cfg_{};   ///< 运行时过滤配置

    // 串口文件描述符（-1 表示未打开）。
    int fd_{-1};

    // Wit SDK 初始化状态。
    bool sdk_inited_{false};

    // 回调（由 init() 注册）
    FrameCallback on_frame_{};
    RawCallback   on_raw_{};

    // 线程控制。
    std::thread       th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
};

} // namespace nav_core::drivers
