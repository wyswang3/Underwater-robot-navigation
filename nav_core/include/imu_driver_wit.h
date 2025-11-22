// Underwater-robot-navigation/nav_core/include/nav_core/imu_driver_wit.h
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <thread>

#include "nav_core/imu_types.h"

namespace nav_core {

/**
 * @brief 基于 WitMotion HWT9073-485 (Modbus-RTU) 的 IMU 驱动
 *
 * 设计思路：
 *  - 串口与线程模型由 nav_core 自己管理（POSIX + C++11 线程）
 *  - Modbus 协议解析、寄存器更新逻辑复用厂家 wit_c_sdk
 *  - 通过 WitSerialWriteRegister / WitRegisterCallBack 做桥接
 *  - 输出统一的 ImuFrame 结构，方便与 DvlFrame 一起做导航融合
 *
 * 限制：
 *  - Wit SDK 使用全局寄存器数组 sReg[]，因此当前实现只支持单实例
 *    （在一个进程中只创建一个 ImuDriverWit 对象）
 */
class ImuDriverWit {
public:
    using FrameCallback = std::function<void(const ImuFrame&)>;
    using RawCallback   = std::function<void(const ImuRawRegs&)>;

    /**
     * @param port        串口设备路径，例如 "/dev/ttyUSB0"
     * @param baud        波特率，例如 230400
     * @param slave_addr  Modbus 从机地址（HWT9073-485 通常为 0x50，可根据实际配置调整）
     * @param filter      初始过滤配置
     * @param on_frame    解析并过滤后的 IMU 帧回调
     * @param on_raw      可选：原始寄存器更新信息回调（用于调试）
     */
    ImuDriverWit(const std::string& port,
                 int baud,
                 uint8_t slave_addr,
                 const ImuFilterConfig& filter,
                 FrameCallback on_frame,
                 RawCallback   on_raw = nullptr);

    ~ImuDriverWit();

    /// 启动 IMU 驱动线程（打开串口，初始化 Wit SDK）
    bool start();

    /// 停止 IMU 驱动线程（关闭串口，释放 Wit SDK）
    void stop();

    /// 设置过滤配置（线程安全：内部按值拷贝）
    void setFilterConfig(const ImuFilterConfig& cfg);

    /// 获取当前过滤配置（按值返回）
    ImuFilterConfig filterConfig() const;

private:
    // 串口管理
    bool openPort();
    void closePort();

    // 线程主循环：周期性发起 WitReadReg + 从串口接收数据喂给 WitSerialDataIn
    void threadFunc();

    // ---- Wit SDK 回调桥接（C → C++） ----
    // Wit SDK 只支持全局回调，因此这里用静态指针指向当前实例
    static ImuDriverWit* s_instance_;

    // 提供给 WitSerialWriteRegister 的静态回调
    static void serialWriteBridge(uint8_t* data, uint32_t len);

    // 提供给 WitRegisterCallBack 的静态回调
    static void regUpdateBridge(uint32_t start_reg, uint32_t num);

    // 实际成员实现
    void onSerialWrite(uint8_t* data, uint32_t len);
    void onRegUpdate(uint32_t start_reg, uint32_t num);

    // 根据 sReg[] + start_reg/num 解析出 ImuFrame，并做简单过滤
    void makeFrameFromRegs(uint32_t start_reg, uint32_t num, ImuFrame& out);

private:
    // 配置
    std::string    port_;
    int            baud_;
    uint8_t        slave_addr_;

    // 串口 fd
    int            fd_{-1};

    // 过滤配置
    ImuFilterConfig filter_cfg_;

    // 回调
    FrameCallback   on_frame_;
    RawCallback     on_raw_;

    // 线程控制
    std::thread       th_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stop_requested_{false};
};

} // namespace nav_core
