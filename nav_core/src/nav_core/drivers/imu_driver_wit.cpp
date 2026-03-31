// nav_core/src/drivers/imu_driver_wit.cpp
//
// 基于 WitMotion HWT9073-485 (Modbus-RTU) 的 IMU 驱动实现
// - 串口配置 + C SDK 桥接 + 寄存器解析 → ImuFrame
// - 线程模型：内部起一个轮询线程，定频率发 Modbus 读命令，按字节喂给 Wit SDK
//

#include "nav_core/drivers/imu_driver_wit.hpp"
#include "nav_core/drivers/serial_port_utils.hpp"
#include "nav_core/core/timebase.hpp"

#include <cmath>
#include <cstring>
#include <ctime>
#include <iostream>
#include <sys/select.h>
#include <unistd.h>
#include <utility>
#include <thread>
// 如需要也可以补上 <chrono>，目前很多编译器会通过 <thread> 间接包含
// #include <chrono>

// Wit 官方 C SDK（在 third_party/witmotion 下）
extern "C" {
#include "wit_c_sdk.h"   // 内部包含 REG.h，提供 AX/GX/HRoll 等寄存器宏
}

// Wit SDK 内部的全局寄存器数组
extern "C" int16_t sReg[REGSIZE];

// 起一个短别名，后面写起来简单
namespace tb = nav_core::core::timebase;

namespace nav_core::drivers {

// ============================================================================
// 匿名命名空间：本文件内部小工具
// ============================================================================

namespace {

inline bool isFinite(double x) noexcept {
    return std::isfinite(x);
}

} // namespace

// ============================================================================
// 静态实例（桥接 C SDK 回调）
// ============================================================================
//
// Wit C SDK 使用“全局回调 + 全局寄存器数组”，只支持单实例。
// 这里用一个静态指针指向当前活动的 ImuDriverWit 对象。
//
ImuDriverWit* ImuDriverWit::s_instance_ = nullptr;

// ============================================================================
// 构造 / 析构 / init
// ============================================================================

ImuDriverWit::ImuDriverWit() = default;

ImuDriverWit::ImuDriverWit(const ImuConfig& cfg,
                           FrameCallback    on_frame,
                           RawCallback      on_raw)
{
    init(cfg, std::move(on_frame), std::move(on_raw));
}

ImuDriverWit::ImuDriverWit(const std::string&     port,
                           int                    baud,
                           std::uint8_t           slave_addr,
                           const ImuFilterConfig& filter,
                           FrameCallback          on_frame,
                           RawCallback            on_raw)
{
    ImuConfig cfg;
    cfg.port       = port;
    cfg.baud       = baud;
    cfg.slave_addr = slave_addr;
    cfg.filter     = filter;
    init(cfg, std::move(on_frame), std::move(on_raw));
}

ImuDriverWit::~ImuDriverWit()
{
    stop();
}

bool ImuDriverWit::init(const ImuConfig& cfg,
                        FrameCallback    on_frame,
                        RawCallback      on_raw)
{
    // 保存一份完整配置
    cfg_ = cfg;

    // 展开到成员字段，供实现内部使用
    port_       = cfg.port;
    baud_       = cfg.baud;
    slave_addr_ = cfg.slave_addr;
    filter_cfg_ = cfg.filter;

    on_frame_ = std::move(on_frame);
    on_raw_   = std::move(on_raw);

    // 运行时状态重置
    fd_         = -1;
    sdk_inited_ = false;

    stop_requested_.store(false);
    running_.store(false);

    port_open_.store(false);
    have_any_frame_.store(false);
    have_valid_frame_.store(false);
    last_frame_mono_ns_.store(0);
    serial_diag_.reset(slave_addr_);

    return true;
}

// ============================================================================
// 公共接口：start / stop / filterConfig / 健康检查
// ============================================================================

bool ImuDriverWit::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        // 已经在运行，直接认为成功
        return true;
    }

    stop_requested_.store(false);

    // 每次启动重置“本轮运行”的健康状态
    have_any_frame_.store(false);
    have_valid_frame_.store(false);
    last_frame_mono_ns_.store(0);
    port_open_.store(false);
    serial_diag_.reset(slave_addr_);

    if (!openPort()) {
        std::cerr << "[IMU] openPort() failed on " << port_ << "\n";
        running_.store(false);
        return false;
    }

    // -------- 初始化 Wit SDK（Modbus 模式） --------
    if (s_instance_ != nullptr && s_instance_ != this) {
        std::cerr
            << "[IMU] Warning: multiple ImuDriverWit instances, "
               "Wit SDK only supports a single active instance\n";
    }
    s_instance_ = this;

    // 选择 Modbus 协议（HWT9073-485 使用 Modbus RTU）
    if (WitInit(WIT_PROTOCOL_MODBUS, slave_addr_) != WIT_HAL_OK) {
        std::cerr << "[IMU] WitInit() failed\n";
        closePort();
        s_instance_ = nullptr;
        running_.store(false);
        return false;
    }

    // 注册串口写回调：Wit SDK 通过该回调往串口写 Modbus 帧
    if (WitSerialWriteRegister(&ImuDriverWit::serialWriteBridge) != WIT_HAL_OK) {
        std::cerr << "[IMU] WitSerialWriteRegister() failed\n";
        WitDeInit();
        closePort();
        s_instance_ = nullptr;
        running_.store(false);
        return false;
    }

    // 注册寄存器更新回调：每当解析出一帧数据时会调用
    if (WitRegisterCallBack(&ImuDriverWit::regUpdateBridge) != WIT_HAL_OK) {
        std::cerr << "[IMU] WitRegisterCallBack() failed\n";
        WitDeInit();
        closePort();
        s_instance_ = nullptr;
        running_.store(false);
        return false;
    }

    sdk_inited_ = true;

    // 启动内部线程：负责周期性发读命令 + 读取串口数据喂给 WitSerialDataIn
    try {
        th_ = std::thread(&ImuDriverWit::threadFunc, this);
    } catch (...) {
        std::cerr << "[IMU] failed to start thread\n";
        if (sdk_inited_) {
            WitDeInit();
            sdk_inited_ = false;
        }
        closePort();
        s_instance_ = nullptr;
        running_.store(false);
        throw;
    }

    std::cerr << "[IMU] started on " << port_ << " @" << baud_
              << " (Modbus, addr=0x"
              << std::hex << static_cast<int>(slave_addr_) << std::dec << ")\n";
    return true;
}

void ImuDriverWit::stop()
{
    if (!running_.load()) {
        return;
    }

    stop_requested_.store(true);

    // 提前关闭串口，打断 threadFunc 里的 select/read
    closePort();

    if (th_.joinable()) {
        th_.join();
    }

    if (sdk_inited_) {
        WitDeInit();
        sdk_inited_ = false;
    }

    if (s_instance_ == this) {
        s_instance_ = nullptr;
    }

    running_.store(false);

    std::cerr << "[IMU] stopped\n";
}

void ImuDriverWit::setFilterConfig(const ImuFilterConfig& cfg)
{
    // 简单按值覆盖。严格意义上这里没有做互斥保护，
    // 但 ImuFilterConfig 是小结构体，竞争概率较低。
    filter_cfg_ = cfg;
}

ImuFilterConfig ImuDriverWit::filterConfig() const
{
    return filter_cfg_;
}

// -------- 健康检查接口 --------

bool ImuDriverWit::isRunning() const noexcept
{
    return running_.load();
}

bool ImuDriverWit::isPortOpen() const noexcept
{
    return port_open_.load();
}

bool ImuDriverWit::hasEverReceivedFrame() const noexcept
{
    return have_any_frame_.load();
}

bool ImuDriverWit::hasEverReceivedValidFrame() const noexcept
{
    return have_valid_frame_.load();
}

MonoTimeNs ImuDriverWit::lastFrameMonoNs() const noexcept
{
    return last_frame_mono_ns_.load();
}

ImuSerialDebugSnapshot ImuDriverWit::serialDebugSnapshot() const
{
    return serial_diag_.snapshot();
}

// ============================================================================
// 串口打开 / 关闭
// ============================================================================

bool ImuDriverWit::openPort()
{
    if (fd_ >= 0) {
        port_open_.store(true);
        return true;
    }

    std::string error;
    fd_ = open_serial_port_raw(port_, baud_, /*read_timeout_ds=*/5, &error);
    if (fd_ < 0) {
        std::cerr << "[IMU] " << error << "\n";
        port_open_.store(false);
        return false;
    }

    std::cerr << "[IMU] serial opened: " << port_ << " @" << baud_ << "\n";
    port_open_.store(true);
    return true;
}

void ImuDriverWit::closePort()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cerr << "[IMU] serial closed\n";
    }
    port_open_.store(false);
}

// ============================================================================
// 线程主体：轮询读取 Modbus 寄存器
// ============================================================================
//
// 简单策略：
//   - 以 poll_hz 频率发起 WitReadReg(AX, 15)；
//   - 在一个短时间窗口内用 select + read 收集响应；
//   - 收到的每个字节都喂给 WitSerialDataIn()；
//   - Wit SDK 解析成功后会调用 regUpdateBridge()。
// ============================================================================
void ImuDriverWit::threadFunc()
{
    // 经验值：
    // - 现场 USB-RS485 + IMU 的回包延迟可能 >5ms，且 35B 回包可能被拆成多段 read()；
    // - Wit SDK 在 Modbus 模式下需要“请求-响应”对齐（回复帧不带起始寄存器地址），
    //   若我们在上一帧回复尚未完整到达前就发下一次 WitReadReg()，会覆盖 SDK 的请求上下文，
    //   导致 SDK 无法触发 regUpdate 回调，从而上层误判“没有可解析帧”。 
    //
    // 因此这里采用更保守的策略：降低轮询频率，并在每次请求后持续 drain 一段接收窗口。
    const int poll_hz       = 50;               // 50 Hz 轮询（更稳，足够给 ESKF propagate）
    const int period_ms     = 1000 / poll_hz;   // ~20 ms
    const int recv_win_ms   = 30;               // 每次命令后给更长接收窗口，确保完整回包到达
    const int select_slice_ms = 5;              // 细分 select，便于循环 drain

    while (!stop_requested_.load()) {
        if (fd_ < 0) {
            if (!openPort()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
        }

        // 1) 发送读寄存器命令：AX 开始读取 15 个寄存器
        //    当前实机稳定路径与 Python reader 对齐：读取 0x34..0x42，
        //    覆盖 acc/gyro/mag + 高精度姿态角，避免请求 16 个寄存器时实机无响应。
        if (WitReadReg(AX, 15) != WIT_HAL_OK) {
            // TODO: 如有需要，可做错误日志节流
            std::cerr << "[IMU] WitReadReg(AX,15) failed\n";
        }

        // 2) drain 串口接收窗口：尽量在当前周期内把回复读完并喂给 SDK
        const auto deadline =
            std::chrono::steady_clock::now() + std::chrono::milliseconds(recv_win_ms);

        while (!stop_requested_.load() &&
               fd_ >= 0 &&
               std::chrono::steady_clock::now() < deadline) {
            const auto now = std::chrono::steady_clock::now();
            const auto remaining_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - now);
            const int wait_ms = static_cast<int>(
                std::max<long long>(0ll, std::min<long long>(remaining_ms.count(), select_slice_ms)));
            if (wait_ms <= 0) {
                break;
            }

            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(fd_, &rfds);

            timeval tv{};
            tv.tv_sec  = wait_ms / 1000;
            tv.tv_usec = (wait_ms % 1000) * 1000;

            const int ret = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
            if (ret < 0) {
                if (errno == EINTR) {
                    continue;
                }
                std::cerr << "[IMU] select() error: " << std::strerror(errno) << "\n";
                closePort();
                break;
            }
            if (ret == 0 || !FD_ISSET(fd_, &rfds)) {
                continue;
            }

            std::uint8_t buf[256];
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n < 0) {
                if (errno == EAGAIN || errno == EWOULDBLOCK) {
                    continue;
                }
                std::cerr << "[IMU] read() error: " << std::strerror(errno) << "\n";
                closePort();
                break;
            }
            if (n == 0) {
                std::cerr << "[IMU] read() returned EOF, treating device as disconnected\n";
                closePort();
                break;
            }

            serial_diag_.record_rx_bytes(buf, static_cast<std::size_t>(n));
            for (ssize_t i = 0; i < n; ++i) {
                WitSerialDataIn(buf[i]);
            }
        }

        // 控制循环节奏
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    std::cerr << "[IMU] threadFunc exit\n";
}

// ============================================================================
// SDK 回调桥接（C → C++ 对象）
// ============================================================================

void ImuDriverWit::serialWriteBridge(std::uint8_t* data, std::uint32_t len)
{
    if (s_instance_) {
        s_instance_->onSerialWrite(data, len);
    }
}

void ImuDriverWit::regUpdateBridge(std::uint32_t start_reg, std::uint32_t num)
{
    if (s_instance_) {
        s_instance_->onRegUpdate(start_reg, num);
    }
}

void ImuDriverWit::onSerialWrite(std::uint8_t* data, std::uint32_t len)
{
    if (fd_ < 0 || data == nullptr || len == 0) {
        return;
    }
    const ssize_t wn = ::write(fd_, data, static_cast<std::size_t>(len));
    if (wn != static_cast<ssize_t>(len)) {
        std::cerr << "[IMU] write() failed: "
                  << std::strerror(errno) << "\n";
    }
}

// ============================================================================
// 寄存器更新 → ImuRawRegs + ImuFrame
// ============================================================================
//
// Wit SDK 每成功解析一帧数据，就会调用 onRegUpdate()，告诉我们：
//   - 哪个起始寄存器 start_reg
//   - 连续多少个寄存器 num
//
// 我们在这里：
//   1) 可选地上报 ImuRawRegs（用于调试/日志）；
//   2) 调用 makeFrameFromRegs() 将 sReg[] 转成物理单位的 ImuFrame；
//   3) 做基础过滤，最后通过 on_frame_ 回调交给上层（ESKF / logger）。
// ============================================================================

void ImuDriverWit::onRegUpdate(std::uint32_t start_reg, std::uint32_t num)
{
    // 统一时间基：先拿一份当前单调时间
    MonoTimeNs now_ns = tb::now_ns();

    ImuRawRegs raw{};
    raw.timestamp_s  = static_cast<double>(::time(nullptr));
    raw.host_mono_ns = now_ns;
    raw.start_reg    = start_reg;
    raw.count        = num;

    // 健康检查：只要回调被调用，就认为“收到过原始寄存器更新”
    serial_diag_.mark_parseable_frame();
    have_any_frame_.store(true);
    last_frame_mono_ns_.store(now_ns);

    if (on_raw_) {
        on_raw_(raw);
    }

    if (!on_frame_) {
        return; // 没有人关心高层帧，直接返回
    }

    ImuFrame frame{};
    makeFrameFromRegs(start_reg, num, frame);
    if (!frame.valid) {
        return;
    }

    // 有效帧统计
    have_valid_frame_.store(true);
    last_frame_mono_ns_.store(frame.mono_ns);

    on_frame_(frame);
}

// ============================================================================
// 单位转换小函数
// ============================================================================

float ImuDriverWit::rawAccCountToG(std::int16_t raw) const
{
    // 官方公式：raw / 32768 * 量程_g（默认 16g）
    return static_cast<float>(raw) / 32768.0f * cfg_.acc_range_g;
}

float ImuDriverWit::rawAccCountToMps2(std::int16_t raw) const
{
    // 先转成 g，再乘 raw_g_to_mps2（默认 9.78）
    const float acc_g = rawAccCountToG(raw);
    return acc_g * cfg_.raw_g_to_mps2;
}

float ImuDriverWit::rawGyroCountToDegPerSec(std::int16_t raw) const
{
    // 官方公式：raw / 32768 * 量程_dps（默认 2000 °/s）
    return static_cast<float>(raw) / 32768.0f * cfg_.gyro_range_dps;
}

float ImuDriverWit::rawGyroCountToRadPerSec(std::int16_t raw) const
{
    constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);
    return rawGyroCountToDegPerSec(raw) * kDeg2Rad;
}

float ImuDriverWit::rawAngleCountToDeg(std::int16_t raw) const
{
    // 一般 16 位角度寄存器常见缩放：raw / 32768 * 180.0
    // 目前 HWT9073-485 我们使用高精度 HRoll/LRoll 组合方式，
    // 这个函数暂时未在 makeFrameFromRegs 中使用，保留给其他型号。
    return static_cast<float>(raw) / 32768.0f * 180.0f;
}

float ImuDriverWit::rawAngleCountToRad(std::int16_t raw) const
{
    constexpr float kDeg2Rad = static_cast<float>(M_PI / 180.0);
    return rawAngleCountToDeg(raw) * kDeg2Rad;
}

// ============================================================================
// 寄存器 → 物理量 + 基本过滤
// ============================================================================
//
// 注意：
//   - 厂家寄存器含义参考官方 Demo (Main.cpp)，这里做了最小封装；
//   - 单位换算：加速度 g→m/s^2，角速度 deg/s→rad/s，角度 deg→rad；
//   - 基本过滤：NaN 检查 + 幅值限幅，避免严重异常数据污染后端。
// ============================================================================

void ImuDriverWit::makeFrameFromRegs(std::uint32_t /*start_reg*/,
                                     std::uint32_t /*num*/,
                                     ImuFrame& out)
{
    // 先标记为无效，确保 early-return 时不会误用旧数据
    out.valid  = false;
    out.status = 0;

    auto get_i16 = [](int reg) -> std::int16_t {
        if (reg < 0 || reg >= REGSIZE) {
            return 0;
        }
        return sReg[reg];
    };

    double acc_mps2[3];
    double gyro_rad_s[3];
    double angle_deg[3];

    // 加速度 / 角速度：使用配置驱动的单位转换
    for (int i = 0; i < 3; ++i) {
        const std::int16_t raw_acc  = get_i16(AX + i);
        const std::int16_t raw_gyro = get_i16(GX + i);

        acc_mps2[i]   = static_cast<double>(rawAccCountToMps2(raw_acc));
        gyro_rad_s[i] = static_cast<double>(rawGyroCountToRadPerSec(raw_gyro));
    }

    // 高精度角度：HRoll/LRoll 组合，单位 0.001 deg
    for (int i = 0; i < 3; ++i) {
        const std::uint32_t hi    = static_cast<std::uint16_t>(get_i16(HRoll + 2 * i));
        const std::uint32_t lo    = static_cast<std::uint16_t>(get_i16(LRoll + 2 * i));
        const std::uint32_t iBuff = (hi << 16) | lo;
        const std::int32_t  as_i32 = static_cast<std::int32_t>(iBuff);
        angle_deg[i] = static_cast<double>(as_i32) / 1000.0;  // 0.001 deg
    }

    const double temp_c = static_cast<double>(get_i16(TEMP905x)) / 100.0;

    const double roll  = angle_deg[0] * (M_PI / 180.0);
    const double pitch = angle_deg[1] * (M_PI / 180.0);
    const double yaw   = angle_deg[2] * (M_PI / 180.0);

    // -------- 过滤逻辑：这里只做非常基础的幅值/NaN 检查 --------
    const ImuFilterConfig cfg = filter_cfg_;  // 拷贝一份，避免并发撕裂

    if (cfg.enable_accel) {
        if (!isFinite(acc_mps2[0]) || !isFinite(acc_mps2[1]) || !isFinite(acc_mps2[2])) {
            return;
        }
        if (std::fabs(acc_mps2[0]) > cfg.max_abs_accel ||
            std::fabs(acc_mps2[1]) > cfg.max_abs_accel ||
            std::fabs(acc_mps2[2]) > cfg.max_abs_accel) {
            return;
        }
    }

    if (cfg.enable_gyro) {
        if (!isFinite(gyro_rad_s[0]) || !isFinite(gyro_rad_s[1]) || !isFinite(gyro_rad_s[2])) {
            return;
        }
        if (std::fabs(gyro_rad_s[0]) > cfg.max_abs_gyro ||
            std::fabs(gyro_rad_s[1]) > cfg.max_abs_gyro ||
            std::fabs(gyro_rad_s[2]) > cfg.max_abs_gyro) {
            return;
        }
    }

    if (cfg.enable_euler) {
        if (!isFinite(roll) || !isFinite(pitch) || !isFinite(yaw)) {
            return;
        }
        if (std::fabs(roll)  > cfg.max_euler_abs ||
            std::fabs(pitch) > cfg.max_euler_abs ||
            std::fabs(yaw)   > cfg.max_euler_abs) {
            return;
        }
    }

    // -------- 时间戳：使用统一 timebase::stamp --------
    auto ts = tb::stamp("imu0", tb::SensorKind::IMU);

    out.recv_mono_ns   = ts.host_time_ns;
    out.sensor_time_ns = ts.corrected_time_ns;
    out.consume_mono_ns = 0;
    out.mono_ns = (out.sensor_time_ns > 0) ? out.sensor_time_ns : out.recv_mono_ns;
    out.est_ns  = out.mono_ns;   // 兼容字段：当前与规范化样本时间保持一致

    // -------- 填充 IMU 数据 --------
    out.lin_acc[0] = static_cast<float>(acc_mps2[0]);
    out.lin_acc[1] = static_cast<float>(acc_mps2[1]);
    out.lin_acc[2] = static_cast<float>(acc_mps2[2]);

    out.ang_vel[0] = static_cast<float>(gyro_rad_s[0]);
    out.ang_vel[1] = static_cast<float>(gyro_rad_s[1]);
    out.ang_vel[2] = static_cast<float>(gyro_rad_s[2]);

    out.euler[0] = static_cast<float>(roll);
    out.euler[1] = static_cast<float>(pitch);
    out.euler[2] = static_cast<float>(yaw);

    out.temperature = static_cast<float>(temp_c);
    out.valid       = true;
    out.status      = 0;
}

} // namespace nav_core::drivers
