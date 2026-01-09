// nav_core/src/drivers/imu_driver_wit.cpp
//
// 基于 WitMotion HWT9073-485 (Modbus-RTU) 的 IMU 驱动实现
// - 串口配置 + C SDK 桥接 + 寄存器解析 → ImuFrame
// - 线程模型：内部起一个轮询线程，定频率发 Modbus 读命令，按字节喂给 Wit SDK
//

#include "nav_core/drivers/imu_driver_wit.hpp"
#include "nav_core/core/timebase.hpp"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <ctime>
#include <fcntl.h>
#include <iostream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <utility>
#include <thread>

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

/// 将整型波特率转换为 termios 需要的 speed_t
speed_t baudToTermios(int baud) {
    switch (baud) {
        case 4800:   return B4800;
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
#ifdef B230400
        case 230400: return B230400;
#endif
#ifdef B460800
        case 460800: return B460800;
#endif
        default:
            std::cerr << "[IMU] unsupported baud " << baud
                      << ", fallback to 115200\n";
            return B115200;
    }
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
    // 保存一份完整配置，兼容老代码
    cfg_ = cfg;

    // 展开到成员字段，供实现内部使用
    port_       = cfg.port;
    baud_       = cfg.baud;
    slave_addr_ = cfg.slave_addr;
    filter_cfg_ = cfg.filter;

    on_frame_ = std::move(on_frame);
    on_raw_   = std::move(on_raw);

    // 运行时状态重置
    fd_           = -1;
    sdk_inited_   = false;
    stop_requested_.store(false);
    running_.store(false);

    return true;
}

// ============================================================================
// 公共接口：start / stop / filterConfig
// ============================================================================

bool ImuDriverWit::start()
{
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        // 已经在运行，直接认为成功
        return true;
    }

    stop_requested_.store(false);

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

// ============================================================================
// 串口打开 / 关闭
// ============================================================================

bool ImuDriverWit::openPort()
{
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        std::cerr << "[IMU] open(" << port_ << ") failed: "
                  << std::strerror(errno) << "\n";
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        std::cerr << "[IMU] tcgetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // 配置为原始模式：不做行缓冲/回显等
    cfmakeraw(&tio);

    const speed_t s = baudToTermios(baud_);
    cfsetispeed(&tio, s);
    cfsetospeed(&tio, s);

    tio.c_cflag |= (CLOCAL | CREAD);
    const tcflag_t csize_mask  = CSIZE;
    const tcflag_t parenb_mask = PARENB;
    const tcflag_t cstopb_mask = CSTOPB;

    tio.c_cflag &= ~parenb_mask;
    tio.c_cflag |= CS8;          // 8N1
    tio.c_cflag &= ~csize_mask;
    tio.c_cflag &= ~cstopb_mask;

    // 读超时：配合 select 使用
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 5;        // 0.5s 超时

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        std::cerr << "[IMU] tcsetattr failed: "
                  << std::strerror(errno) << "\n";
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    std::cerr << "[IMU] serial opened: " << port_ << " @" << baud_ << "\n";
    return true;
}

void ImuDriverWit::closePort()
{
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cerr << "[IMU] serial closed\n";
    }
}

// ============================================================================
// 线程主体：轮询读取 Modbus 寄存器
// ============================================================================
//
// 简单策略：
//   - 以 poll_hz 频率发起 WitReadReg(AX, 16)；
//   - 在一个短时间窗口内用 select + read 收集响应；
//   - 收到的每个字节都喂给 WitSerialDataIn()；
//   - Wit SDK 解析成功后会调用 regUpdateBridge()。
// ============================================================================
void ImuDriverWit::threadFunc()
{
    const int poll_hz     = 100;                // 100 Hz 轮询
    const int period_ms   = 1000 / poll_hz;     // 控制周期 ~10 ms
    const int recv_win_ms = 5;                  // 每次命令后给 5 ms 接收窗口

    while (!stop_requested_.load()) {
        if (fd_ < 0) {
            if (!openPort()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
        }

        // 1) 发送读寄存器命令：AX 开始读取 16 个寄存器
        //    用法参考官方 Demo Main.cpp
        if (WitReadReg(AX, 16) != WIT_HAL_OK) {
            // TODO: 如有需要，可做错误日志节流
            std::cerr << "[IMU] WitReadReg(AX,16) failed\n";
        }

        // 2) 使用 select 等待一小段时间的串口数据
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        timeval tv{};
        tv.tv_sec  = recv_win_ms / 1000;
        tv.tv_usec = (recv_win_ms % 1000) * 1000;

        int ret = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                // 被信号打断，不算致命错误，下次循环继续
                std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
                continue;
            }
            std::cerr << "[IMU] select() error: "
                      << std::strerror(errno) << "\n";
            closePort();
            std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
            continue;
        }

        if (ret > 0 && FD_ISSET(fd_, &rfds)) {
            std::uint8_t buf[256];
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n < 0) {
                std::cerr << "[IMU] read() error: "
                          << std::strerror(errno) << "\n";
                closePort();
            } else if (n > 0) {
                for (ssize_t i = 0; i < n; ++i) {
                    WitSerialDataIn(buf[i]);
                }
            }
            // n == 0: 暂时没有数据，忽略
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
    ImuRawRegs raw{};
    raw.timestamp_s  = static_cast<double>(::time(nullptr));

    // 这里改成用统一时间基
    raw.host_mono_ns = tb::now_ns();

    raw.start_reg    = start_reg;
    raw.count        = num;

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

    on_frame_(frame);
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

    // 参考 Wit 官方 Demo：
    //  a[i] = sReg[AX+i] / 32768.0 * 16.0                 // 16g 量程，加速度单位：g
    //  w[i] = sReg[GX+i] / 32768.0 * 2000.0              // 2000 dps 量程，角速度单位：deg/s
    //  Angle[i] = (((uint32_t)sReg[HRoll+2*i])<<16 |
    //              (uint16_t)sReg[LRoll+2*i]) / 1000.0   // 高精度角度，单位：deg
    //  Temp = sReg[TEMP905x] / 100.0                     // 温度，°C

    constexpr double g       = 9.80665;
    constexpr double deg2rad = M_PI / 180.0;

    double acc_g[3];
    double gyro_dps[3];
    double angle_deg[3];

    for (int i = 0; i < 3; ++i) {
        acc_g[i]    = static_cast<double>(get_i16(AX + i)) / 32768.0 * 16.0;
        gyro_dps[i] = static_cast<double>(get_i16(GX + i)) / 32768.0 * 2000.0;

        const std::uint32_t hi    = static_cast<std::uint16_t>(get_i16(HRoll + 2 * i));
        const std::uint32_t lo    = static_cast<std::uint16_t>(get_i16(LRoll + 2 * i));
        const std::uint32_t iBuff = (hi << 16) | lo;
        angle_deg[i]              = static_cast<double>(static_cast<std::int32_t>(iBuff)) / 1000.0;
    }

    const double temp_c = static_cast<double>(get_i16(TEMP905x)) / 100.0;

    // 转换到导航常用单位
    const double ax = acc_g[0] * g;
    const double ay = acc_g[1] * g;
    const double az = acc_g[2] * g;

    const double gx = gyro_dps[0] * deg2rad;
    const double gy = gyro_dps[1] * deg2rad;
    const double gz = gyro_dps[2] * deg2rad;

    const double roll  = angle_deg[0] * deg2rad;
    const double pitch = angle_deg[1] * deg2rad;
    const double yaw   = angle_deg[2] * deg2rad;

    // -------- 过滤逻辑：这里只做非常基础的幅值/NaN 检查 --------
    const ImuFilterConfig cfg = filter_cfg_;  // 拷贝一份，避免并发撕裂

    if (cfg.enable_accel) {
        if (!isFinite(ax) || !isFinite(ay) || !isFinite(az)) {
            return;
        }
        if (std::fabs(ax) > cfg.max_abs_accel ||
            std::fabs(ay) > cfg.max_abs_accel ||
            std::fabs(az) > cfg.max_abs_accel) {
            return;
        }
    }

    if (cfg.enable_gyro) {
        if (!isFinite(gx) || !isFinite(gy) || !isFinite(gz)) {
            return;
        }
        if (std::fabs(gx) > cfg.max_abs_gyro ||
            std::fabs(gy) > cfg.max_abs_gyro ||
            std::fabs(gz) > cfg.max_abs_gyro) {
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

    out.mono_ns = ts.host_time_ns;        // 单调时间
    out.est_ns  = ts.corrected_time_ns;   // 延迟补偿后的统一时间

    // -------- 填充 IMU 数据 --------
    out.lin_acc[0] = static_cast<float>(ax);
    out.lin_acc[1] = static_cast<float>(ay);
    out.lin_acc[2] = static_cast<float>(az);

    out.ang_vel[0] = static_cast<float>(gx);
    out.ang_vel[1] = static_cast<float>(gy);
    out.ang_vel[2] = static_cast<float>(gz);

    out.euler[0] = static_cast<float>(roll);
    out.euler[1] = static_cast<float>(pitch);
    out.euler[2] = static_cast<float>(yaw);

    out.temperature = static_cast<float>(temp_c);
    out.valid       = true;
    out.status      = 0;
}

} // namespace nav_core::drivers
