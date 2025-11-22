#include "nav_core/imu_driver_wit.h"
#include "nav_core/timebase.h"

#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

extern "C" {
#include "wit_c_sdk.h"   // 来自 third_party/witmotion，内部包含 REG.h
}

namespace nav_core {

// ================== 串口 + 小工具 ==================

namespace {

inline bool isFinite(double x) {
    return std::isfinite(x);
}

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
        default:     return B115200;
    }
}

} // anonymous namespace

// ================== 静态实例（桥接 C SDK 回调） ==================

ImuDriverWit* ImuDriverWit::s_instance_ = nullptr;

// ================== 构造 / 析构 ==================

ImuDriverWit::ImuDriverWit(const std::string& port,
                           int baud,
                           uint8_t slave_addr,
                           const ImuFilterConfig& filter,
                           FrameCallback on_frame,
                           RawCallback   on_raw)
    : port_(port),
      baud_(baud),
      slave_addr_(slave_addr),
      filter_cfg_(filter),
      on_frame_(std::move(on_frame)),
      on_raw_(std::move(on_raw)) {
}

ImuDriverWit::~ImuDriverWit() {
    stop();
}

// ================== 公共接口 ==================

bool ImuDriverWit::start() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        // 已经在运行
        return true;
    }

    stop_requested_.store(false);

    if (!openPort()) {
        std::cerr << "[IMU] openPort() failed on " << port_ << "\n";
        running_.store(false);
        return false;
    }

    // ---- 初始化 Wit SDK（Modbus 模式） ----
    if (s_instance_ != nullptr && s_instance_ != this) {
        std::cerr << "[IMU] Warning: multiple ImuDriverWit instances, "
                     "Wit SDK only supports single instance\n";
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

    // 注册串口写回调
    if (WitSerialWriteRegister(&ImuDriverWit::serialWriteBridge) != WIT_HAL_OK) {
        std::cerr << "[IMU] WitSerialWriteRegister() failed\n";
        WitDeInit();
        closePort();
        s_instance_ = nullptr;
        running_.store(false);
        return false;
    }

    // 注册寄存器更新回调
    if (WitRegisterCallBack(&ImuDriverWit::regUpdateBridge) != WIT_HAL_OK) {
        std::cerr << "[IMU] WitRegisterCallBack() failed\n";
        WitDeInit();
        closePort();
        s_instance_ = nullptr;
        running_.store(false);
        return false;
    }

    sdk_inited_ = true;

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
              << " (Modbus, addr=0x" << std::hex << int(slave_addr_) << std::dec << ")\n";
    return true;
}

void ImuDriverWit::stop() {
    if (!running_.load()) {
        return;
    }

    stop_requested_.store(true);
    closePort();  // 打断 select/read

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

void ImuDriverWit::setFilterConfig(const ImuFilterConfig& cfg) {
    filter_cfg_ = cfg;
}

ImuFilterConfig ImuDriverWit::filterConfig() const {
    return filter_cfg_;
}

// ================== 串口打开 / 关闭 ==================

bool ImuDriverWit::openPort() {
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

    cfmakeraw(&tio);

    const speed_t s = baudToTermios(baud_);
    cfsetispeed(&tio, s);
    cfsetospeed(&tio, s);

    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSIZE;
    tio.c_cflag |= CS8;
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CSTOPB;

    // 非阻塞 + 超时配合 select
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 5;  // 0.5s 超时

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

void ImuDriverWit::closePort() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        std::cerr << "[IMU] serial closed\n";
    }
}

// ================== 线程主体：轮询读取 Modbus 寄存器 ==================

void ImuDriverWit::threadFunc() {
    // 以 100 Hz 轮询（每 10ms 发一次读寄存器命令）
    const int poll_hz     = 100;
    const int period_ms   = 1000 / poll_hz;
    const int recv_win_ms = 5;   // 每次发完命令后给 5ms 窗口接收数据

    while (!stop_requested_.load()) {
        if (fd_ < 0) {
            if (!openPort()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
        }

        // 1) 发送读寄存器命令：AX 开始读取 16 个寄存器
        //    与官方 Demo Main.cpp 用法保持一致：
        //    WitReadReg(AX, 16);
        if (WitReadReg(AX, 16) != WIT_HAL_OK) {
            // TODO: 如有需要，可做日志节流
            std::cerr << "[IMU] WitReadReg(AX,16) failed\n";
        }

        // 2) 在一个短窗口中收集响应数据，逐字节喂给 WitSerialDataIn()
        fd_set rfds;
        FD_ZERO(&rfds);
        FD_SET(fd_, &rfds);

        timeval tv{};
        tv.tv_sec  = recv_win_ms / 1000;
        tv.tv_usec = (recv_win_ms % 1000) * 1000;

        int ret = ::select(fd_ + 1, &rfds, nullptr, nullptr, &tv);
        if (ret < 0) {
            if (errno == EINTR) {
                // 被信号打断，下一次循环重试即可
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
            uint8_t buf[256];
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
            // n == 0: 设备暂时无数据，忽略即可
        }

        // 控制周期
        std::this_thread::sleep_for(std::chrono::milliseconds(period_ms));
    }

    std::cerr << "[IMU] threadFunc exit\n";
}

// ================== SDK 回调桥接 ==================

void ImuDriverWit::serialWriteBridge(uint8_t* data, uint32_t len) {
    if (s_instance_) {
        s_instance_->onSerialWrite(data, len);
    }
}

void ImuDriverWit::regUpdateBridge(uint32_t start_reg, uint32_t num) {
    if (s_instance_) {
        s_instance_->onRegUpdate(start_reg, num);
    }
}

void ImuDriverWit::onSerialWrite(uint8_t* data, uint32_t len) {
    if (fd_ < 0 || data == nullptr || len == 0) {
        return;
    }
    const ssize_t wn = ::write(fd_, data, static_cast<size_t>(len));
    if (wn != static_cast<ssize_t>(len)) {
        std::cerr << "[IMU] write() failed: "
                  << std::strerror(errno) << "\n";
    }
}

// sReg 在 wit_c_sdk.c 中定义
extern "C" int16_t sReg[REGSIZE];

void ImuDriverWit::onRegUpdate(uint32_t start_reg, uint32_t num) {
    // 这里 start_reg 通常是 AX，num 为 16（与 WitReadReg(AX,16) 对应）
    ImuRawRegs raw{};
    raw.timestamp_s = static_cast<double>(::time(nullptr));
    raw.start_reg   = start_reg;
    raw.count       = num;

    if (on_raw_) {
        on_raw_(raw);
    }

    if (!on_frame_) {
        return;
    }

    ImuFrame frame{};
    makeFrameFromRegs(start_reg, num, frame);
    if (!frame.valid) {
        return;
    }

    on_frame_(frame);
}

// ================== 寄存器 → 物理量 + 过滤 ==================

void ImuDriverWit::makeFrameFromRegs(uint32_t /*start_reg*/,
                                     uint32_t /*num*/,
                                     ImuFrame& out) {
    // ---- 1. 从 sReg[] 中取出各物理量 ----
    auto get_i16 = [](int reg) -> int16_t {
        if (reg < 0 || reg >= REGSIZE) {
            return 0;
        }
        return sReg[reg];
    };

    // 来自厂家 Main.cpp 示例（HWT905x / HWT9073 系列）：
    // a[i] = sReg[AX+i] / 32768.0 * 16.0                 // 16g 量程，加速度单位：g
    // w[i] = sReg[GX+i] / 32768.0 * 2000.0              // 2000 dps 量程，角速度单位：deg/s
    // Angle[i] = (((uint32_t)sReg[HRoll+2*i])<<16 |
    //             (uint16_t)sReg[LRoll+2*i]) / 1000.0   // 高精度角度，单位：deg
    // Temp = sReg[TEMP905x] / 100.0                     // 温度，°C

    constexpr double g       = 9.80665;
    constexpr double deg2rad = M_PI / 180.0;

    double acc_g[3];
    double gyro_dps[3];
    double angle_deg[3];

    for (int i = 0; i < 3; ++i) {
        acc_g[i]    = static_cast<double>(get_i16(AX + i)) / 32768.0 * 16.0;
        gyro_dps[i] = static_cast<double>(get_i16(GX + i)) / 32768.0 * 2000.0;

        const uint32_t hi = static_cast<uint16_t>(get_i16(HRoll + 2 * i));
        const uint32_t lo = static_cast<uint16_t>(get_i16(LRoll + 2 * i));
        const uint32_t iBuff = (hi << 16) | lo;
        angle_deg[i] = static_cast<double>(static_cast<int32_t>(iBuff)) / 1000.0;
    }

    const double temp_c = static_cast<double>(get_i16(TEMP905x)) / 100.0;

    // 转换到导航常用单位：
    const double ax = acc_g[0] * g;
    const double ay = acc_g[1] * g;
    const double az = acc_g[2] * g;

    const double gx = gyro_dps[0] * deg2rad;
    const double gy = gyro_dps[1] * deg2rad;
    const double gz = gyro_dps[2] * deg2rad;

    const double roll  = angle_deg[0] * deg2rad;
    const double pitch = angle_deg[1] * deg2rad;
    const double yaw   = angle_deg[2] * deg2rad;

    // ---- 2. 过滤逻辑 ----
    const ImuFilterConfig cfg = filter_cfg_;  // 拷贝一份，避免竞态

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

    // ---- 3. 时间戳 + 填充 ImuFrame ----
    int64_t mono_ns = 0;
    int64_t est_ns  = 0;
    stamp(mono_ns, est_ns);

    out.mono_ns = mono_ns;
    out.est_ns  = est_ns;

    out.lin_acc[0] = static_cast<float>(ax);
    out.lin_acc[1] = static_cast<float>(ay);
    out.lin_acc[2] = static_cast<float>(az);

    out.ang_vel[0] = static_cast<float>(gx);
    out.ang_vel[1] = static_cast<float>(gy);
    out.ang_vel[2] = static_cast<float>(gz);

    out.euler[0]   = static_cast<float>(roll);
    out.euler[1]   = static_cast<float>(pitch);
    out.euler[2]   = static_cast<float>(yaw);

    out.temperature = static_cast<float>(temp_c);
    out.valid       = true;
    out.status      = 0;
}

} // namespace nav_core
