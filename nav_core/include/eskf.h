#pragma once

#include <cstdint>
#include <array>

#include "nav_core/imu_types.h"   // ImuFrame
#include "nav_core/dvl_driver.h"  // DvlFrame（假定这里声明了 DvlFrame）

namespace nav_core {

/**
 * @brief ESKF 配置参数（只放最关键的，后续可以扩展）
 */
struct EskfConfig {
    // 传感器频率（用于预测步整定）
    double imu_rate_hz   = 100.0;   ///< IMU 频率（预测步）
    double dvl_rate_hz   = 10.0;    ///< DVL 频率（更新步）

    // 重力
    double gravity       = 9.80665; ///< 重力加速度（m/s^2）

    // IMU 噪声（标准差）
    double accel_noise_std = 0.05;  ///< 线加速度噪声（m/s^2）
    double gyro_noise_std  = 0.01;  ///< 角速度噪声（rad/s）

    // DVL 速度观测噪声（标准差）
    double dvl_vel_noise_std = 0.02; ///< DVL 速度噪声（m/s）

    // 预留：过程噪声、观测噪声矩阵的缩放系数等
    double process_noise_scale = 1.0;
    double meas_noise_scale    = 1.0;
};

/**
 * @brief ESKF 对外可见的状态结构（导航结果）
 *
 * 注意：这里是“估计后的导航状态”，不是内部误差状态向量。
 */
struct EskfState {
    int64_t mono_ns = 0;   ///< 单调时钟时间戳
    int64_t est_ns  = 0;   ///< 估算时间基（可与日志等对齐）

    float pos[3]  = {0.f, 0.f, 0.f}; ///< 位置（m），NED 或 ENU 由上层定义
    float vel[3]  = {0.f, 0.f, 0.f}; ///< 速度（m/s）
    float euler[3]= {0.f, 0.f, 0.f}; ///< 姿态（rad），(roll, pitch, yaw)

    float bias_accel[3] = {0.f, 0.f, 0.f}; ///< 加速度偏置估计
    float bias_gyro[3]  = {0.f, 0.f, 0.f}; ///< 角速度偏置估计

    bool  valid  = false; ///< 状态是否有效
    int   status = 0;     ///< 状态码（0=正常，其他可用于诊断）
};

/**
 * @brief ESKF 主类
 *
 * 设计原则：
 *  - 不负责多线程，所有函数假定在同一线程内调用
 *  - handleImu() 频率较高（典型 100Hz）
 *  - handleDvl() 频率较低（典型 10Hz）
 *  - latestState() 可在控制周期（例如 50Hz）调用
 */
class Eskf {
public:
    explicit Eskf(const EskfConfig& cfg);
    ~Eskf() = default;

    /// 重置滤波器（可选提供初始状态）
    void reset(const EskfState* init_state = nullptr);

    /// 处理一帧 IMU 数据（预测步）
    void handleImu(const ImuFrame& imu);

    /// 处理一帧 DVL 速度观测（更新步）
    void handleDvl(const DvlFrame& dvl);

    /// 获取最新状态（拷贝一份）
    EskfState latestState() const;

    /// 尝试获取最新状态，返回是否有效
    bool latestState(EskfState& out) const;

    /// 返回当前是否已完成初始对准/初始化
    bool isInitialized() const { return initialized_; }

private:
    // 内部预测 / 更新实现（eskf.cpp 中实现）
    void predictFromImu(const ImuFrame& imu, double dt);
    void updateWithDvl(const DvlFrame& dvl);

private:
    EskfConfig cfg_;

    EskfState state_{};
    bool      has_state_    = false;   ///< 是否已有任何状态
    bool      initialized_  = false;   ///< 是否完成初步对准/初始化

    int64_t   last_imu_ns_  = 0;       ///< 上一帧 IMU 时间，用于计算 dt

    // 预留：内部协方差矩阵等（可在 eskf.cpp 中实现）
    // 例如：Eigen::Matrix<double, N, N> P_;
};

} // namespace nav_core
