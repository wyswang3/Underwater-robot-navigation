// nav_core/include/nav_core/eskf.hpp
#pragma once

#include <array>
#include <cstdint>

#include <nav_core/core/types.hpp>

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
 * 坐标系（NED/ENU）由上层模块约定并保持一致。
 */
struct EskfState {
    MonoTimeNs mono_ns{0};   ///< 单调时钟时间戳（ns）
    SysTimeNs  est_ns{0};    ///< 估算时间基（ns，可与日志等对齐）

    float pos[3]   = {0.f, 0.f, 0.f}; ///< 位置（m）
    float vel[3]   = {0.f, 0.f, 0.f}; ///< 速度（m/s）
    float euler[3] = {0.f, 0.f, 0.f}; ///< 姿态（rad），(roll, pitch, yaw)

    float bias_accel[3] = {0.f, 0.f, 0.f}; ///< 加速度偏置估计 (m/s^2)
    float bias_gyro[3]  = {0.f, 0.f, 0.f}; ///< 角速度偏置估计 (rad/s)

    bool  valid  = false; ///< 状态是否有效
    int   status = 0;     ///< 状态码（0=正常，其他可用于诊断）
};

/**
 * @brief ESKF 主类
 *
 * 设计原则：
 *  - 不负责多线程，所有接口假定在同一线程内调用；
 *  - handleImu() 频率较高（典型 100 Hz）；
 *  - handleDvl() 频率较低（典型 10 Hz）；
 *  - latestState() 可在控制周期（例如 50 Hz）调用。
 *
 * 若需跨线程访问 latestState()，建议在调用方自行加锁，
 * 或在后续版本中为 Eskf 增加内部互斥保护。
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

    /**
     * @brief 处理一帧低频航向观测（更新步）
     *
     * @param mono_ns 单调时钟时间戳（ns）
     * @param yaw_rad 航向角观测，单位 rad，已解缠/平滑
     * @param R_yaw   航向观测方差（rad^2），用于观测噪声矩阵
     *
     * 典型用法：由实时 IMU 滤波器输出平滑的 yaw，低频调用本函数。
     */
    void handleYawObservation(MonoTimeNs mono_ns,
                              double     yaw_rad,
                              double     R_yaw);

    /// 获取最新状态（按值返回一份拷贝）
    EskfState latestState() const;

    /// 尝试获取最新状态，返回是否有效（out 仅在返回 true 时有效）
    bool latestState(EskfState& out) const;

    /// 返回当前是否已完成初始对准/初始化
    bool isInitialized() const { return initialized_; }

private:
    // 内部预测 / 更新实现（在 eskf.cpp 中实现）
    void predictFromImu(const ImuFrame& imu, double dt);
    void updateWithDvl(const DvlFrame& dvl);
    void updateWithYaw(MonoTimeNs mono_ns, double yaw_rad, double R_yaw);

private:
    EskfConfig cfg_{};

    EskfState state_{};
    bool      has_state_{false};      ///< 是否已有任何状态
    bool      initialized_{false};    ///< 是否完成初步对准/初始化

    MonoTimeNs last_imu_ns_{0};       ///< 上一帧 IMU 时间，用于计算 dt

    // 预留：内部协方差矩阵等（可在 eskf.cpp 中实现）
    // 例如：Eigen::Matrix<double, N, N> P_;
};

} // namespace nav_core
