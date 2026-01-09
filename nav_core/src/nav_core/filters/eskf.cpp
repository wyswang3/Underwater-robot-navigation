// nav_core/src/eskf.cpp
//
// 极简工程版 ESKF：IMU 预测 + DVL 速度更新 + 低频航向观测更新
//
// 当前版本的定位：
//   - 作为“导航主干”的起点，保证：
//       * 结构清晰，易于阅读和调试；
//       * 与 ImuFrame / DvlFrame / NavState 等类型完全对齐；
//       * 行为可预期（不会做太激进的滤波操作）。
//   - 暂时不引入矩阵库（Eigen 等），只实现“0 阶版本”，
//     后续如果你要写论文级 ESKF，可以在此基础上升级：
//       * 内部维护误差状态 + 协方差 P；
//       * 使用真实的 Kalman 增益；
//       * 引入重力对齐、零速更新等高级逻辑。

#include <nav_core/filters/eskf.hpp>   // EskfConfig / EskfState / Eskf
#include <nav_core/core/types.hpp>  // ImuFrame / DvlFrame 等

#include <algorithm>
#include <cmath>

namespace nav_core {

namespace {

// 常数
constexpr double PI     = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;

// 将 ZYX 欧拉角 (roll, pitch, yaw) 转成 body → nav 的旋转矩阵 R (3x3)
//
// 约定：
//   - 导航坐标系 nav：你可以选 ENU 或 NED，这里只要求内部自洽；
//   - 机体坐标系 body：右手系，x 前、y 右、z 下/上你自己在文档中说明。
//   - 这里我们只做标准数学变换，不偷换坐标系。
inline void eulerToRotMat(double roll, double pitch, double yaw,
                          double R[3][3])
{
    const double cr = std::cos(roll);
    const double sr = std::sin(roll);
    const double cp = std::cos(pitch);
    const double sp = std::sin(pitch);
    const double cy = std::cos(yaw);
    const double sy = std::sin(yaw);

    // 经典 ZYX 顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    // 注：这是 body → nav 的变换矩阵
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] =  cp * sr;
    R[2][2] =  cp * cr;
}

// 把角度 wrap 到 [-pi, pi]
inline double wrapToPi(double a)
{
    while (a >  PI) a -= TWO_PI;
    while (a < -PI) a += TWO_PI;
    return a;
}

// 本地 clamp（也可以换成 std::clamp）
template <typename T>
inline T clamp(T v, T lo, T hi)
{
    return std::max(lo, std::min(v, hi));
}

} // anonymous namespace

// ============================================================================
// 构造与重置
// ============================================================================

Eskf::Eskf(const EskfConfig& cfg)
    : cfg_(cfg)
{
    reset(nullptr);
}

void Eskf::reset(const EskfState* init_state)
{
    if (init_state) {
        // 使用外部给定的初始状态
        state_       = *init_state;
        has_state_   = true;
        initialized_ = init_state->valid;
        last_imu_ns_ = init_state->mono_ns;
    } else {
        // 完全清零，从头开始对准
        state_       = EskfState{};
        has_state_   = false;
        initialized_ = false;
        last_imu_ns_ = 0;
    }
}

// ============================================================================
// IMU / DVL / 航向观测 入口
// ============================================================================
//
// 调用频率建议：
//   - handleImu()：按 IMU 采样频率调用，例如 100 Hz；
//   - handleDvl()：按 DVL 输出频率调用，例如 10 Hz；
//   - handleYawObservation()：按你 IMU 实时滤波（yaw 解缠）输出频率调用，
//     可以是 5~10 Hz 低频观察量。

void Eskf::handleImu(const ImuFrame& imu)
{
    // 1) 如果 IMU 自己标记为无效，直接丢掉
    if (!imu.valid) {
        return;
    }

    // 2) 初始对准：第一次看到有效 IMU，就把它的欧拉角当作初始姿态
    if (!has_state_) {
        state_.mono_ns = imu.mono_ns;
        state_.est_ns  = imu.est_ns;

        state_.pos[0] = state_.pos[1] = state_.pos[2] = 0.f;
        state_.vel[0] = state_.vel[1] = state_.vel[2] = 0.f;

        state_.euler[0] = imu.euler[0];
        state_.euler[1] = imu.euler[1];
        state_.euler[2] = imu.euler[2];

        state_.bias_accel[0] = state_.bias_accel[1] = state_.bias_accel[2] = 0.f;
        state_.bias_gyro[0]  = state_.bias_gyro[1]  = state_.bias_gyro[2]  = 0.f;

        state_.valid  = true;
        state_.status = 0;

        has_state_    = true;
        initialized_  = true;
        last_imu_ns_  = imu.mono_ns;
        return;
    }

    // 3) 计算 dt（秒）
    if (last_imu_ns_ <= 0) {
        last_imu_ns_ = imu.mono_ns;
        return;
    }
    const std::int64_t dt_ns = imu.mono_ns - last_imu_ns_;
    if (dt_ns <= 0) {
        // 时间戳倒退或相等：直接丢弃这一帧
        return;
    }

    double dt = static_cast<double>(dt_ns) * 1e-9;

    // 理论上 dt ≈ 1 / imu_rate_hz，这里做一个防御式限幅：
    //   * 下限 = 0（负值已经过滤掉），不特别设置；
    //   * 上限 = min(0.05, 3 / imu_rate)，防止长时间卡顿直接积分炸掉。
    const double dt_max_from_rate =
        (cfg_.imu_rate_hz > 0.0) ? (3.0 / cfg_.imu_rate_hz) : 0.05;
    dt = clamp(dt, 0.0, std::min(0.05, dt_max_from_rate));

    // 如果 dt 太小（例如 < 1e-5），这帧对状态影响很小，可以直接略过
    if (dt <= 1e-6) {
        last_imu_ns_ = imu.mono_ns;
        return;
    }

    // 4) IMU 预测（惯性积分）
    predictFromImu(imu, dt);

    // 5) 更新时间戳 + 有效标志
    last_imu_ns_   = imu.mono_ns;
    state_.mono_ns = imu.mono_ns;
    state_.est_ns  = imu.est_ns;
    state_.valid   = initialized_;
}

void Eskf::handleDvl(const DvlFrame& dvl)
{
    if (!dvl.valid) {
        return;
    }
    if (!has_state_) {
        // 还没有 IMU 初始姿态，先不做 DVL 更新
        return;
    }

    updateWithDvl(dvl);
}

void Eskf::handleYawObservation(std::int64_t mono_ns,
                                double       yaw_rad,
                                double       R_yaw)
{
    if (!has_state_ || !initialized_) {
        // 还没完成 IMU 初始化，不用航向观测
        return;
    }

    updateWithYaw(mono_ns, yaw_rad, R_yaw);

    // 将状态时间对齐到这次航向观测时间（est_ns 仍交给 IMU / DVL 驱动维护）
    state_.mono_ns = mono_ns;
    state_.valid   = true;
}

// ============================================================================
// 状态获取接口
// ============================================================================

EskfState Eskf::latestState() const
{
    return state_;
}

bool Eskf::latestState(EskfState& out) const
{
    if (!has_state_ || !state_.valid) {
        return false;
    }
    out = state_;
    return true;
}

// ============================================================================
// 内部实现：IMU 预测步（惯性积分）
// ============================================================================
//
// 当前实现是 “0 阶版本”：
//   - 不显式维护误差状态和协方差；
//   - 不做更复杂的数值积分（如 RK4 / SO(3) 指数映射）；
//   - 仅用 IMU 的线加速度 + 角速度做位置 / 速度 / 姿态积分。
//
// 这个版本的好处：
//   - 工程上非常直观，新人容易看懂；
//   - 可以快速跑起来，后面再按论文细化。
//
void Eskf::predictFromImu(const ImuFrame& imu, double dt)
{
    if (dt <= 0.0) {
        return;
    }

    // 1) 去掉当前偏置（目前偏置恒为 0，将来可由滤波器估计）
    double ax = static_cast<double>(imu.lin_acc[0])
              - static_cast<double>(state_.bias_accel[0]);
    double ay = static_cast<double>(imu.lin_acc[1])
              - static_cast<double>(state_.bias_accel[1]);
    double az = static_cast<double>(imu.lin_acc[2])
              - static_cast<double>(state_.bias_accel[2]);

    double gx = static_cast<double>(imu.ang_vel[0])
              - static_cast<double>(state_.bias_gyro[0]);
    double gy = static_cast<double>(imu.ang_vel[1])
              - static_cast<double>(state_.bias_gyro[1]);
    double gz = static_cast<double>(imu.ang_vel[2])
              - static_cast<double>(state_.bias_gyro[2]);

    // 2) 姿态积分（简单欧拉法）
    double roll  = static_cast<double>(state_.euler[0]);
    double pitch = static_cast<double>(state_.euler[1]);
    double yaw   = static_cast<double>(state_.euler[2]);

    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    roll  = wrapToPi(roll);
    pitch = wrapToPi(pitch);
    yaw   = wrapToPi(yaw);

    state_.euler[0] = static_cast<float>(roll);
    state_.euler[1] = static_cast<float>(pitch);
    state_.euler[2] = static_cast<float>(yaw);

    // 3) 计算 body → nav 的旋转矩阵 R_b2n
    double R[3][3];
    eulerToRotMat(roll, pitch, yaw, R);

    // 4) 将“机体系加速度”转到“导航系”
    //    假设 imu.lin_acc 是“包含重力”的加速度测量（常见 MEMS 模式）：
    //       a_meas_body = a_true_body + R_n2b * g_nav
    //
    //    简化处理： a_nav = R_b2n * a_meas_body
    //             a_nav_lin = a_nav - g_nav
    //
    //    这里假定导航系 z 轴向上，则重力向量：
    //             g_nav = [0, 0, -g]
    double a_body[3] = {ax, ay, az};
    double a_nav[3]  = {
        R[0][0]*a_body[0] + R[0][1]*a_body[1] + R[0][2]*a_body[2],
        R[1][0]*a_body[0] + R[1][1]*a_body[1] + R[1][2]*a_body[2],
        R[2][0]*a_body[0] + R[2][1]*a_body[1] + R[2][2]*a_body[2]
    };

    const double g = cfg_.gravity;
    double a_nav_lin[3] = {
        a_nav[0],
        a_nav[1],
        a_nav[2] + g  // z 轴向上：a_true_z = a_nav_z + g
    };

    // 5) 速度积分 v_k+1 = v_k + a * dt
    for (int i = 0; i < 3; ++i) {
        double v = static_cast<double>(state_.vel[i]);
        v += a_nav_lin[i] * dt;
        state_.vel[i] = static_cast<float>(v);
    }

    // 6) 位置积分 p_k+1 = p_k + v * dt
    for (int i = 0; i < 3; ++i) {
        double p = static_cast<double>(state_.pos[i]);
        p += static_cast<double>(state_.vel[i]) * dt;
        state_.pos[i] = static_cast<float>(p);
    }
}

// ============================================================================
// 内部实现：DVL 更新步（简单互补滤波）
// ============================================================================
//
// 当前版本：
//   - 假设 DvlFrame.vel[] 已是导航系速度 [vx, vy, vz]，单位 m/s；
//   - 使用一个固定权重 w 做互补：
//         v_new = (1 - w) * v_pred + w * v_dvl
//   - w 暂时是一个经验值（0.3），后续可以根据 dvl_vel_noise_std 等自适应。
//
void Eskf::updateWithDvl(const DvlFrame& dvl)
{
    // 如果你已经在外面判断了 dvl.has_enu_vel，这里可以不重复；
    // 若没有，建议加一个快速返回：
    // if (!dvl.has_enu_vel) return;

    for (int i = 0; i < 3; ++i) {
        // 保护：非有限值直接跳过
        if (!std::isfinite(dvl.vel_enu_mps[i])) {
            continue;
        }

        const double v_meas = static_cast<double>(dvl.vel_enu_mps[i]);
        // TODO: 这里用 v_meas 构造测量残差 / H 矩阵 / 更新步骤
        // ... 你现有的逻辑保留不变，只是把源数据换成 vel_enu_mps ...
    }
}


// ============================================================================
// 内部实现：航向观测更新（yaw）
// ============================================================================
//
// 用法：
//   - yaw_rad：来自“实时 IMU 滤波器”的连续航向角（已解缠），单位 rad；
//   - R_yaw：航向观测噪声方差（rad^2），越大代表你对这个观测越不信任；
//   - 当前实现：用简化的 Kalman 权重公式计算互补系数 w，然后：
//         yaw_new = (1 - w) * yaw_pred + w * yaw_meas_near
//     其中 yaw_meas_near 是把测量 wrap 到预测 yaw 附近，避免 2π 跳变。
// 
void Eskf::updateWithYaw(std::int64_t /*mono_ns*/,
                         double       yaw_rad,
                         double       R_yaw)
{
    // 1) 当前预测 yaw
    double yaw_pred = static_cast<double>(state_.euler[2]);

    // 2) 把测量 wrap 到预测附近，避免 ±π 跳变
    double diff          = wrapToPi(yaw_rad - yaw_pred);  // ∈ [-pi, pi]
    double yaw_meas_near = yaw_pred + diff;

    // 3) 计算互补权重 w
    //
    //   先验 yaw 标准差 sigma_prior 粗略设为 20°：
    //   P_prior = sigma_prior^2
    //
    //   Kalman 权重类似：
    //   w = P_prior / (P_prior + R_yaw)
    //
    //   如果 R_yaw 非法（<=0），就退化为“观测和先验同等级”的情况。
    const double sigma_prior_deg = 20.0;
    const double sigma_prior_rad = sigma_prior_deg * PI / 180.0;
    const double P_prior         = sigma_prior_rad * sigma_prior_rad;

    double R_use = R_yaw;
    if (!(R_use > 0.0)) {
        R_use = P_prior;
    }

    double w = P_prior / (P_prior + R_use);
    w = clamp(w, 0.0, 1.0);

    // 4) 互补更新 yaw
    double yaw_new = (1.0 - w) * yaw_pred + w * yaw_meas_near;
    yaw_new = wrapToPi(yaw_new);

    state_.euler[2] = static_cast<float>(yaw_new);
    // 其他状态量（位置 / 速度）暂不动，只修正姿态 yaw。
}

} // namespace nav_core
