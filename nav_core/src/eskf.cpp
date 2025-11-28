#include "eskf.h"

#include <cmath>
#include <algorithm>

namespace nav_core {

namespace {

// 计算 body → nav 的旋转矩阵 R (3x3)
// 使用 ZYX 欧拉顺序：R = Rz(yaw) * Ry(pitch) * Rx(roll)
static inline void eulerToRotMat(double roll, double pitch, double yaw,
                                 double R[3][3]) {
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double cy = std::cos(yaw);
    double sy = std::sin(yaw);

    // 经典 ZYX，body → nav
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

// clamp 辅助函数
template <typename T>
static inline T clamp(T v, T lo, T hi) {
    return std::max(lo, std::min(v, hi));
}

} // anonymous namespace


// ================== 构造 / 重置 ==================

Eskf::Eskf(const EskfConfig& cfg)
    : cfg_(cfg) {
    reset(nullptr);
}

void Eskf::reset(const EskfState* init_state) {
    if (init_state) {
        state_ = *init_state;
        has_state_   = true;
        initialized_ = init_state->valid;
        last_imu_ns_ = init_state->mono_ns;
    } else {
        state_ = EskfState{};
        has_state_   = false;
        initialized_ = false;
        last_imu_ns_ = 0;
    }
}

// ================== IMU / DVL 处理接口 ==================

void Eskf::handleImu(const ImuFrame& imu) {
    // 简单保护：若 IMU 标记无效直接丢弃
    if (!imu.valid) {
        return;
    }

    // 初始对准：第一次收到有效 IMU，用其欧拉角作为初始姿态
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

    // 计算 dt（秒）
    if (last_imu_ns_ <= 0) {
        last_imu_ns_ = imu.mono_ns;
        return;
    }
    int64_t dt_ns = imu.mono_ns - last_imu_ns_;
    if (dt_ns <= 0) {
        // 时间戳异常，跳过
        return;
    }

    double dt = static_cast<double>(dt_ns) * 1e-9;
    // 对 dt 做一个限幅，防止长时间卡顿造成巨大 dt
    dt = clamp(dt, 0.0, 0.05);  // 最长 50ms

    predictFromImu(imu, dt);

    last_imu_ns_ = imu.mono_ns;
    state_.mono_ns = imu.mono_ns;
    state_.est_ns  = imu.est_ns;
    state_.valid   = initialized_;
}

void Eskf::handleDvl(const DvlFrame& dvl) {
    if (!dvl.valid) {
        return;
    }
    if (!has_state_) {
        // 还没有 IMU 初始化，先不更新
        return;
    }

    updateWithDvl(dvl);
}

// ================== 状态获取 ==================

EskfState Eskf::latestState() const {
    return state_;
}

bool Eskf::latestState(EskfState& out) const {
    if (!has_state_ || !state_.valid) {
        return false;
    }
    out = state_;
    return true;
}

// ================== 内部实现：预测步 ==================

void Eskf::predictFromImu(const ImuFrame& imu, double dt) {
    if (dt <= 0.0) {
        return;
    }

    // 1) 取出 IMU 原始测量，并减去偏置（当前使用偏置=0，后续可扩展估计）
    double ax = static_cast<double>(imu.lin_acc[0]) - static_cast<double>(state_.bias_accel[0]);
    double ay = static_cast<double>(imu.lin_acc[1]) - static_cast<double>(state_.bias_accel[1]);
    double az = static_cast<double>(imu.lin_acc[2]) - static_cast<double>(state_.bias_accel[2]);

    double gx = static_cast<double>(imu.ang_vel[0]) - static_cast<double>(state_.bias_gyro[0]);
    double gy = static_cast<double>(imu.ang_vel[1]) - static_cast<double>(state_.bias_gyro[1]);
    double gz = static_cast<double>(imu.ang_vel[2]) - static_cast<double>(state_.bias_gyro[2]);

    // 2) 姿态积分（简单欧拉积分，后续可换成四元数 + SO(3) 指数映射）
    double roll  = static_cast<double>(state_.euler[0]);
    double pitch = static_cast<double>(state_.euler[1]);
    double yaw   = static_cast<double>(state_.euler[2]);

    roll  += gx * dt;
    pitch += gy * dt;
    yaw   += gz * dt;

    // 保持 yaw 在 [-pi, pi] 范围内（简单归一化）
    const double pi = 3.14159265358979323846;
    auto wrap_pi = [pi](double a) {
        while (a >  pi) a -= 2.0 * pi;
        while (a < -pi) a += 2.0 * pi;
        return a;
    };
    roll  = wrap_pi(roll);
    pitch = wrap_pi(pitch);
    yaw   = wrap_pi(yaw);

    state_.euler[0] = static_cast<float>(roll);
    state_.euler[1] = static_cast<float>(pitch);
    state_.euler[2] = static_cast<float>(yaw);

    // 3) 计算旋转矩阵 R_b2n
    double R[3][3];
    eulerToRotMat(roll, pitch, yaw, R);

    // 4) 将加速度从机体系转到导航系
    // 这里假设 imu.lin_acc 是“包含重力的加速度测量”（常见 MEMS 模式）。
    // 简化处理： a_nav = R * a_body - g_nav
    // 假设导航系 z 轴向上，则重力在导航系为 [0, 0, -g]
    double a_body[3] = { ax, ay, az };
    double a_nav[3]  = {
        R[0][0]*a_body[0] + R[0][1]*a_body[1] + R[0][2]*a_body[2],
        R[1][0]*a_body[0] + R[1][1]*a_body[1] + R[1][2]*a_body[2],
        R[2][0]*a_body[0] + R[2][1]*a_body[1] + R[2][2]*a_body[2]
    };

    // 重力补偿（保守处理）：a_nav_lin = a_nav - [0, 0, -g]
    // 即 a_nav_lin_z = a_nav_z + g
    double g = cfg_.gravity;
    double a_nav_lin[3] = {
        a_nav[0],
        a_nav[1],
        a_nav[2] + g   // 视 nav z 轴为“向上”
    };

    // 5) 速度积分 v = v + a * dt
    for (int i = 0; i < 3; ++i) {
        double v = static_cast<double>(state_.vel[i]);
        v += a_nav_lin[i] * dt;
        state_.vel[i] = static_cast<float>(v);
    }

    // 6) 位置积分 x = x + v * dt
    for (int i = 0; i < 3; ++i) {
        double p = static_cast<double>(state_.pos[i]);
        p += static_cast<double>(state_.vel[i]) * dt;
        state_.pos[i] = static_cast<float>(p);
    }
}

// ================== 内部实现：DVL 更新步 ==================

void Eskf::updateWithDvl(const DvlFrame& dvl) {
    // 当前 DVL 帧 vel[] 假设已经是导航系速度 (m/s)，例如 (ve, vn, vu)
    // 这里先做一个简单的互补滤波式融合：
    //
    //   v_new = (1 - w) * v_pred + w * v_dvl
    //
    // w 在 (0,1) 之间，保守起见先设为 0.3，
    // 后续可以由 dvl_vel_noise_std / accel_noise_std 自动推导。
    double w = 0.3;
    w = clamp(w, 0.0, 1.0);

    for (int i = 0; i < 3; ++i) {
        double v_pred = static_cast<double>(state_.vel[i]);
        double v_meas = static_cast<double>(dvl.vel[i]);
        double v_new  = (1.0 - w) * v_pred + w * v_meas;
        state_.vel[i] = static_cast<float>(v_new);
    }

    // 状态仍由 IMU 预测得到的位置/姿态，DVL 只调节速度
    state_.status = 0;
    state_.valid  = true;
}

} // namespace nav_core
