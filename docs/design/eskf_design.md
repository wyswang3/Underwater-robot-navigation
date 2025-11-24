
---

# **Error-State Kalman Filter (ESKF) Design Document**

**水下机器人多传感器导航系统 · ESKF 设计文档**

---

## 1. 文档目的（Purpose）

本文件明确本项目中 ESKF（Error-State Kalman Filter）的位置、功能、数学框架和工程实现路径。

目标：

* 统一项目中“状态估计”的数学基础
* 明确 IMU / DVL / Depth / USBL 的观测模型
* 描述时间对齐、误差状态、偏置估计策略
* 给 Python离线 / C++在线 实现提供明确指导
* 作为新成员理解多传感器融合的首选文档

---

# **2. 为什么使用 ESKF？**

对于你的水下机器人：

* IMU 频率高（50~200 Hz）但漂移
* DVL 频率低（1~10 Hz）但稳态速度准
* Depth 频率适中（5~20 Hz）
* USBL 超低频（0.1~2 Hz），但提供绝对位置

这些传感器特点决定：

**不能只用一个传感器；必须用滤波融合。**

ESKF 的优势：

* 状态使用“真实物理状态”
* Kalman 更新在“误差空间”，数学更稳定
* IMU 离散化稳定，适合高频积分
* 易于扩展更多传感器（USBL/Depth/Barometer）

这也是无人机、SLAM、PX4 等系统的主流方法。

---

# **3. 状态定义（State Definition）**

ESKF 有两套状态：

---

## 3.1 Nominal State（真实的物理状态）

```
X = [
    p   (位置 3x1)
    v   (速度 3x1)
    q   (姿态四元数 4x1)
    b_a (加速度偏置 3x1)
    b_g (陀螺仪偏置 3x1)
]
```

维度：**16维**。

---

## 3.2 Error State（误差状态）

用小量 perturbation 表示：

```
δx = [
    δp
    δv
    δθ   (姿态误差的三维小角度)
    δb_a
    δb_g
]
```

维度：**15维**。

误差状态用于 Kalman 更新；
nominal state 用于物理积分。

---

# **4. 系统预测模型（IMU Propagation）**

预测方程是 ESKF 的关键，也是本项目 IMU → 状态融合的核心。

---

## 4.1 连续时间模型

IMU 提供：

```
a_m = a_true + b_a + noise
ω_m = ω_true + b_g + noise
```

状态微分：

```
dp/dt = v
dv/dt = R(q) * (a_m - b_a - n_a) + g
dq/dt = 1/2 * Ω(ω_m - b_g - n_g) * q
db_a/dt = n_ba
db_g/dt = n_bg
```

---

## 4.2 离散化（Δt 时间步）

每个 IMU 时间步执行：

```
p ← p + v*dt + 0.5*R(a_m - b_a)*dt^2
v ← v + R(a_m - b_a)*dt
q ← q ⊗ Exp((ω_m - b_g)*dt)
b_a 不变（或随机游走）
b_g 不变（或随机游走）
```

---

## 4.3 误差状态传播（Jacobian 线性化）

误差状态满足：

```
δx_k+1 = F_k δx_k + G_k w_k
```

其中 `F_k` 为线性化 Jacobian。

常见结构：

```
δp' = δv
δv' = -R[a]_x δθ - I δb_a + noise
δθ' = -[ω]_x δθ - I δb_g + noise
δb_a' = noise
δb_g' = noise
```

---

# **5. 多传感器观测模型（Measurement Models）**

ESKF 的关键是**不同传感器如何约束状态**。

你的系统当前支持：

* DVL（速度）
* Depth Sensor（深度）
* USBL（绝对位置）
* IMU（已包含在预测中）

---

## 5.1 DVL 观测模型（Velocity）

DVL 输出：

```
z_dvl = [v_east, v_north, v_up]
```

观测模型：

```
z = R_wb * v_body + n
```

或直接使用 world frame 速度：

```
H_dvl = [0  I  0  0  0]
```

Kalman 更新：

```
δx ← δx + K (z - H x_hat)
```

---

## 5.2 Depth Sensor（深度）

深度是 Z 轴位置：

```
z_depth = p_z + noise

H_depth = [0 0 1  0...]
```

更新只约束 `p_z`。

---

## 5.3 USBL（绝对位置）

USBL 输出 `(x,y,z)`（低频）：

```
z_usbl = p + noise

H_usbl = [I_3, 0...]
```

USBL 的适配方式：

* 时间戳匹配（merge_asof）
* 异步融合（任意频率）

---

## 5.4 Magnetometer（可选）

可用于航向观测：

```
z_mag = yaw + noise
```

---

# **6. 时间同步（Time Sync）**

多传感器融合系统最难的部分之一。

你的系统采用双时间基：

```
MonoNS（用于对齐）  
EstNS（用于可读时间）
```

ESKF 中的策略：

* 所有传感器在离线阶段对齐到 DVL 的 EstS
* 在线运行时，由调度器统一分发时间戳
* 预测阶段按 IMU 间隔 dt
* 更新阶段按测量时间执行更新

---

# **7. 滤波流程（Pipeline）**

核心 ESKF 循环：

```
loop:
    1) 读取 IMU
    2) 按 IMU 执行预测步 (propagation)
    3) 若到达某传感器时间戳：
          执行对应 Kalman 更新
    4) 输出最新状态
```

流程图：

```
IMU (50-200Hz) → 预测
DVL (1-10Hz) → 速度更新
Depth (5-20Hz) → 深度更新
USBL (0.1-2Hz) → 位置更新
```

---

# **8. 噪声矩阵（Noise Matrices）**

---

## 8.1 过程噪声 Q

包含：

* 加速度噪声
* 角速度噪声
* 加速度偏置随机游走
* 陀螺偏置随机游走

推荐结构：

```
Q = blockdiag(
    σ_a^2 I3,
    σ_g^2 I3,
    σ_ba^2 I3,
    σ_bg^2 I3
)
```

---

## 8.2 测量噪声 R

按传感器特性设置：

| 传感器   | 噪声特征                 |
| ----- | -------------------- |
| DVL   | σ_v ≈ 0.003–0.01 m/s |
| depth | σ_z ≈ 0.01–0.05 m    |
| USBL  | σ ≈ 0.1–0.5 m（视环境）   |

---

# **9. 离线实现（Python ESKF）**

文件建议：

```
uwnav/fusion/eskf_python.py
```

用途：

* 实验数据验证
* 参数调优
* 门控调试
* 观测模型快速迭代
* 训练数据生成

与 `multisensor_postproc.py` 直接相连。

---

# **10. 在线实现（C++ / pybind11）**

未来目标：在 OrangePi / STM32 / Pixhawk 上运行。

结构：

```
eskf.cpp
eskf.h
eskf_wrapper.cpp  ← pybind11 用于 Python 调用
```

在线实现关键点：

* 固定点精度（保持数值稳定）
* 限制 sqrt / exp map 调用次数
* 优化矩阵维度（大部分矩阵是稀疏结构）
* 与采集线程隔离，使用消息队列传入测量

---

# **11. 门控（Mahalanobis Gating）**

避免 DVL / USBL 偶发跳变污染状态。

对于测量残差：

```
r = z - h(x)
D = rᵀ S⁻¹ r
```

若：

```
D > χ²(df, p)
```

则拒绝该观测。

---

# **12. 对齐数据在 ESKF 中的使用**

从 `aligned_imu_dvl_volt_YYYY-MM-DD.csv` 中取：

* `Acc_filt` + `Gyro_filt` → 预测
* `DVL velocity` → 速度更新
* `Volt` → （可用于动力学建模配合控制器）
* `Depth`（未来）
* `USBL`（未来）

该数据集就是离线验证 ESKF 的标准输入。

---

# **13. 可视化与调试**

调试 ESKF 时建议输出：

* 速度预测 vs DVL 测量
* 深度预测 vs 深度传感器
* USBL vs 滤波器历史轨迹
* 误差状态范数（||δx||）
* 残差（innovation）
* K 增益大小
* 门控拒绝次数

---

# **14. 未来扩展（Roadmap）**

| 阶段      | 内容                          |
| ------- | --------------------------- |
| Phase 1 | IMU + DVL ESKF（离线完成）        |
| Phase 2 | 加 Depth 更新                  |
| Phase 3 | 加 USBL 观测                   |
| Phase 4 | 在线 C++ ESKF                 |
| Phase 5 | 与 MPC 控制器闭环结合               |
| Phase 6 | 结合深度学习（Hybrid Dynamics）改进模型 |
| Phase 7 | 水池 + 海试验证                   |

---

# **15. 总结（Summary）**

ESKF 是本项目 **水下导航定位层的核心模块**。
它将多传感器数据统一为：

```
稳定可控的 状态估计 p, v, q, b_a, b_g
```

为你未来的：

* 动力学建模
* 控制器（MPC / RL）
* SLAM
* 轨迹跟踪系统

提供可靠的底座。

---