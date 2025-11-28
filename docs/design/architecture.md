
---

# **Underwater Robot Navigation System — Architecture Overview**

**水下机器人多传感器导航系统 · 总体架构说明文档**

---

## 1. 文档目的（Purpose）

本文件旨在向项目新成员快速解释：

1. 整个水下导航系统的总体架构
2. 各功能模块之间是如何协作的
3. 数据流在系统中的流动路径
4. 如何开发、扩展与调试当前工程



---

## 2. 系统总览（System Overview）

本系统是一个 **多传感器水下定位框架**，承担水下机器人试验中的：

* 传感器数据采集（IMU / DVL / Volt / Depth / USBL）
* 时间同步（统一的 MonoNS / EstNS）
* 数据归档与清洗
* 离线融合与可视化
* 为后续 ESKF、深度学习、控制算法提供标准数据接口

整个结构由 **采集层 → 存储层 → 后处理层 → 融合层 → 控制层** 组成。

```
┌────────────────────┐
│   Sensors 层       │  IMU / DVL / Volt / Depth / USBL
└────────────┬────────┘
             │
┌────────────┴────────┐
│   Acquire Layer      │  apps/acquire/*
│   (多传感器采集层)   │  IMU/DVL/Volt 统一时间基采集
└────────────┬────────┘
             │
┌────────────┴──────────┐
│   Data Storage Layer   │  data/YYYY-MM-DD/{imu,dvl,volt}
│   (按日期 + 传感器分类) │
└────────────┬──────────┘
             │
┌────────────┴──────────┐
│ Post-Processing Layer  │  apps/tools/multisensor_postproc.py
│ (对齐/插值/可视化/融合) │
└────────────┬──────────┘
             │
┌────────────┴──────────┐
│   Fusion Layer         │  ESKF / Offline KF / 门控 / 多传感器融合
│   uwnav/fusion/*       │
└────────────┬──────────┘
             │
┌────────────┴──────────┐
│ Control & Estimation   │  MPC / RL / System ID / Dynamics Fit
│ (未来扩展)             │
└────────────────────────┘
```

---

## 3. 目录结构（Repository Structure）

```
Underwater-robot-navigation/
├── apps/
│   ├── acquire/                  ← 传感器采集脚本
│   └── tools/                    ← 后处理、tmux管理器等
│
├── uwnav/
│   ├── drivers/                  ← IMU/DVL驱动
│   ├── sensors/                  ← 数据对象 IMUData / DVLData
│   ├── fusion/                   ← ESKF / 观测模型
│   ├── preprocess/               ← 标定、滤波
│   └── io/                       ← 时间基、CSV工具
│
├── config/                       ← 设备配置文件
├── docs/                         ← 文档目录
│   └── overview/architecture.md  ← 当前文件
└── data/                         ← 实验数据（按日期归档）
```

---

## 4. 采集层（Acquire Layer）

采集层由三个核心脚本组成，每个脚本独立运行，负责：

* 与传感器通讯（串口/网口）
* 接收原始数据
* 按统一时间基记录（MonoNS / EstNS）
* 写入对应的传感器目录

### 4.1 IMU 采集（HWT9073-485）

文件：`apps/acquire/imu_realtime_pipeline.py`

特性：

* 高频串口采集（230400 baud）
* 滤波处理（低通）
* 自动保存 raw + filtered CSV
* 时间基：将接收时刻标记为 EstS

### 4.2 DVL 采集（Hover H1000）

文件：`apps/acquire/dvl_data_verifier.py`

特性：

* 支持 PD6 / EPD6 解析
* 自动下发 DF/PR/PM/ST/CS/CZ 指令
* 保存双时间戳数据：

  * `MonoNS` 机器人单调时钟
  * `EstNS` 系统估计时间
* 为后处理提供可靠的参考时间轴

### 4.3 电压/电流板采集（Volt32）

文件：`apps/acquire/volt32_logger.py`

特性：

* 16/32 路电压/电流采集
* 自动滚动文件（例如 50 MB）
* 定期 flush & 7 天旧文件自动清理

---

## 5. 多传感器数据存储结构（Data Storage Layer）

采集脚本会自动将数据分类存储到：

```
data/YYYY-MM-DD/imu/
data/YYYY-MM-DD/dvl/
data/YYYY-MM-DD/volt/
data/YYYY-MM-DD/aligned/
```

严格按照：

* 日期
* 传感器类别

进行组织，使得：

* 多天实验互不干扰
* 便于后处理自动扫描文件
* 方便训练深度模型、标定、实验管理

目录举例：

```
data/2025-11-27/
    imu/       ← IMU raw + filtered
    dvl/       ← DVL 最小速度表
    volt/      ← 电压/电流数据
    aligned/   ← 处理与对齐后的融合数据
```

---

## 6. 后处理层（Post-Processing Layer）

核心脚本：
`apps/tools/multisensor_postproc.py`

功能：

1. 自动查找指定日期的 IMU / DVL / Volt 数据
2. 加载 CSV → 转换为 DataFrame
3. 基于 DVL 时间轴（EstS）对齐 IMU / Volt
4. 输出融合数据表
5. 自动绘制多种图像：

   * IMU 加速度趋势
   * DVL 东北上速度
   * 电压/电流趋势
6. 可选保存对齐后的 CSV 至：

```
data/YYYY-MM-DD/aligned/
```

这是后续 ESKF、深度学习模型（LSTM、混合动力学网络）、系统鉴定的重要输入。

---

## 7. 融合层（Fusion Layer）

`uwnav/fusion/` 包含：

* ESKF（Error-State Kalman Filter）
* 多传感器观测模型（IMU + DVL + Depth + USBL）
* 时间同步逻辑
* 门控（Mahalanobis gating）
* 离线与在线版本（未来在线版将以 pybind11 调用 C++ 实现）

功能目标：

* 对齐的多传感器数据
* → 状态估计（位置 / 速度 / 姿态 / 偏置）
* → 控制器可直接使用

---

## 8. 控制与系统鉴定层（Control & System Identification）

后续任务包括：

* 轨迹跟踪控制（MPC / iMPC / Sliding-Mode MPC）
* 动力学参数辨识（基于 Volt + DVL + IMU）
* 神经网络动力学模型（LSTM / 物理增强网络）
* RL 强化学习控制器（Actor-Critic）

这些模块不在此文档范围内，但本导航框架为其提供：

* 多源高质量数据
* 对齐后的统一格式
* 状态估计输入

保证控制层具有“测量完整性”。

---

## 9. 工具层（Tools Layer）

工具文件位于：

```
apps/tools/
```

核心工具包括：

### 9.1 tmux 多窗口实时采集管理器

`tmux_telemetry_manager.py`

特点：

* 一条指令启动 IMU / DVL / Volt 三个采集窗口
* 自动配置端口/波特率
* 不需要 sudo
* 支持优雅停止机制

### 9.2 multisensor_postproc.py

三传感器后处理脚本。详见上一节。

### 9.3 设备调试小工具（verifiers）

如：

* `imu_data_verifier.py`
* `dvl_data_verifier.py`

用于快速验证串口、数据格式、传感器健康状态。

---

## 10. 典型数据流（Data Flow）

```
Sensors
   │
   │ (serial/TCP data stream)
   ▼
Acquire Scripts (IMU/DVL/Volt)
   │        │
   │ 写入   │ 统一的时间基 (MonoNS + EstNS)
   ▼        ▼
data/YYYY-MM-DD/{imu,dvl,volt}
   │
   │ 离线分析
   ▼
multisensor_postproc.py
   │
   │ 插值/对齐/融合
   ▼
aligned CSV (position-ready)
   │
   │ 状态估计 / 动力学拟合
   ▼
Fusion Layer (ESKF / Learning)
   │
   ▼
Control System (MPC / RL / System ID)
```

---

## 11. 新成员学习路径（Onboarding Guide）

1. **先读本文件（architecture.md）**：了解系统全景
2. 阅读 `README.md`：掌握项目使用方式
3. 了解采集流程：

   * `tmux_telemetry_manager.py`
   * `imu_realtime_pipeline.py`
   * `dvl_data_verifier.py`
4. 运行一次实际采集 + 后处理流程
5. 查看融合数据样例
6. 如参与 ESKF/控制，再去看：

   * `uwnav/fusion/*`
   * `docs/protocols/*`
   * `docs/sensors/*`

---