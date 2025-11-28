---

# **Underwater Robot Navigation – Tools Layer Overview**

**工具层总体说明文档（正式版 · 2025 更新）**

---

## 1. 工具层简介

`apps/tools/` 目录包含本项目的**基础数据采集与数据后处理工具**。
工具层位于整个系统架构中最底层，承担“数据生产”的职责。其核心任务包括：

* 稳定采集多种传感器数据（IMU / DVL / 电压电流板）
* 统一时间基（MonoNS + EstNS）对齐多传感器时间
* 长时间采集能力（自动文件轮换、定期 flush）
* 多窗口采集管理（tmux 自动化）
* 实验后对原始数据进行后处理和格式化整合（multisensor_postproc）

这些工具是导航系统、动力学建模模块、控制模块、和深度学习模块的数据输入源头。

---

## 2. 目录结构（2025 最新版）

```
apps/tools/
│
├── imu_data_verifier.py              # IMU，高速采集 + raw/filt 双CSV + timebase
├── volt32_logger.py                  # 多路电压/电流采集 + RollingCSVWriter
├── dvl_data_verifier.py              # DVL PD6/EPD6 数据采集 + timebase
│
├── multisensor_postproc.py           # 多传感器数据后处理（同步/格式化/可视化）
│
└── tmux_telemetry_manager.py         # 多传感器一键启动（tmux 管理）
```

**已删除脚本**（不再提及）：

* `imu_realtime_pipeline.py`
* `sensor2_rotating.py`

---

## 3. 工具脚本详细说明（最新版）

---

## 3.1 IMU Data Verifier（统一时间基 + 原始/滤波双流）

**文件：`imu_data_verifier.py`**
**功能：IMU高速采集、滤波、结构化记录、时间统一（MonoNS / EstNS）**

适配设备：
WitMotion HWT9073-485 / WT901C-485

核心特点：

* 单调时间基：`stamp("imu0", SensorKind.IMU)`
* 原始数据记录：raw.csv
* 滤波后数据：filt.csv
* 实时滤波器（RealTimeIMUFilter）：

  * 初始静置校准（bias）
  * 加速度/角速度低通滤波
  * 航向角解缠
* 空闲超时报警（>3s）
* 回调线程轻量化 + 后台消费者线程
* SIGINT/SIGTERM 平滑退出

适用：

* 下水前 IMU 通信调试
* IMU 噪声与稳定性验证
* 动力学建模数据采集（LSTM / PhysicsNN）
* ESKF 的高频输入源

---

## 3.2 Volt32 Logger（多通道电压/电流板）

**文件：`volt32_logger.py`**
**功能：记录 16/32 路电压/电流传感器数据，适用于电机功率分析**

特点：

* RollingCSVWriter：

  * 文件大小轮换（默认 50MB）
  * 跨天轮换
  * 自动清理 7 天前日志
* CH0~CH15 自动聚合一帧再写入
* 自动解析 "CHx: value" 行
* 支持调试 raw 输出
* 后台定时 flush
* 用于电机功率–推力实验 & system identification

输出格式：

```
Timestamp, t_s_volt, CH0..CH15
```

---

## 3.3 DVL Data Verifier（Hover H1000）

**文件：`dvl_data_verifier.py`**
**功能：以 PD6/EPD6 协议解析 Hover H1000 DVL 数据**

特点：

* 统一时间基：`stamp("dvl0", SensorKind.DVL)`
* 自动执行安全指令：

  * 启动：CZ → CS
  * 停止：CZ（退出时自动发送）
* 双输出：

  * 完整 raw/parsed 日志（用于排障）
  * 精简速度表（Mono+Est 时间基，用于融合/学习）
* 速率统计、高鲁棒解析（兼容 BE/BI/BS/WE 等帧）

输出：

```
MonoNS, EstNS, SensorID, Src, East, North, Up
```

适用：

* 池试/海试的 DVL 调试验证
* ESKF 融合 IMU+DVL
* 速度数据记录与分析

---

## 3.4 MultiSensor Post-Processor（新增 · 非采集脚本）

**文件：`multisensor_postproc.py`**
**功能：多传感器原始数据的“后处理工具”**

工具层中最新加入的脚本，用于：

* IMU、DVL、Volt32 的数据**统一整理**
* 多传感器 CSV 的**时间轴对齐（MonoNS）**
* 数据清洗、缺失补齐、格式重整
* 时间同步后的联表合成（如 multi-sensor merged CSV）
* 可选绘图：加速度/角速度/速度/电流可视化
* 指定范围裁剪（如 20000–20200）
* 支持大型数据集（多段采集自动拼接）

典型任务：

```
python3 multisensor_postproc.py \
  --imu data/2025-11-27/imu/imu_raw_*.csv \
  --dvl data/2025-11-27/dvl/dvl_speed_min_tb_*.csv \
  --volt data/2025-11-27/volt/motor_data_*.csv \
  --out processed/2025-11-27/
```

它是整个“数据清洗→建模→融合”链路的关键工具。

---

## 3.5 tmux Telemetry Manager（一键运行多路采集）

**文件：`tmux_telemetry_manager.py`**

功能：

* 一键启动多个传感器（IMU / DVL / Volt）
* 自动创建 tmux session + 多窗口布局
* 串口参数自动带入
* 支持列表端口（--list-ports）
* 适合池试/现场调试

示例：

```bash
python3 tmux_telemetry_manager.py \
    --imu-port /dev/ttyUSB1 \
    --volt-port /dev/ttyUSB0 \
    --dvl-port /dev/ttyUSB2
```

---

## 4. 工具层与系统架构关系（正式版）

```
             +-----------------------------+
             |         apps/tools          |
             |  传感器数据采集与后处理层   |
             +-----------+-----------------+
                         |
        raw sensor logs  |  (IMU / DVL / Volt / USBL)
                         v
             +-----------------------------+
             |            data/            |
             |  multi-day multi-sensor     |
             +-------------+---------------+
                           |  timebase(MonoNS)
                           v
             +-----------------------------+
             |            uwnav/           |
             |  统一时间基 + 驱动 + ESKF   |
             +-------------+---------------+
                           |
                           v
             +-----------------------------+
             |        nav_core control     |
             |       MPC / RL / TrajGen    |
             +-----------------------------+
```

---

## 5. 数据目录规范（采用时间分层结构）

```
data/
  2025-11-27/
    imu/
    dvl/
    volt/
    usbl/
```

优点：

* 多传感器天然归档
* timestamp 统一 → 方便对齐融合
* 记录整洁，数据可溯源

---

## 6. 工程设计建议（更新版）

### 6.1 RollingCSVWriter 建议统一整理

放入 `uwnav.io.logging`，tools 与 acquire 层共用同一实现。

### 6.2 multisensor_postproc 作为数据入口

将所有训练/建模前的数据清洗步骤集中处理。

### 6.3 未来工具层可加入：

* USBL logger
* 摄像头帧抓取（含时间基）
* 多传感器校准工具（IMU–DVL lever arm）

---

## 7. 文档状态

本文件为 Tools Layer 的正式说明文档，适用于：

* 工程组新人
* 池试操作人员
* 数据处理人员
* 深度学习建模组
* 导航系统开发者（ESKF/MPC/RL）

---