
---

# **README.md – Underwater Robot Navigation System**

**(IMU + DVL + Volt Telemetry Framework · Multi-Sensor Logging & Processing Pipeline)**
升级后的正式版


---

## 1. 项目简介（Overview）

本项目构建一个 **多传感器水下导航数据链路**，覆盖：

1. **多源传感器采集**（IMU / DVL / 电压电流板 / 未来 USBL、Depth）
2. **统一时间基记录**（MonoNS / EstNS）
3. **按日期 + 传感器类型归档的数据结构**
4. **自动可视化与后处理工具**
5. **为 ESKF / LSTM / 物理建模 / MPC 控制 提供标准输入**

项目采用模块化架构：
采集脚本 → data/ 归档 → tools/ 后处理 → uwnav/ 融合逻辑。

---

## 2. 项目结构（Project Structure）

```
Underwater-robot-navigation/
├── README.md              ← 当前文档
├── pyproject.toml
├── config/
│   ├── devices/           ← 传感器配置（端口 / 波特率 / 安装角 / 数据格式）
│   ├── fusion/            ← ESKF / 门控 / 噪声
│   └── lever_arm.yaml     ← 传感器杆臂（机体系）
│
├── data/                  ← ★ 统一数据根目录（按日期归档）
│   └── YYYY-MM-DD/
│        ├── imu/
│        ├── dvl/
│        ├── volt/
│        └── aligned/      ← 后处理输出（对齐后的融合CSV）
│
├── apps/
│   ├── tools/             ← 实验工具（最常用）
│   │    ├── tmux_telemetry_manager.py
│   │    ├── dvl_data_verifier.py
│   │    ├── multisensor_postproc.py    ← ★ 新增：三传感器后处理 + 可视化
│   │    └── imu_data_verifier.py
│   └── acquire/           ← 纯采集脚本（读取 + 落盘）
│        ├── imu_realtime_pipeline.py
│        ├── dvl_data_verifier.py
│        └── volt32_logger.py
│
├── uwnav/                 ← 核心库（驱动 / 传感器抽象 / 融合 / 工具）
│   ├── drivers/           ← IMU / DVL 驱动 & 协议
│   ├── sensors/           ← 输出结构化 IMUData / DVLData
│   ├── fusion/            ← ESKF、观测模型、门控
│   ├── preprocess/
│   └── io/                ← CSV 加载 / 时间基
│
└── docs/                  ← 协议文档 / 设计说明nav_core/
├── nav_core/
│   ├─ timebase.h        # 时间基准（mono_ns / est_ns）
│   ├─ imu_types.h       # IMU 数据结构与过滤配置
│   ├─ imu_driver_wit.h  # WitMotion IMU 驱动（HWT9073-485）
│   ├─ dvl_driver.h      # Hover H1000 DVL 驱动
│   ├─ volt_driver.h     # 电压电流驱动
│   ├─ eskf.h            # 扩展卡尔曼滤波入口
│   └─ bin_logger.h      # 二进制日志记录
│
├─ src/
│   ├─ timebase.cpp
│   ├─ imu_driver_wit.cpp    # 当前核心 IMU 驱动（Modbus 230400）
│   ├─ dvl_driver.cpp
│   ├─ volt_driver.cpp
│   ├─ eskf.cpp
│   ├─ bin_logger.cpp
│   └─ nav_daemon.cpp        # 主程序：uwnav_navd
│
├─ third_party/
│   └─ witmotion/
│       ├─ REG.h             # WitMotion IMU 寄存器表
│       ├─ wit_c_sdk.h       # SDK 头文件
│       └─ wit_c_sdk.c       # Modbus 协议解析核心（厂家提供）
│
└─ CMakeLists.txt


```

---

## 3. 数据目录规范（Data Directory Standard）

项目所有传感器采集数据统一写入：

```
data/YYYY-MM-DD/<sensor>/
```

例如：

```
data/2025-11-27/
    imu/
        imu_raw_data_20251127.csv
        imu_filtered_data_20251127.csv
    dvl/
        dvl_speed_min_tb_20251127.csv
    volt/
        motor_data_20251127_1015.csv
    aligned/
        aligned_imu_dvl_volt_20251127.csv
```

此结构允许：

* 自动对齐（IMU/DVL/Volt）
* 自动按日期管理
* 多次实验清晰区分
* 与 tmux 管理器一键联动

---

## 4. 多传感器实时采集（IMU + DVL + Volt）

### **4.1 使用 tmux 管理器（一键启动三路采集）**

Linux 环境下执行：

```bash
python3 apps/tools/tmux_telemetry_manager.py \
  --imu-port /dev/ttyUSB1 --imu-baud 230400 \
  --dvl-port /dev/ttyUSB2 --dvl-baud 115200 \
  --volt-port /dev/ttyUSB0 --volt-baud 115200 \
  --attach
```

启动后自动创建三个 tmux 窗口：

```
window 0 : IMU
window 1 : DVL
window 2 : Volt
```

退出方式：

* 在管理器终端输入：`s`（优雅关闭三路传感器）
* 或在 tmux 内按 `Ctrl+C`

---

## 5. 单独采集脚本（Acquire Layer）

### **5.1 IMU — HWT9073-485**

```bash
python3 apps/acquire/imu_realtime_pipeline.py --port /dev/ttyUSB1 --baud 230400
```

特性：

* 230400 baud 高速串口
* 实时滤波（窗口可调）
* 输出 raw / filtered double CSV
* 自动归档到 `data/YYYY-MM-DD/imu/`

---

### **5.2 DVL — Hover H1000**

```bash
python3 apps/acquire/dvl_data_verifier.py --port /dev/ttyUSB2 --baud 115200
```

特性：

* PD6 / EPD6 协议解析
* 自动 downlink 发送：DF/PR/PM/ST/CS/CZ
* 输出双时间戳速度表（MonoNS + EstNS）
* 自动归档到 `data/YYYY-MM-DD/dvl/`

---

### **5.3 Volt32 — 16/32 路电压电流板**

```bash
python3 apps/acquire/volt32_logger.py --port /dev/ttyUSB0 --baud 115200
```

特性：

* 自动滚动文件（50MB）
* 7 天自动清理
* Zero-loss 实时写入
* 自动归档到 `data/YYYY-MM-DD/volt/`

---

## 6. 三传感器离线后处理（Post-Processing）

新工具：
**`apps/tools/multisensor_postproc.py`**

### 使用方式：

```bash
python3 apps/tools/multisensor_postproc.py \
  --date 2025-11-27 \
  --save-merged
```

功能：

* 自动扫描：imu / dvl / volt 文件
* 统一解析时间戳（EstS / t_s）
* 使用 merge_asof() 完成时间对齐
* 输出对齐后的融合表（aligned CSV）
* 自动绘制三类可视化图像：

  * IMU 滤波加速度三轴
  * DVL 东、北、上速度
  * Volt 前两路电压/电流趋势图

输出路径：

```
data/YYYY-MM-DD/aligned/aligned_imu_dvl_volt_YYYY-MM-DD.csv
```

---

## 7. 核心库 uwnav/（Drivers → Sensors → Fusion）

```
uwnav/
├── drivers/               ← 设备驱动（IMU 485 / DVL PD6 / TCP）
├── sensors/               ← 统一输出 IMUData, DVLData
├── fusion/                ← ESKF（python外观 + C++ 热路径）
├── preprocess/            ← 标定 / 数据对齐
└── io/                    ← 读取工具 / 时间基（MonoNS / EstNS）
```

你之后的定位融合、轨迹控制、环境感知都基于此管线构建。

---

## 8. 后续规划（Roadmap）

| 阶段      | 内容                                |
| ------- | --------------------------------- |
| Phase 1 | IMU + DVL + Volt 采集闭环（已完成）        |
| Phase 2 | USBL + 深度计接入（计划）                  |
| Phase 3 | 完整时间同步（稳态 + 对齐）                   |
| Phase 4 | 离线 ESKF（Python）与在线 ESKF（pybind11） |
| Phase 5 | 实池实验数据闭环验证（自动评估工具）                |
| Phase 6 | 与 MPC 控制集成                        |

---

## 9. 快速安装

```bash
pip install -r requirements.txt
```

---

## 10. 文档导航（Docs）

* DVL 协议说明：`docs/protocols/HoverH1000_PD6.md`
* DVL 安装手册：`docs/sensors/dvl_hover_h1000_setup.md`
* IMU 笔记：`docs/sensors/imu_hwt9073_notes.md`

---