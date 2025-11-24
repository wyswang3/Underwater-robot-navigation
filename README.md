
---

# Underwater Robot Navigation System

**多传感器水下定位导航系统（IMU + DVL + ESKF + Python/C++ 运行框架）**
**分支：`feature/IMU-DVL-serial`**

---

# 0. 本文档能帮你什么？

如果你是第一次接触本项目，这份 README 可以让你：

* 快速理解项目整体结构、分层逻辑以及传感器数据流向
* 学会在 Orange Pi 上**编译与运行 C++ 导航核心**
* 学会使用 Python 工具链**采集/校验/对齐/可视化传感器数据**
* 学会如何排查常见错误（IMU / DVL / USB / 波特率）
* 理解接下来你需要修改哪些文件（便于参与开发）

---

# 1. 项目定位（What is this system for?）

本系统是一套针对 **水下 ROV/AUV 的实时导航体系**，由 Python + C++ 两部分构成：

* **Python：数据采集（IMU/DVL）、工具、后处理、可视化、训练数据准备**
* **C++：实时运行在 SBC（Orange Pi）上的导航核心**

  * 传感器驱动（IMU / DVL）
  * 时间同步
  * 扩展卡尔曼滤波（ESKF）
  * 二进制日志写入

目标：
**在真实水池/海上环境中，为 ROV 提供实时的 6DoF 位姿、速度估计，并为后续控制系统（MPC/RL）提供可靠反馈。**

---

# 2. 项目目录结构（超清晰解说版）

```
Underwater-robot-navigation/
│
├── config/                     # YAML 配置文件（传感器、ESKF、杆臂）
│   ├── devices/                # 串口/波特率/安装角
│   ├── fusion/                 # ESKF 参数（噪声、门控）
│   └── lever_arm.yaml          # IMU/DVL/USBL 的机体系杆臂
│
├── data/                       # 所有数据写入此目录（自动按日期分类）
│   └── YYYY-MM-DD/
│        ├── imu/               # IMU 100Hz 原始数据
│        ├── dvl/               # DVL 10Hz 原始数据
│        ├── volt/              # 电流/电压数据
│        └── aligned/           # 多传感器对齐、融合后的CSV
│
├── apps/                       # Python 工具链（最常用）
│   ├── acquire/                # 采集脚本 IMU/DVL/Volt（落盘）
│   │    ├── imu_logger.py
│   │    ├── DVL_logger.py
│   │    └── Volt32_logger.py
│   ├── tools/                  # 后处理/校验/可视化/多传感器融合
│   │    ├── dvl_data_verifier.py
│   │    ├── imu_data_verifier.py
│   │    ├── multisensor_postproc.py
│   │    └── tmux_telemetry_manager.py
│   ├── pipelines/              # 多传感器自动流水线（可扩展）
│   └── examples/               # 示例
│
├── uwnav/                      # Python 版导航库（数据结构、融合、预处理）
│   ├── drivers/                # Python版 IMU/DVL 驱动
│   ├── sensors/                # IMUData / DVLData
│   ├── fusion/                 # Python版ESKF
│   ├── preprocess/             # 滤波/时间对齐
│   └── io/                     # CSV 读写
│
├── nav_core/                   # ★ C++ 实时导航核心（运行在 Orange Pi）
│   ├── include/nav_core/       # 头文件（API）
│   ├── src/                    # 所有 CPP 实现文件
│   ├── third_party/witmotion/  # WitMotion 官方 SDK
│   └── CMakeLists.txt
│
└── docs/                       # 官方文档
    ├── device_test_and_debug.md   # 上电测试与排错（非常重要）
    ├── protocols/                 # 传感器协议（DVL/IMU）
    ├── sensors/                   # 安装说明
    └── design/                    # 导航架构说明
```
---

# 3. Python 环境安装（在 PC 或 Orange Pi 都可）

推荐使用 Python ≥ 3.8：

```bash
pip install numpy pyserial pyyaml
```

安装项目本体（可选）：

```bash
pip install -e .
```

---

# 4. C++ 导航核心编译（在 Orange Pi 上）

### 4.1 安装依赖

```bash
sudo apt update
sudo apt install -y git build-essential cmake g++ pkg-config
```

### 4.2 编译 nav_core

```bash
cd nav_core
mkdir build && cd build
cmake ..
make -j4
```

生成：

* `uwnav_navd` → 主程序
* `libnav_core.a` → C++ 库

---

# 5. 运行：IMU / DVL / 全链路（实时导航）

### 5.1 测试 IMU

```bash
sudo ./uwnav_navd \
  --imu-port /dev/ttyUSB0 \
  --imu-baud 230400 \
  --dvl-port /dev/null \
  --log-dir ./data
```

### 5.2 测试 DVL

**注意：必须在水中运行！空转不得超过 2 秒！**

```bash
sudo ./uwnav_navd \
  --imu-port /dev/null \
  --dvl-port /dev/ttyUSB1 \
  --dvl-baud 115200 \
  --log-dir ./data
```

### 5.3 全链路导航 IMU + DVL + ESKF

```bash
sudo ./uwnav_navd \
  --imu-port /dev/ttyUSB0 \
  --imu-baud 230400 \
  --dvl-port /dev/ttyUSB1 \
  --dvl-baud 115200 \
  --log-dir ./data
```

日志文件将自动写入：

```
data/YYYY-MM-DD/
    imu.bin
    dvl.bin
    eskf.bin
```

---

# 6. Python 后处理：对齐 + 可视化 + 融合

完成实验后运行：

```bash
python apps/tools/multisensor_postproc.py
```

你会得到：

```
aligned/
    fusion_imu_dvl.csv
    plots/      ← 自动生成可视化
```

这一步对于：

* 深度学习动力学辨识（LSTM/Transformer）
* 控制系统（MPC/AMPC）
* 误差分析
* 轨迹跟踪评估

都非常关键。

---

# 7. 上电测试指南（强烈建议新人阅读）

文档位置：

```
docs/device_test_and_debug.md
```

内容包括：

* DVL 必须在水中运行的原因
* IMU RS485 线序校验方法
* 波特率常见错误排查
* 如何查看串口原始数据
* ESKF 为什么没有输出
* 如何验证二进制日志文件格式

> **这是新人上手必读的文档之一。**

---

# 8. 如何扩展（未来开发路线图）

| 模块                 | 状态    | 说明                   |
| ------------------ | ----- | -------------------- |
| IMU 驱动（Modbus）     | 完成    | 100Hz, 稳定            |
| DVL 驱动（PD6）        | 完成    | 多帧解析                 |
| ESKF               | 完成基础版 | 可进一步扩展偏差估计           |
| Python 数据流水线       | 完成    | 支持对齐分析               |
| USBL 模块            | 待接入   | 数据解析器设计中             |
| 深度传感器接口            | 待接入   |                      |
| PX4 / ArduSub 控制闭环 | 计划    |                      |
| 水动力学辨识（深度学习）       | 进行中   | LSTM/Hybrid Dynamics |
| MPC 控制器            | 计划适配  | C++/PX4 接口           |

---

# 9. 常见问题（FAQ）

### Q1：为什么 DVL 必须在水中运行？

因为换能器在空气中无法建立声场，会导致过热损坏。

### Q2：IMU 数据全是 0 或者 NAN？

通常是 RS485 A/B 线反了。

### Q3：为什么 ESKF 没有输出？

可能 IMU 数据无效、未移动设备、或缺少第一帧初始化。

### Q4：为什么 DVL 解析失败？

可能收到的帧不是 PD6 格式，请用：

```bash
sudo cat /dev/ttyUSB1
```

查看原始输出。

---

# 10. 贡献与开发指南

欢迎提交 PR / Issue。
项目鼓励：

* 模块化开发
* 文档完善
* 传感器驱动补充
* 控制算法集成
* 深度学习动力学建模

---