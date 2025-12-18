
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
.
├── apps
│   ├── acquire
│   │   ├── DVL_logger.py
│   │   ├── imu_logger.py
│   │   └── Volt32_logger.py
│   ├── __init__.py
│   └── tools
│       ├── dvl_data_verifier.py
│       ├── imu_baud_switch.py
│       ├── imu_data_verifier.py
│       ├── imu_xy_zero.py
│       ├── multisensor_postproc.py
│       ├── tmux_telemetry_manager.py
│       └── volt32_data_verifier.py
├── config
│   └── devices
│       ├── DVL.yaml
│       └── imu.yaml
├── data
├── docs
│   ├── 传感器读取程序
│   ├── 命令行
│   ├── design
│   │   ├── architecture.md
│   │   ├── dataset_spec.md
│   │   └── eskf_design.md
│   ├── protocols
│   │   ├── HoverH1000_PD6.md
│   │   └── imu_witmotion_modbus.md
│   ├── sensors
│   │   ├── dvl_hover_h1000_setup.md
│   │   └── imu_hwt9073_notes.md
│   ├── timebase_spec.md
│   └── tools_overview.md
├── git_auto.py
├── git-auto.sh
├── nav_core
│   ├── CMakeLists.txt
│   ├── docs
│   │   ├── device_test_and_debug.md
│   │   ├── filters
│   │   ├── nav_core_runtime.md
│   │   └── update_guidelines.md
│   ├── include
│   │   └── nav_core
│   ├── README.md
│   ├── src
│   │   ├── bin_logger.cpp
│   │   ├── dvl_driver.cpp
│   │   ├── eskf.cpp
│   │   ├── imu_driver_wit.cpp
│   │   ├── imu_rt_filter.cpp
│   │   ├── nav_daemon.cpp
│   │   ├── nav_state_publisher.cpp
│   │   └── timebase.cpp
│   └── third_party
│       └── witmotion
├── nav_core.zip
├── pyproject.toml
├── README.md
├── requirements.txt
├── shared
│   └── msg
│       └── nav_state.hpp
├── tools
│   ├── project_size_report.py
│   └── project_size_report.txt
├── uwnav
│   ├── drivers
│   │   ├── dvl
│   │   ├── imu
│   │   ├── __init__.py
│   │   └── __pycache__
│   ├── fusion
│   │   ├── __init__.py
│   │   └── meas_models
│   ├── __init__.py
│   ├── io
│   │   ├── data_paths.py
│   │   ├── __init__.py
│   │   ├── __pycache__
│   │   └── timebase.py
│   ├── nav
│   │   └── __init__.py
│   ├── preprocess
│   │   └── __init__.py
│   ├── __pycache__
│   │   ├── __init__.cpython-312.pyc
│   │   └── __init__.cpython-38.pyc
│   └── sensors
│       ├── dvl_hover_h1000.py
│       ├── imu.py
│       ├── __init__.py
│       └── __pycache__
└── uwnav.egg-info
    ├── dependency_links.txt
    ├── entry_points.txt
    ├── PKG-INFO
    ├── requires.txt
    ├── SOURCES.txt
    └── top_level.txt
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