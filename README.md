
---

# Underwater-robot-navigation — 传感器通讯与导航框架（IMU + DVL 起步）

该分支旨在搭建水下机器人 **传感器通讯与导航管线** 的基础框架，
当前已完成 **IMU（RS-485 / Modbus-like）** 读写闭环，正在集成 **DVL（Hover H1000 / PD6 协议）** 通讯解析。
未来将逐步集成 **USBL**、**Depth**、**时间同步** 和 **ESKF 融合** 等模块，打造一个完整的水下导航系统。

---

## 🎯 项目目标

* 打通传感器 **通讯 → 解析 → 时间戳对齐 → 记录 → 融合** 的完整链路。
* 实现统一的 **数据接口规范**（时间、坐标系、单位、健康度标志等）。
* 构建可扩展的 **组合导航框架**，支持 IMU + DVL + USBL + Depth 多源数据融合。
* 最终并入主控层，实现 **自主悬停** 与 **轨迹跟踪控制**。

---

## 📦 当前进展

| 模块                            | 状态     | 说明                                                           |
| ----------------------------- | ------ | ------------------------------------------------------------ |
| **IMU（RS-485）**               | ✅ 完成   | 串口通信、寄存器读写（0x03 / 0x06）、CRC 校验、Acc/Gyro/Mag/Euler 解算         |
| **IMU 数据处理**                  | ✅ 完成   | 实时滤波（RealTimeIMUFilter）、CSV 记录、异常与丢包检测                       |
| **DVL（Hover H1000 / PD6 协议）** | 🚧 进行中 | PD6/EPD6 协议解析、TCP/串口通信模块开发中                                  |
| **DVL 数据结构化**                 | 🚧 进行中 | 输出 `DVLData`（vel_body / vel_earth / height / valid_flag）统一接口 |
| **USBL / Depth**              | ⏳ 计划中  | 定位与深度数据采集接口                                                  |
| **ESKF 融合模块**                 | ⏳ 计划中  | IMU + DVL + Depth / USBL 融合、时间同步、门控更新                        |
| **统一记录与时间同步**                 | 🚧 进行中 | CSV 滚动写入、时间对齐、空闲预警机制                                         |

---

## ⚙️ 主要功能特性

### 🧭 IMU 模块

* **接口**：RS-485 / Modbus（波特率 230400，默认地址 0x50）

* **数据**：Acc, Gyro, Mag, Euler

* **输出格式**：CSV（原始数据 + 滤波后数据）

* **使用示例**：

  ```bash
  python apps/acquire/imu_logger.py --port /dev/ttyUSB0 --baud 230400
  ```

* **配置文件**：`config/devices/imu.yaml`

  ```yaml
  imu:
    port: "/dev/ttyUSB0"
    baudrate: 230400
    installation_angle: 0.0
    units:
      angles: "radians"
      acceleration: "g"
    range:
      acceleration_max: 4.0
  ```

---

### 🌊 DVL 模块（Hover H1000）

* **协议**：标准 PD6 / EPD6 ASCII 文本协议

* **接口**：串口（115200 8N1）或以太网 TCP（默认 `192.168.2.102`，命令端口 `10000`，数据端口 `10001`）

* **输出频率**：1–20 Hz

* **主要语句**：

  | 语句   | 内容                   | 坐标系         | 说明          |
  | ---- | -------------------- | ----------- | ----------- |
  | `SA` | Pitch, Roll, Heading | 设备坐标        | 姿态角信息       |
  | `TS` | 时间戳, 声速, 盐度等         | -           | 系统状态信息      |
  | `BI` | VX, VY, VZ           | 体坐标系        | 对底速度（mm/s）  |
  | `BE` | VE, VN, VU           | 地理坐标系 (NED) | 投影速度（mm/s）  |
  | `BD` | E, N, U, Depth       | 地理坐标系       | 累计位移 / 对底高度 |

* **示例帧**：

  ```
  :SA, -1.76, 0.07, 322.57
  :BE, -1532, 1945, -42, A
  :BD, 0.00, 0.00, 0.00, 39.08, 0.00
  ```

* **配置文件**：`config/devices/dvl_hover_h1000.yaml`

  ```yaml
  dvl_hover_h1000:
    interface:
      type: "ethernet"
      ip_address: "192.168.2.102"
      data_port: 10001
      command_port: 10000
    measurement:
      frequency: 20
      range:
        min: 0.05
        max: 45.0
      beams: 4
    data_format:
      selected: "EPD6"
  ```

* **解析文档**：`docs/protocols/HoverH1000_PD6.md`

* **采集脚本**：

  ```bash
  python apps/acquire/dvl_logger.py --ip 192.168.2.102 --port 10001
  ```

* **输出示例**（转换为 m/s）：

  ```
  [12:00:01.123] BE: [0.152, -0.004, -0.021] m/s | Height=2.43 m | Flag=A
  ```

---

## 🧩 目录结构简要

```
Underwater-robot-navigation/
├─ README.md
├─ pyproject.toml
├─ config/
│  ├─ devices/
│  │  ├─ imu.yaml
│  │  └─ dvl_hover_h1000.yaml          ← DVL 配置（串口/网口、频率、量程、格式）
│  ├─ fusion/
│  │  └─ eskf.yaml
│  └─ lever_arm.yaml
├─ data/                                 ← 所有采集数据移到这里（按日期归档）
│  └─ 2025-11-05/
├─ logs/
├─ docs/
│  ├─ protocols/
│  │  └─ HoverH1000_PD6.md             ← DVL 协议解析与说明（开发文档）
│  └─ sensors/
│     └─ dvl_hover_h1000_setup.md      ← 安装/接线/参数示例（面向使用）
├─ apps/
│  ├─ acquire/                          ← 只做“读取与落盘”，不做融合
│  │  ├─ imu_logger.py                  ← 你的脚本（已优化版本），移动到此
│  │  ├─ power_logger.py                ← 原 volt32_logger.py
│  │  ├─ dvl_logger.py                  ← DVL 采集→CSV（新增）
│  │  └─ usbl_logger.py                 ← 预留
│  ├─ pipelines/                        ← 读取→融合→发布（在线导航）
│  │  ├─ realtime_nav.py
│  │  └─ nav_fusion_demo.py
│  ├─ tools/
│  │  ├─ tmux_telemetry_manager.py
│  │  └─ data_replay.py
│  └─ examples/
│     ├─ imu_quickstart.py
│     └─ dvl_quickstart.py
├─ uwnav/
│  ├─ __init__.py
│  ├─ drivers/                          ← “硬件/协议/收发”层（不做坐标变换/融合）
│  │  ├─ imu/
│  │  │  └─ WitHighModbus/              ← 你现有 IMU 驱动移到 imu/ 子目录
│  │  └─ dvl/
│  │     └─ hover_h1000/
│  │        ├─ __init__.py
│  │        ├─ protocol.py              ← PD6/EPD6 解析 + 16字节命令组帧（无 I/O）
│  │        ├─ serial_if.py             ← 串口收发（行缓冲→protocol.parse_line）
│  │        └─ tcp_if.py                ← TCP 收发（10000/10001→protocol）
│  ├─ sensors/                          ← “设备抽象层”（统一单位/时间戳/健康度）
│  │  ├─ __init__.py
│  │  ├─ imu.py
│  │  └─ dvl_hover_h1000.py            ← 调用 drivers.dvl.hover_h1000.*，输出 DVLData
│  ├─ fusion/
│  │  ├─ __init__.py
│  │  ├─ eskf.py
│  │  ├─ process_models.py
│  │  └─ meas_models/
│  │     ├─ __init__.py
│  │     └─ dvl.py                      ← DVL 的观测模型 H/R/门控
│  ├─ nav/
│  │  ├─ __init__.py
│  │  ├─ frames.py
│  │  └─ state_def.py
│  ├─ io/
│  │  ├─ __init__.py
│  │  ├─ recorder.py
│  │  └─ loader.py
│  └─ preprocess/
│     ├─ __init__.py
│     ├─ calibrations.py
│     └─ align.py
└─ tests/
   ├─ drivers/
   │  └─ test_pd6_parser.py             ← 纯协议单测（喂厂家/实测样本）
   ├─ integration/
   │  └─ test_dvl_end_to_end.py         ← TCP/串口→协议→传感器对象→结构化输出
   └─ golden/
      └─ dvl_pd6_samples.txt            ← 采样金样本（回归用）
```
模块化落地期望：
uwnav/
  core/                      ← C++ 核心实时库
    include/uwnav/...
    src/
      imu_driver.cpp         ← 串口读、解包、时间戳（100Hz）
      dvl_driver.cpp         ← 串口读、定宽解析、A/V判定（10Hz）
      eskf.cpp               ← 状态传播+观测更新（200Hz主循环+事件更新）
      time_sync.cpp          ← 单机时钟、统计串口延迟
      pub_ctrl_iface.cpp     ← 50Hz 发布估计结果/对接 MAVLink
      log_ringbuf.cpp        ← 环形缓冲日志（零拷贝、低抖动）
  apps/
    nav_daemon/              ← C++ 可执行（服务）
      main.cpp               ← 读配置、起线程、注册回调、优雅退出
  tools/                     ← Python 工具（保留现有）
    dvl_data_verifier.py
    imu_logger.py
    plot_*.py
  drivers/
    dvl/hover_h1000/         ← 现有 Python 驱动保留做工具
    imu/WitHighModbus/
  config/
    nav.yaml                 ← 频率、端口、滤波器Q/R、DVL阈值、占位策略窗口等

---

## 🧠 开发计划

| 阶段          | 内容                          | 状态     |
| ----------- | --------------------------- | ------ |
| **Phase 1** | IMU 通信与滤波闭环                 | ✅ 完成   |
| **Phase 2** | DVL (Hover H1000) PD6 通信与解析 | 🚧 开发中 |
| **Phase 3** | DVL + IMU 同步记录与时间戳对齐        | ⏳ 计划   |
| **Phase 4** | 融合算法（ESKF）与姿态/位置估算          | ⏳ 计划   |
| **Phase 5** | 加入 USBL 与 Depth 模块          | ⏳ 计划   |
| **Phase 6** | 实池测试与多传感器标定                 | ⏳ 计划   |

---

## ⚡ 快速依赖安装

```bash
pip install -U pyserial numpy
# DVL TCP 模式需要
pip install socket
# 日志记录与 CSV
pip install pandas
```

---

## 📚 参考文档

* 《海底鹰 Hover H1000 用户手册（Ver 1.2）》
* `docs/protocols/HoverH1000_PD6.md` – PD6 协议结构与字段定义
* `config/devices/dvl_hover_h1000.yaml` – DVL 通信配置
* `config/devices/imu.yaml` – IMU 配置与单位说明

---