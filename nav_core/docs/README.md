
---

# `nav_core/README.md`

# nav_core – Underwater Navigation Core (C++)

`nav_core` 是整个 **Underwater-robot-navigation** 项目中最核心的 **实时导航内核（Real-Time Navigation Core）**。
它运行在 Orange Pi 等嵌入式平台上，通过 C++ 高性能读取 IMU、DVL、电压传感器数据，并进行实时 ESKF 状态估计，生成高频导航状态输出。

相比 Python 侧的 `uwnav`（数据采集、可视化、算法原型），`nav_core` 是真正用于 **部署、上船、实时控制** 的部分。

---

## 1. 功能总览（核心价值）

`nav_core` 在系统中的角色：

```
传感器  →  nav_core (C++)  →  导航状态  →  控制器 (MPC/RL/PID/…)

IMU (Modbus-RTU)
DVL (PD6 / E/D ベロシティ)
Volt/Current Sensor
```

nav_core 的主要职责：

### ✔ 1) 实时读写传感器

* IMU (WitMotion HWT9073-485, Modbus/RS485)
* DVL (Hover H1000, PD6/EPD6 协议)
* Volt driver（用于记录系统电气状态）

### ✔ 2) 时间同步

统一使用 `nav_core/timebase.h` 提供的：

* 单调时间戳（monotonic ns）
* 系统估计时间（est ns）

### ✔ 3) 数据预处理 & 过滤

各驱动内部提供基础过滤：

* 异常尖刺过滤
* 有效位（IMU raw）
* 速度有效性（DVL A/V）
* 阈值保护（避免对 ESKF 造成污染）

### ✔ 4) 扩展卡尔曼滤波器（ESKF）

* 目前已包含 ESKF 基础结构（待进一步完善）
* 负责融合 IMU + DVL + 深度计（未来）
* 输出：位置、速度、姿态、偏置等

### ✔ 5) 导航守护进程（nav_daemon）

编译后生成可执行程序：

```
uwnav_navd
```

负责：

* 加载配置
* 启动各驱动
* 运行 ESKF
* 输出导航数据
* 写入二进制日志（bin_logger）

---

## 2. 目录结构

```
nav_core/
    ├── CMakeLists.txt                       # nav_core 构建脚本
    │
    ├── include/
    │   └── nav_core/                        # 对外暴露的头文件（对外 API 只从这里 include）
    │       ├── imu_types.h                  # IMU / DVL / ESKF 用到的基础类型（ImuFrame 等，若已有）
    │       ├── imu_driver_wit.h             # Wit IMU 驱动
    │       ├── dvl_driver.h                 # DVL 驱动
    │       ├── eskf.h                       # ESKF 状态估计
    │       ├── imu_rt_filter.h              # 实时 IMU 滤波 / 航向积分
    │       ├── bin_logger.h                 # 二进制日志读写工具
    │       ├── timebase.h                   # 导航时间基 / 时间戳工具
    │       ├── log_packets.h                # ★ IMU / DVL / ESKF 日志包结构（共享给写入/解析）
    │       └── nav_state_publisher.h        # ★ 导航状态发布接口（对控制进程/其它模块暴露的出口）
    │
    ├── src/
    │   ├── nav_daemon.cpp                   # 主程序 uwnav_navd（导航守护进程）
    │   ├── timebase.cpp                     # 时间基实现
    │   ├── imu_driver_wit.cpp               # IMU 驱动实现
    │   ├── dvl_driver.cpp                   # DVL 驱动实现
    │   ├── eskf.cpp                         # ESKF 实现
    │   ├── bin_logger.cpp                   # 二进制日志实现
    │   ├── imu_rt_filter.cpp                # 实时 IMU 滤波 / 航向积分实现
    │   ├── nav_state_publisher.cpp          # ★ 导航状态发布实现（写共享内存 / UDP / ZeroMQ 等）
    │   └── dump_nav_logs.cpp                # ★ 导航日志解析工具（原 dump_imu_bin.cpp 升级版，使用 log_packets.h）
    │
    └── third_party/
        └── witmotion/                       # WitMotion 官方 SDK
            ├── wit_c_sdk.c
            └── *.h
```

---

## 3. 构建方法（Linux / Orange Pi）

### 3.1 安装依赖

```
sudo apt-get install build-essential cmake
```

### 3.2 编译 nav_core

```bash
cd nav_core
mkdir build && cd build
cmake ..
make -j
```

编译成功后生成：

```
uwnav_navd
libnav_core.a
```

---

## 4. 驱动说明

### 4.1 IMU（HWT9073-485，Modbus/RS485）

* 串口读取：230400 波特率
* 每 10ms 轮询一次（100Hz）
* 读取寄存器：`WitReadReg(AX, 16)`
* 寄存器解析：`wit_c_sdk.c`
* 输出：角速度、线加速度、欧拉角、温度

---

### 4.2 DVL（Hover H1000，PD6/EPD6 协议）

* 行协议解析
* 自动识别 BE / WI / SI 等速度帧类别
* 有效位（A/V）过滤
* 输出：ve, vn, vu (m/s)

---

### 4.3 Volt Driver (ADC/Modbus/自定义协议)

* 读取系统电压、电流
* 用于日志记录与后续功率消耗分析

---

## 5. 配置文件（由 nav_daemon 读取）

未来将在：

```
config/devices/imu.yaml
config/devices/dvl_hover_h1000.yaml
config/fusion/eskf.yaml
config/lever_arm.yaml
```

统一管理 IMU/DVL/ESKF 的参数。

---

## 6. nav_daemon（核心入口）

编译后运行：

```bash
./uwnav_navd
```

负责：

1. 打开 IMU、DVL、Volt 设备
2. 读取配置
3. 启动 ESKF
4. 输出导航状态
5. 写入日志（CSV / 二进制）

未来会加入：

* UDP 广播
* ZeroMQ
* pipes / shared-memory
  以供上层控制器（MPC/RL）读取。

---

## 7. Python 与 nav_core 的关系

```
Python(uwnav) → 采集、可视化、原型算法
C++(nav_core) → 实船实时导航
```

Python 侧可以读取 C++ 输出的 `.bin` 或 `.csv` 数据文件进行可视化、训练神经网络、调试 ESKF 参数。

---

## 8. 下一步计划（Roadmap）

* [ ] 完成 ESKF 状态向量结构
* [ ] 增加深度计（Depth Sensor）驱动
* [ ] 增加 USBL 驱动
* [ ] 增加 DVL + IMU 同步对齐逻辑
* [ ] 增加 nav_daemon → 控制器的实时数据管道（UDP/ZMQ）
* [ ] 增加 bin_logger 高性能写盘（环形 buffer）

---

# Conclusion

这个 README 的作用：

* 让新成员能快速理解 nav_core 在整个系统中的定位
* 掌握驱动/目录/构建方式
* 理解 IMU 与 DVL 的 C++ 实现
* 为未来的导航融合（ESKF）奠定结构基础

