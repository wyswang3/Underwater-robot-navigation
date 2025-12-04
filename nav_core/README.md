# `nav_core/README.md`

# `nav_core` – Underwater Navigation Core (C++)

`nav_core` 是 **Underwater-robot-navigation** 项目的实时导航核心模块，负责运行在 **Orange Pi** 等嵌入式平台上，提供 **高频传感器读取、时间同步、多传感器融合（ESKF）、实时状态输出**。它向机器人控制系统提供稳定、低延迟、结构化的导航状态。

与 Python 侧的 `uwnav`（采集、可视化、离线处理、算法验证）不同，`nav_core` 负责：

* 上船运行
* 与控制器并行工作
* 提供高可靠、高实时性的导航解算

`nav_core` 是整套水下机器人系统的导航“发动机”。

---

# 1. 功能总览

`nav_core` 的数据流：

```
           +-----------------------------+
Sensors →  |   nav_core (C++ real-time)  | → Navigation State → Controller
(IMU/DVL)  +-----------------------------+
```

核心功能：

### ✔ 1) 高性能传感器驱动

* **IMU (WitMotion HWT9073-485)**

  * Modbus-RTU，230400bps，100Hz
* **DVL (Hover H1000 PD6/EPD6)**

  * 行协议解析，自动识别 velocity frame
* 可扩展电压/电流传感器

提供：

* 异常尖刺过滤
* 有效性检查（IMU valid flag / DVL A/V 标志）
* 基于配置的阈值保护

---

### ✔ 2) 时间同步（Timebase）

所有模块统一使用：

* `mono_ns`（单调时钟，纳秒）
* `est_ns`（融合后时间戳，用于 IMU-DVL 对齐）

由 `timebase.h/.cpp` 管理。

---

### ✔ 3) 数据预处理（Real-Time Filters）

IMU 侧提供：

* 滤波、限幅
* Yaw 低频观测
* 重力系加速度转换
* 与 ESKF 的同步接口

DVL 侧提供：

* A/V 标志过滤
* 速度限幅
* 质控过滤规则

---

### ✔ 4) 扩展卡尔曼滤波（ESKF）

模块 `eskf.*` 提供基础的 15/18 维状态：

* 位置
* 速度
* 姿态（Euler or quaternion）
* 陀螺 / 加速度偏置（bias）

融合：

* IMU 高频预测
* DVL 速度更新
* 后续可加入深度计、USBL、磁力计等

这是整个导航系统的核心算法部分。

---

### ✔ 5) 二进制日志系统（bin_logger）

写入高频数据：

```
imu.bin   → ImuLogPacket
dvl.bin   → DvlLogPacket
eskf.bin  → EskfLogPacket
```

结构统一由 `log_packets.h` 定义，Python 与 C++ 完全可读。

后处理工具 `dump_nav_logs` 支持解析三类数据。

---

### ✔ 6) 导航状态发布器（nav_state_publisher）

所有导航结果结构化为：

```
shared/msg/nav_state.hpp
```

发布模块：

```
nav_state_publisher.h
nav_state_publisher.cpp
```

当前实现为 NullPublisher（不输出），后续可无缝替换：

* UDP 广播
* ZeroMQ PUB
* POSIX 共享内存
* pipes / IPC

用于控制器（PID / MPC / RL）实时读取导航状态。

---

# 2. 目录结构

```
nav_core/
├── CMakeLists.txt
├── include/
│   └── nav_core/
│       ├── status.hpp          ← 统一 Status 类型 / 错误码
│       ├── types.hpp           ← 基本类型、别名（时间戳、向量等）
│       ├── logging.hpp         ← 日志接口（宏/函数）
│       ├── timebase.hpp        ← 时间源工具
│       ├── imu_driver_wit.hpp  ← IMU 驱动接口
│       ├── dvl_driver.hpp      ← DVL 驱动接口
│       ├── eskf.hpp            ← ESKF 接口
│       ├── bin_logger.hpp      ← 二进制日志接口
│       ├── imu_rt_filter.hpp   ← 实时 IMU 滤波接口
│       └── nav_daemon.hpp      ← 导航主进程封装（可选）
└── src/
    ├── timebase.cpp
    ├── imu_driver_wit.cpp
    ├── dvl_driver.cpp
    ├── eskf.cpp
    ├── bin_logger.cpp
    ├── imu_rt_filter.cpp
    ├── nav_daemon.cpp          ← 真正的 main 在 apps/ 里时，这里可以是封装类实现
    └── logging.cpp             ← 如需复杂日志实现
```

特点：

* **所有 API 都在 `include/nav_core` 下**
* shared 消息类型位于项目根目录 `shared/msg`
* `dump_nav_logs.cpp` 是日志解析工具的新版入口（待开发）

---

# 3. 编译方式（Linux/Orange Pi）

### 安装依赖

```bash
sudo apt-get install build-essential cmake
```

### 编译

```bash
cd nav_core
mkdir build && cd build
cmake ..
make -j
```

生成：

```
uwnav_navd       → 导航守护进程
dump_nav_logs    → 日志解析工具（待开发）
libnav_core.a    → 可供集成的静态库
```

---

# 4. 驱动说明

## 4.1 IMU 驱动（HWT9073-485）

* 串口 RS485（230400bps）
* 100Hz
* 使用 WitMotion SDK（`wit_c_sdk.c`）
* 包含必要过滤与校验
* 输出：加速度、角速度、姿态、温度、valid

---

## 4.2 DVL 驱动（Hover H1000）

* 解析 PD6 行协议
* 自动识别 BE/WI/SI 等 velocity frame
* 支持有效性过滤（A/V）
* 输出：vel_x, vel_y, vel_z（北东地或机体坐标）

---

## 4.3 Volt Driver（未来扩展）

用于电池监控、功率估计、能耗模型训练。

---

# 5. 导航守护进程 nav_daemon

执行：

```bash
./uwnav_navd
```

功能：

1. 加载设备配置
2. 打开 IMU / DVL 驱动
3. 运行 ESKF（预测+更新）
4. 写入二进制日志
5. 调用 nav_state_publisher 发布当前导航状态

核心特点：

* 多线程 + 回调架构
* 低延迟（IMU 处理路径通常 sub-ms）
* 无锁状态读取（通过 ESKF 快照）

---

# 6. 日志系统 dump_nav_logs(待开发)

运行示例：

```bash
./dump_nav_logs imu.bin --imu
./dump_nav_logs dvl.bin --dvl
./dump_nav_logs eskf.bin --eskf
```

输出 CSV 兼容 Python、MATLAB、Pandas 处理。

所有结构来自：

```
include/nav_core/log_packets.h
```

保证长期兼容性。

---

# 7. Python 与 nav_core 的关系

```
Python / uwnav     → 数据采集、预处理、可视化、算法验证
C++ / nav_core     → 实时运行、融合、状态发布（实际部署）
```

Python 侧可以读取 nav_core 生成的 `.bin` 数据进行：

* ESKF 参数调优
* 神经网络训练
* 时序对齐分析
* 轨迹可视化

---

# 8. Roadmap（演进计划）

| 项目                           | 状态            |
| ---------------------------- | ------------- |
| 深度计驱动融合                      | planned       |
| USBL 驱动与定位融合                 | planned       |
| 更完整 ESKF（DVL 质量建模、外点剔除）      | in progress   |
| ZeroMQ / UDP / SHM 输出        | planned       |
| 高性能 logger（环形 buffer + 异步写盘） | planned       |
| 机体系→惯性系坐标变换与杆臂补偿             | upcoming      |
| 与控制器（MPC/RL）实时闭环             | ultimate goal |

---

# 总结

`nav_core` 是整个水下机器人系统中的关键组件，其定位是：

* 高实时性
* 高可靠性
* 面向实际部署
* 完整的传感器输入 → 状态输出链路

本 README 旨在帮助新开发者快速理解：
模块组成、代码位置、编译方法、运行方式、与 Python 与控制器的耦合方式。
