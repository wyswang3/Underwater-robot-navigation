````markdown
# nav_core 运行说明（Orange Pi / 香橙派）

本说明文档面向工程使用场景，介绍 `nav_core` C++ 子项目在香橙派上的编译、运行方式，以及导航守护进程 `uwnav_navd` 的日志目录结构与二进制格式，便于后续数据分析与工具开发。

---

## 1. 模块与功能概述

`nav_core` 子项目由一个核心静态库和若干可执行程序组成：

- **核心库：`nav_core`**
  - IMU 驱动：HWT9073-485（Modbus RTU，230400 bps）
  - DVL 驱动：Hover H1000（PD6/EPD6 串口协议）
  - 实时 IMU 滤波与航向积分：
    - 对加速度 / 角速度做低通滤波和启动期零偏估计
    - 对 Z 轴角速度积分并解缠，生成平滑的 yaw 观测
  - 扩展卡尔曼滤波（ESKF）：
    - 输入：IMU（高频预测）+ DVL（速度更新）+ yaw 观测（低频）
    - 输出：位置 / 速度 / 姿态 / IMU 零偏估计
  - 二进制日志写入：
    - `imu.bin`：IMU 观测序列
    - `dvl.bin`：DVL 观测序列
    - `eskf.bin`：ESKF 状态序列

- **导航守护进程：`uwnav_navd`**
  - 运行在香橙派上，负责：
    - 打开 IMU / DVL 串口
    - 驱动 `nav_core` 中的 ESKF 进行状态估计
    - 将 IMU / DVL / ESKF 状态落盘
  - 注意：**不直接控制推进器**，仅负责导航相关数据采集与状态估计。

- **工具程序（可选）：`dump_imu_bin`**
  - 用于离线解析 `imu.bin` 等二进制文件，并输出为人类可读格式（文本 / CSV）。
  - 由 CMake 选项 `BUILD_NAV_TOOLS` 控制是否编译。

---

## 2. C++ 工程目录结构

`nav_core` 子项目整体结构如下：

Underwater-robot-navigation/
└── nav_core/
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


说明：
- `nav_core` 是核心静态库，包含 IMU/DVL 驱动、滤波器、ESKF、日志工具等全部模块。
- `nav_daemon.cpp` 编译为独立可执行程序 `uwnav_navd`。
- `dump_imu_bin.cpp` 当前作为库的一部分编译（编译工具时会用到。
- `third_party/witmotion/` 路径在 CMake 中写死，不要随意移动。

---

## 3. 环境与编译

### 3.1 环境要求

**硬件**

* 香橙派 / Orange Pi 或其他 ARM Linux 开发板

**软件**

* Linux 系统环境（推荐 Debian/Ubuntu 系）
* CMake ≥ 3.14
* g++ 支持 C++17
* Git（用于拉取代码）

**传感器连接（推荐默认接线）**

* IMU HWT9073-485 → `/dev/ttyUSB0`，波特率 230400，Modbus 从站地址 0x50
* DVL Hover H1000 → `/dev/ttyACM0`，波特率 115200

> 可以通过 `ls /dev/ttyUSB*` 和 `ls /dev/ttyS*` 检查具体设备号。

### 3.2 获取代码

在香橙派上执行：

```bash
cd ~
git clone -b feature/IMU-DVL-serial https://github.com/wyswang3/Underwater-robot-navigation.git
cd Underwater-robot-navigation/nav_core
```

如需切换其它分支，可根据实际情况调整 `-b` 参数。

### 3.3 编译 nav_core

```bash
cd ~/Underwater-robot-navigation/nav_core

mkdir -p build
cd build

cmake ..        # 如需关闭工具构建，可用：cmake .. -DBUILD_NAV_TOOLS=OFF
make -j4
```

编译完成后，`build/` 目录下包含：

```text
build/
├── uwnav_navd      ← 导航守护进程（主程序）
└── dump_imu_bin    ← 二进制日志解析工具（若 BUILD_NAV_TOOLS=ON）
```

---

## 4. 运行导航守护进程

### 4.1 串口检查

确认 IMU 和 DVL 对应串口存在，例如：

```bash
ls /dev/ttyUSB*
# 预期：/dev/ttyUSB0（IMU）, /dev/ttyUSB1（DVL）
```

如串口号不同，后续通过命令行参数修改即可。

### 4.2 基本运行方式（默认配置）

```bash
cd ~/Underwater-robot-navigation/nav_core/build
./uwnav_navd
```

默认参数：

* IMU：

  * 端口：`/dev/ttyUSB0`
  * 波特率：`230400`
  * Modbus 从站地址：`0x50`
* DVL：

  * 端口：`/dev/ttyUSB1`
  * 波特率：`115200`
* 日志根目录：`./logs`（相对 `build/`）

启动后典型输出如下：

```text
[navd] Started navigation daemon
       IMU: /dev/ttyUSB0 @230400 addr=0x50
       DVL: /dev/ttyUSB1 @115200
       log root: ./logs
       log dir : ./logs/2025-11-25
```

按 `Ctrl+C` 可以优雅关闭进程，驱动与日志会自动停止与刷新。

### 4.3 命令行参数

`uwnav_navd` 支持以下命令行参数：

```text
Usage: uwnav_navd [options]
  --imu-port <path>      default: /dev/ttyUSB0
  --imu-baud <baud>      default: 230400
  --imu-addr <hex>       default: 0x50
  --dvl-port <path>      default: /dev/ttyUSB1
  --dvl-baud <baud>      default: 115200
  --log-dir <dir>        default: ./logs
  -h, --help             show this help
```

参数解释：

* `--imu-port`

  * IMU 串口路径，例如 `/dev/ttyUSB0`、`/dev/ttyS1` 等。
* `--imu-baud`

  * IMU 串口波特率，HWT9073-485 当前配置为 230400。
* `--imu-addr`

  * Modbus 从站地址，支持十进制或 `0x..` 十六进制形式。
* `--dvl-port` / `--dvl-baud`

  * DVL 串口路径与波特率，需要与 DVL 内部配置一致。
* `--log-dir`

  * 日志根目录，程序会在其下自动创建日期子目录 `YYYY-MM-DD/`。

#### 示例：修改串口号

```bash
./uwnav_navd \
    --imu-port /dev/ttyS3 \
    --dvl-port /dev/ttyS4
```

#### 示例：指定日志目录

```bash
./uwnav_navd \
    --log-dir /data/uwnav_logs
```

实际日志会写入：

```text
/data/uwnav_logs/YYYY-MM-DD/imu.bin
/data/uwnav_logs/YYYY-MM-DD/dvl.bin
/data/uwnav_logs/YYYY-MM-DD/eskf.bin
```

#### 示例：使用 tmux 做长时间实验

```bash
tmux new -s navd
cd ~/Underwater-robot-navigation/nav_core/build
./uwnav_navd --log-dir /data/uwnav_logs
# 实验结束后 Ctrl+C 停止，再退出 tmux
```

---

## 5. 日志目录结构与文件说明

进程运行时，会按当前日期创建日志目录。以默认 `./logs` 为例：

```text
nav_core/build/
└── logs/
    └── YYYY-MM-DD/
         ├── imu.bin    ← IMU 原始数据序列
         ├── dvl.bin    ← DVL 速度观测序列
         └── eskf.bin   ← ESKF 状态估计序列
```

如果通过 `--log-dir` 指定为 `/data/uwnav_logs`，则为：

```text
/ data/uwnav_logs/
└── YYYY-MM-DD/
     ├── imu.bin
     ├── dvl.bin
     └── eskf.bin
```

三个 `.bin` 文件均由定长结构体顺序写入，紧凑布局（1 字节对齐），便于用 C++/Python 按结构解析。

---

## 6. 二进制日志结构定义

以下为 `uwnav_navd` 代码中使用的日志结构体定义（`#pragma pack(push, 1)` 对齐）。

### 6.1 IMU 日志：`imu.bin`

```cpp
#pragma pack(push, 1)
struct ImuLogPacket {
    int64_t mono_ns;        // 单调时钟时间戳（ns）
    int64_t est_ns;         // 估计的墙钟时间（ns）
    float   lin_acc[3];     // 线加速度 [ax, ay, az] (m/s^2)
    float   ang_vel[3];     // 角速度   [gx, gy, gz] (rad/s)
    float   euler[3];       // 欧拉角   [roll, pitch, yaw] (rad)
    float   temperature;    // 温度 (°C)
    uint8_t valid;          // 1=有效, 0=无效
    uint8_t reserved[3];    // 保留对齐
};
#pragma pack(pop)
```

要点：

* `mono_ns`：

  * 单调递增的时间戳，用于积分、插值、对齐 ESKF 内部时间。
* `est_ns`：

  * 对外对齐使用的“估计墙钟”时间戳，用于和其他进程 / 设备数据对齐。
* `lin_acc` / `ang_vel` / `euler`：

  * 已经按物理单位换算。
* `valid`：

  * 来自 IMU 质量过滤结果（`ImuFilterConfig` 控制阈值）。

### 6.2 DVL 日志：`dvl.bin`

```cpp
#pragma pack(push, 1)
struct DvlLogPacket {
    int64_t mono_ns;    // 单调时钟时间戳（ns）
    int64_t est_ns;     // 估计墙钟时间（ns）
    float   vel[3];     // 速度 [vx, vy, vz] (m/s) - 坐标系由 DVL 驱动约定
    int32_t valid;      // 1=有效(A), 0=无效(V)
    int32_t quality;    // 预留质量指标（当前版本暂不使用）
};
#pragma pack(pop)
```

说明：

* `vel[3]`：

  * 源自 PD6/EPD6 解析结果，单位 `m/s`。
  * 坐标系（ENU / NED 等）在 DVL 驱动文档中约定。
* `valid`：

  * 映射自 DVL 报文中的 `A` / `V` 标志。

### 6.3 ESKF 状态日志：`eskf.bin`

```cpp
#pragma pack(push, 1)
struct EskfLogPacket {
    int64_t mono_ns;       // 单调时钟时间戳（ns）
    int64_t est_ns;        // 估计墙钟时间（ns）
    float   pos[3];        // 位置 [x, y, z] (m)
    float   vel[3];        // 速度 [vx, vy, vz] (m/s)
    float   euler[3];      // 姿态 [roll, pitch, yaw] (rad)
    float   bias_accel[3]; // 加速度零偏估计 (m/s^2)
    float   bias_gyro[3];  // 陀螺零偏估计 (rad/s)
    uint8_t valid;         // 1=状态有效
    uint8_t status;        // 状态码（bitmask 用于诊断）
    uint8_t reserved[2];   // 保留对齐
};
#pragma pack(pop)
```

说明：

* `pos` / `vel` / `euler`：

  * ESKF 时刻对应的状态估计，单位分别为 m / m/s / rad。
* `bias_accel` / `bias_gyro`：

  * 对 IMU 偏置的在线估计，有助于分析传感器长时间漂移。
* `status`：

  * 预留状态位（例如：初始化中 / 正常 / 观测缺失 / DVL 失锁等）。

---

## 7. 离线解析与工具链

### 7.1 C++ 工具：`dump_imu_bin`（可选）

若 CMake 构建时开启了 `BUILD_NAV_TOOLS=ON`（默认），会生成 `dump_imu_bin` 工具：

```bash
cd ~/Underwater-robot-navigation/nav_core/build
./dump_imu_bin \
    --imu logs/2025-11-25/imu.bin \
    --dvl logs/2025-11-25/dvl.bin \
    --eskf logs/2025-11-25/eskf.bin \
    --out imu_dump.csv
```

具体命令行参数以源码中 `dump_imu_bin.cpp` 为准，可根据需要扩展为输出 CSV / TXT 等格式。

### 7.2 Python 解析示例

使用 Python 的 `struct` 模块也可以快速解析 `.bin`，例如解析 IMU 日志：

```python
import struct

IMU_FMT = "<qqffffffffB3x"   # 小端，对应 ImuLogPacket
IMU_SIZE = struct.calcsize(IMU_FMT)

with open("logs/2025-11-25/imu.bin", "rb") as f:
    idx = 0
    while True:
        chunk = f.read(IMU_SIZE)
        if not chunk:
            break
        (mono_ns, est_ns,
         ax, ay, az,
         gx, gy, gz,
         roll, pitch, yaw,
         temp, valid) = struct.unpack(IMU_FMT, chunk)

        # 这里可以写入 CSV 或直接画图
        # ...
        idx += 1
```

DVL 和 ESKF 的解析，只需根据对应结构体调整 format 字符串即可。

---

## 8. 常见问题与排查建议

1. **程序启动后未生成任何日志文件**

   * 确认当前工作目录为 `nav_core/build`（相对路径的 `./logs` 会受影响）。
   * 检查指定的 `--log-dir` 是否存在写权限（例如使用 `ls -ld <dir>`）。
   * 使用 `ps`, `htop` 确认 `uwnav_navd` 是否仍在运行。

2. **日志文件存在，但文件大小为 0 或极小**

   * 检查串口是否配置正确：

     * `--imu-port` / `--imu-baud`
     * `--dvl-port` / `--dvl-baud`
   * 确认没有其他进程占用同一串口（例如 Python 测试脚本）。
   * 可在 IMU/DVL 回调中临时加计数输出（调试阶段）。

3. **ESKF 输出异常（位置发散、姿态跳变）**

   * 检查 IMU 安装方向与坐标系定义是否与代码约定一致。
   * 确认 ESKF 配置（噪声标准差、重力方向）合理。
   * 对照 `imu.bin` / `dvl.bin` 单独画图，先确认原始观测是否正常。

---

```