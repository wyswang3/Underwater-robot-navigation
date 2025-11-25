---

# nav_core 运行说明（Orange Pi / 香橙派）

本文件说明如何在香橙派上编译、运行 C++ 导航守护进程 `uwnav_navd`，并介绍该进程产生日志的目录结构及二进制格式，方便后续离线分析与工具开发。

---

## 1. 功能概述

`nav_core` C++ 子项目主要实现：

* IMU（HWT9073-485, Modbus RTU, 230400bps）实时读取与滤波；
* DVL（Hover H1000, PD6/EPD6）实时读取与滤波；
* 基于 IMU + DVL 的扩展卡尔曼滤波（ESKF）状态估计（位置/速度/姿态/零偏）；
* 将 IMU、DVL、ESKF 状态以二进制形式落盘到 `logs/YYYY-MM-DD/` 目录。

该进程 **不直接控制推进器**，只负责导航相关的数据采集与状态估计。

---

## 2. 环境与编译

### 2.1 环境要求

* 硬件：香橙派 / Orange Pi（Linux 系统）
* 软件：

  * CMake ≥ 3.14
  * g++ 支持 C++17
  * Git（用于 clone 仓库）
* 已连接传感器：

  * IMU（HWT9073-485）接到某个串口（例如 `/dev/ttyUSB0`）
  * DVL（Hover H1000）接到某个串口（例如 `/dev/ttyUSB1`）

> 提示：可通过 `ls /dev/ttyUSB*` 或 `ls /dev/ttyS*` 查设备号。

### 2.2 获取代码

在香橙派上执行：

```bash
cd ~
git clone -b feature/IMU-DVL-serial https://github.com/wyswang3/Underwater-robot-navigation.git
cd Underwater-robot-navigation/nav_core
```

### 2.3 编译 nav_core

```bash
cd ~/Underwater-robot-navigation/nav_core

mkdir -p build
cd build

cmake ..
make -j4
```

生成的导航守护进程可执行文件为：

```bash
./uwnav_navd
```

---

## 3. 运行导航进程

### 3.1 串口准备

确认 IMU 和 DVL 已正确连接到香橙派串口，例如：

* IMU：`/dev/ttyUSB0`（默认）
* DVL：`/dev/ttyUSB1`（默认）

如串口号与默认为不同，可在启动时通过命令行参数指定。

### 3.2 基本运行方式

进入 build 目录：

```bash
cd ~/Underwater-robot-navigation/nav_core/build
```

最简启动命令（使用默认配置）：

```bash
./uwnav_navd
```

默认配置为：

* IMU：

  * 端口：`/dev/ttyUSB0`
  * 波特率：`230400`
  * Modbus 从站地址：`0x50`
* DVL：

  * 端口：`/dev/ttyUSB1`
  * 波特率：`115200`
* 日志根目录：`./logs`（相对于 `nav_core/build`）

进程启动后，会在终端打印类似信息：

```text
[navd] Started navigation daemon
       IMU: /dev/ttyUSB0 @230400 addr=0x50
       DVL: /dev/ttyUSB1 @115200
       log root: ./logs
       log dir : ./logs/2025-11-25
```

按 `Ctrl+C` 可优雅退出，进程会自动停止驱动并 flush 日志。

### 3.3 命令行参数说明

`uwnav_navd` 支持以下参数：

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

说明：

* `--imu-port`
  IMU 串口路径（如 `/dev/ttyUSB0`、`/dev/ttyS1` 等）。
* `--imu-baud`
  IMU 串口波特率（HWT9073-485 当前为 230400）。
* `--imu-addr`
  Modbus 从站地址，支持十进制或 `0x..` 十六进制。
* `--dvl-port` / `--dvl-baud`
  DVL 串口路径与波特率（需与 DVL 内部配置一致）。
* `--log-dir`
  日志根路径，程序会在其下自动创建日期子目录，例如传 `--log-dir /data/nav_logs`，实际日志目录为 `/data/nav_logs/2025-11-25/`。

#### 示例 1：串口号不同

```bash
./uwnav_navd \
    --imu-port /dev/ttyS3 \
    --dvl-port /dev/ttyS4
```

#### 示例 2：自定义日志目录

```bash
./uwnav_navd \
    --log-dir /data/uwnav_logs
```

日志将写入：

```text
/data/uwnav_logs/YYYY-MM-DD/imu.bin
/data/uwnav_logs/YYYY-MM-DD/dvl.bin
/data/uwnav_logs/YYYY-MM-DD/eskf.bin
```

#### 示例 3：结合 tmux 长时间运行

```bash
tmux new -s navd
./uwnav_navd --log-dir /data/uwnav_logs
# 实验完成后按 Ctrl+C 停止，再退出 tmux
```

---

## 4. 日志目录结构

导航进程运行时，会根据当前日期自动创建：

```text
<nav_core>/build/
└── logs/
    └── YYYY-MM-DD/
         ├── imu.bin    ← IMU 日志（二进制）
         ├── dvl.bin    ← DVL 日志（二进制）
         └── eskf.bin   ← ESKF 状态（二进制）
```

如果通过 `--log-dir` 指定了其它路径，例如 `/data/uwnav_logs`，结构类似：

```text
/data/uwnav_logs/
└── YYYY-MM-DD/
     ├── imu.bin
     ├── dvl.bin
     └── eskf.bin
```

每个 `.bin` 文件是 **定长结构体序列**，可以用 Python、C++ 等按照结构定义逐条读取，进行离线分析。

---

## 5. 二进制日志格式说明

所有日志包均采用 `#pragma pack(push, 1)` 对齐，即 **紧凑布局，无填充**。

### 5.1 IMU 日志 imu.bin

对应 C++ 结构体：

```cpp
#pragma pack(push, 1)
struct ImuLogPacket {
    int64_t mono_ns;        // 单调时钟时间戳（纳秒）
    int64_t est_ns;         // 估计的“墙钟”时间（纳秒）
    float   lin_acc[3];     // 线加速度 [ax, ay, az] (m/s^2)
    float   ang_vel[3];     // 角速度   [gx, gy, gz] (rad/s)
    float   euler[3];       // 欧拉角   [roll, pitch, yaw] (rad)
    float   temperature;    // 温度 (°C)
    uint8_t valid;          // 1=有效, 0=无效
    uint8_t reserved[3];    // 保留对齐
};
#pragma pack(pop)
```

字段说明：

* `mono_ns`：
  使用 `timebase::stamp()` 生成的单调时钟时间（不回跳），适合做积分与差值。
* `est_ns`：
  同一时刻在“估计墙钟”上的时间戳，用于与其他进程/设备对齐。
* `lin_acc`：
  由 IMU 原始寄存器加速度换算而来，单位为 `m/s^2`。
* `ang_vel`：
  由 IMU 原始角速度换算而来，单位为 `rad/s`。
* `euler`：
  由高精度 Roll/Pitch/Yaw 寄存器解算而来，单位为 `rad`。
* `temperature`：
  IMU 内部温度，单位 `°C`。
* `valid`：
  传感器数据是否通过滤波与范围检查。

一个典型的 CPU 上，`sizeof(ImuLogPacket)` 为 72 字节（以实际编译为准）。

### 5.2 DVL 日志 dvl.bin

对应 C++ 结构体：

```cpp
#pragma pack(push, 1)
struct DvlLogPacket {
    int64_t mono_ns;    // 单调时钟时间戳（纳秒）
    int64_t est_ns;     // 估计墙钟时间（纳秒）
    float   vel[3];     // DVL 三轴速度 [ve, vn, vu] (m/s)
    int32_t valid;      // 1=有效(A), 0=无效(V), 其它值保留
    int32_t quality;    // 暂未使用，预留质量指标
};
#pragma pack(pop)
```

字段说明：

* `vel[3]`：
  由 DVL PD6/EPD6 速度帧解析得到，单位为 `m/s`，坐标系为设备定义的 ENU 或相关约定（后续可在文档中进一步细化）。
* `valid`：
  对应 PD6 帧中的 `A` / `V` 标志，1 表示底跟踪成功，0 表示失锁。
* `quality`：
  当前版本暂未使用，预留给后续 DVL 质量字段（如相关系数、波束数量等）。

### 5.3 ESKF 状态日志 eskf.bin

对应 C++ 结构体：

```cpp
#pragma pack(push, 1)
struct EskfLogPacket {
    int64_t mono_ns;       // 单调时钟时间戳（纳秒）
    int64_t est_ns;        // 估计墙钟时间（纳秒）
    float   pos[3];        // 位置 [x, y, z] (m)，坐标系由 ESKF 配置决定
    float   vel[3];        // 速度 [vx, vy, vz] (m/s)
    float   euler[3];      // 姿态 [roll, pitch, yaw] (rad)
    float   bias_accel[3]; // 加速度零偏估计 (m/s^2)
    float   bias_gyro[3];  // 陀螺零偏估计 (rad/s)
    uint8_t valid;         // 1=当前状态有效
    uint8_t status;        // 状态/故障码，低 8 位
    uint8_t reserved[2];   // 保留对齐
};
#pragma pack(pop)
```

字段说明：

* `pos`：
  当前估计的位置，单位 `m`，可为 ENU、NED 或机体系积分坐标，取决于 `EskfConfig` 的具体实现。
* `vel`：
  当前估计的速度，单位 `m/s`。
* `euler`：
  当前估计的姿态（滚转、俯仰、偏航），单位 `rad`。
* `bias_accel` / `bias_gyro`：
  对 IMU 加速度/陀螺零偏的估计，用于后续分析传感器稳定性。
* `status`：
  预留槽位，可用于标记滤波器状态（稳定、初始化、失效等）。

---

## 6. 离线解析建议

### 6.1 用 Python 解析 `.bin`

可以在 `apps/tools/` 下编写 Python 工具，例如 `navcore_log_reader.py`，使用 `struct` 模块按上述结构解包：

```python
import struct

IMU_FMT = "<qqffffffffB3x"  # little-endian, 按 ImuLogPacket 顺序
IMU_SIZE = struct.calcsize(IMU_FMT)

with open("logs/2025-11-25/imu.bin", "rb") as f:
    data = f.read()

n = len(data) // IMU_SIZE
for i in range(n):
    chunk = data[i*IMU_SIZE:(i+1)*IMU_SIZE]
    (mono_ns, est_ns,
     ax, ay, az,
     gx, gy, gz,
     roll, pitch, yaw,
     temp, valid) = struct.unpack(IMU_FMT, chunk)
    # 在此处进行绘图或保存到 CSV
```

DVL 和 ESKF 同理，只需按各自的结构体定义格式串。

### 6.2 与 Python 采集的 CSV 对齐

本 C++ 导航进程产出的 `.bin` 文件主要用于：

* 验证 ESKF 状态估计算法；
* 与 Python 采集的原始 IMU / DVL CSV 做对比；
* 为后续 MPC / RL 等算法提供更高质量的状态标签。

推荐做法：

1. 使用 `est_ns` 与 Python 侧的时间戳（例如 UNIX 秒或纳秒）对齐；
2. 将 `.bin` 转为 CSV 后，与 `data/YYYY-MM-DD/aligned/` 下的对齐数据联合分析。

---

## 7. 常见问题与排查

1. **程序启动后无任何日志文件产生？**

   * 检查 `--log-dir` 指定目录是否有写权限；
   * 确认 `uwnav_navd` 所在工作目录为 `nav_core/build`；
   * 确认程序是否真的在运行（如使用 `ps`, `htop`）。

2. **日志文件存在，但文件大小为 0？**

   * 确认 IMU / DVL 串口配置正确（端口、波特率、地址）；
   * 检查串口是否被其它进程占用（如 Python 采集脚本）；
   * 增加简单调试输出（可临时在回调中打印计数）。

3. **IMU / DVL 状态错误 / 噪声异常大？**

   * 确认滤波配置（`ImuFilterConfig` / `DvlFilterConfig`）是否过于宽松或过紧；
   * 检查传感器安装方向、接线与供电；
   * 使用 Python 侧的 `imu_data_verifier.py` / `dvl_data_verifier.py` 对比输出。

---