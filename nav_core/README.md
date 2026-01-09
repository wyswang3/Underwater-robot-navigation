
---

````md
# nav_core —— 水下机器人在线导航 C++ 子系统

`nav_core` 是 UnderwaterRobotSystem 中的「导航子项目」，运行在香橙派（OrangePi）等板卡上，负责：

- 通过串口采集 **IMU（WitMotion HWT9073-485, Modbus-RTU）** 与 **DVL（Hover H1000, PD6/EPD6 协议）** 数据；
- 对 IMU 数据做基础实时滤波（静止窗 + bias 估计 + deadzone）；
- 利用 IMU 滤波后的航向角 + DVL 体坐标速度，进行轻量级 **在线导航解算**（ENU 位置 / 速度 / 姿态）；
- 将当前导航状态打包成 `shared::msg::NavState`：
  - 一方面落盘为二进制日志，方便离线分析；
  - 另一方面通过 **共享内存（seqlock）发布给控制进程**；
- 在终端打印必要的日志与状态概要，便于现场调试与监控。

当前版本的目标是：  
**验证在线导航系统的实时精度，不参与控制闭环**，最多在终端和下游进程中实时反馈当前时刻的导航信息。

---

## 1. 目录结构概览

与本项目相关的目录结构如下：

```text
nav_core/
├── CMakeLists.txt
├── config
│   └── nav_daemon.yaml          # 导航守护进程配置
├── docs
│   ├── device_test_and_debug.md # 设备调试说明
│   ├── filters
│   │   └── eskf_design.*        # ESKF 设计草稿
│   ├── nav_core_runtime.md      # 运行时说明（可逐步完善）
│   └── update_guidelines.md     # 更新指引
├── include
│   └── nav_core
│       ├── app
│       │   └── nav_daemon_config.hpp
│       ├── core
│       │   ├── logging.hpp
│       │   ├── math.hpp
│       │   ├── status.hpp
│       │   ├── timebase.hpp
│       │   └── types.hpp
│       ├── drivers
│       │   ├── dvl_driver.hpp
│       │   ├── dvl_protocol.hpp
│       │   └── imu_driver_wit.hpp
│       ├── estimator
│       │   ├── graph_smoother_2d.hpp
│       │   ├── nav_health_monitor.hpp
│       │   └── online_estimator.hpp
│       ├── filters
│       │   ├── eskf.hpp
│       │   ├── filter_commom.hpp
│       │   └── imu_rt_filter.hpp
│       └── io
│           ├── bin_logger.hpp
│           ├── log_packets.hpp
│           └── nav_state_publisher.hpp
├── src
│   └── nav_core
│       ├── app
│       │   ├── nav_daemon.cpp        # 导航守护进程主程序（uwnav_navd）
│       │   └── tools
│       │       └── DVL_selftest.cpp  # DVL 自检小工具
│       ├── core
│       │   ├── logging.cpp
│       │   └── timebase.cpp
│       ├── drivers
│       │   ├── dvl_driver.cpp
│       │   ├── dvl_protocol.cpp
│       │   └── imu_driver_wit.cpp
│       ├── estimator
│       │   ├── graph_smoother_2d.cpp
│       │   ├── nav_health_monitor.cpp
│       │   └── online_estimator.cpp
│       ├── filters
│       │   ├── eskf.cpp
│       │   └── imu_rt_filter.cpp
│       └── io
│           ├── bin_logger.cpp
│           └── nav_state_publisher.cpp
└── third_party
    └── witmotion
        ├── REG.h
        ├── wit_c_sdk.c
        └── wit_c_sdk.h
````

编译后，会生成：

* 静态库：`libnav_core.a`（供其他工程复用）
* 导航守护进程可执行文件：`uwnav_navd`
* DVL 自检工具：`DVL_selftest`（可选，根据 CMake 选项）

---

## 2. 当前 C++ 导航子系统已具备的功能

### 2.1 设备驱动层

1. **IMU 驱动 `ImuDriverWit`**

   * 使用厂家 WitMotion C SDK（`wit_c_sdk.c`）封装：

     * 串口打开 / 关闭（香橙派一般为 `/dev/ttyUSB0`，波特率 230400）。
     * Modbus-RTU 读寄存器循环。
     * 通过 SDK 的 C 回调解码寄存器数组 `sReg[]`。
   * 对上层输出统一的 `ImuFrame`：

     * 单调时间戳（MonoTimeNs）。
     * 机体系角速度、线加速度。
     * 欧拉角（roll / pitch / yaw，rad）。
     * 温度等可选字段。
   * 支持基础幅值过滤：根据 `ImuFilterConfig` 丢弃明显异常帧。

2. **DVL 驱动 `DvlDriver`**

   * 串口阻塞读 PD6/EPD6 文本（Hover H1000 常用波特率 115200）。
   * 使用 `dvl_protocol` 解析为结构化数据：

     * 帧类型（底跟踪 / 水团）。
     * 坐标系（体坐标 / 地理 ENU）。
     * 速度、位移、锁底标志、A/V 有效标志等。
   * 输出两类接口：

     * `DvlRawData`：带解析字段的原始协议帧（用于日志 / 调参 / 因子图等）。
     * `DvlFrame`：通过过滤后、适合在线导航使用的 **导航速度帧**。

       * 体坐标速度 `vel_body_mps[3]`（BI/BS 帧）。
       * ENU 速度 `vel_enu_mps[3]`（由 BE 帧或后续估计器旋转得到）。
       * 高度 / FOM / 有效标志等。
   * 提供命令发送接口：

     * `sendCommandCZ()`：停机，避免空气中 ping 损伤换能器。
     * `sendCommandCS(ping_rate, avg_count)`：下水后启动 ping。

### 2.2 过滤与估计层

1. **IMU 实时滤波 `ImuRtFilter`**

   * 静止窗检测（gyro / accel 阈值）。
   * 在静止窗内估计 IMU bias，并限制 bias 的最大幅度。
   * 对滤波后数据应用 deadzone，抑制高频噪声积分。

2. **在线导航估计器 `OnlineEstimator`**
   当前版本聚焦于「轻量级 2D/2.5D 在线解算」：

   * 以 IMU 的 `mono_ns` 为主时间轴，计算 `dt`；
   * 使用 IMU **滤波后的角速度积分 yaw**，并结合原始 yaw 做 `unwrap`，获得连续航向角；
   * 使用最新一帧 DVL 的 **体坐标系速度** + 当前航向角做体→ENU 旋转，得到 `vel_enu`；
   * 对 ENU 速度做积分，得到 `pos_enu`，并对单步位移做上限限制（防止异常跳变）；
   * 管理 DVL 掉线超时，根据 IMU / DVL 状态给出 `NavHealth` 和 `status_flags`。

结果以 `shared::msg::NavState` 的形式输出，包括：

* ENU 位置 `pos[3]` 与速度 `vel[3]`；
* 欧拉角 `rpy[3]`（roll/pitch 来自 IMU，yaw 为连续航向角）；
* 机体系角速度 / 线加速度；
* 深度字段（目前由上层其他模块写入或保持默认）；
* 健康状态与标志位。

> 注意：
> 当前版本**不包含**完整 ESKF / 深度 / USBL 融合，
> `GraphSmoother2D` 和 `NavHealthMonitor` 仅保留框架，尚未启用实际因子图优化与停机判据。

### 2.3 IO 与对外接口

1. **二进制日志写入 `BinLogger`**

   * 按 `nav_daemon.yaml` 中的 `logging` 配置：

     * 自动在 `base_dir`（例如 `/home/wys/data/nav`）下面按日期/session 创建子目录；
     * 可选择记录：

       * `ImuLogPacket`（IMU 数据）。
       * `DvlLogPacket`（DVL 数据）。
       * `NavState` 快照。
   * 用于离线 Python 分析、因子图/ESKF 评估等。

2. **共享内存发布 `NavStatePublisher`**

   * 使用 `ShmHeader + NavState` 的固定布局，通过 POSIX shm 发布；
   * 采用 **seqlock** 协议：

     * 写端递增 `seq` 为奇数 → 写入 NavState → 递增为偶数；
     * 读端只在两次读取 `seq` 相同且为偶数时，认为快照一致；
   * 控制进程可通过统一的 shm 名称（例如 `/rov_nav_state_v1`）零拷贝读取当前导航状态。

---

## 3. 导航守护进程 `uwnav_navd` 的运行与行为

导航主程序位于：

* 源码：`src/nav_core/app/nav_daemon.cpp`
* 可执行名：`uwnav_navd`

### 3.1 构建与安装（示例）

在 UnderwaterRobotSystem 根目录（包含 `shared/` 和 `Underwater-robot-navigation/nav_core/`）下：

```bash
mkdir -p Underwater-robot-navigation/nav_core/build
cd Underwater-robot-navigation/nav_core/build

cmake ..    # 或者加上 -DCMAKE_BUILD_TYPE=RelWithDebInfo 等
make -j4
```

完成后，`uwnav_navd` 将生成在 `build` 目录下。

### 3.2 启动示例

在 `nav_core/build` 目录下，可使用类似命令运行：

```bash
./uwnav_navd --config ../config/nav_daemon.yaml
```

其中：

* `--config` 指定导航配置文件路径；
* `nav_daemon.yaml` 中配置了 IMU/DVL 串口、滤波参数、在线估计器参数、日志目录、共享内存名称等。

### 3.3 主循环逻辑（v1）

`uwnav_navd` 的主循环（默认 50 Hz）大致流程：

1. 加载配置 `NavDaemonConfig`；
2. 初始化并启动：

   * `ImuDriverWit`（后台采集线程）；
   * `ImuRtFilter`（在回调中对 IMU 数据实时滤波）；
   * `DvlDriver`（后台串口读线程）；
   * `OnlineEstimator`（在线导航估计器）；
   * `BinLogger`（按配置写 IMU/DVL/NavState 日志）；
   * `NavStatePublisher`（共享内存发布）。
3. 在主循环中：

   * 从 IMU / DVL 缓冲区中取出最新样本；
   * 将 IMU 滤波数据、DVL 体速度送入 `OnlineEstimator`；
   * 更新 ENU 位置 / 速度 / 姿态；
   * 拿到一帧最新的 `NavState`；
   * 通过 `NavStatePublisher` 写入共享内存；
   * 根据配置将 `NavState` 写入日志文件；
   * 在终端打印状态概要（见下节）。

---

## 4. 能不能把导航信息打印输出到终端界面？

**可以。**

当前架构下，终端有两种方式看到导航状态：

### 4.1 通过导航守护进程自身的日志输出

`nav_core/core/logging.*` 提供了简单的日志接口（例如 `LOG_INFO` / `LOG_WARN` 等）。
在 `nav_daemon.cpp` 中，主循环会周期性地打印导航状态概要，例如：

```text
[navd] loop tick: imu_age=0.004s dvl_age=0.120s health=OK flags=IMU_OK|DVL_OK
[navd] state: pos_enu=(+1.23, -0.45, -0.02) m  vel_enu=(+0.05, +0.00, +0.00) m/s  yaw=+1.57 rad
```

特点：

* 以文本日志形式写到标准输出（终端），可以直接在 SSH 窗口观察；
* 包含当前 ENU 位置、速度、yaw 航向角以及导航健康标志；
* 打印频率通常低于导航主循环频率（例如每 10 次循环打印一次），以避免刷屏；
* 日志格式是纯文本，便于重定向到文件或用 `grep` / `less` 检查。

因此，在不接任何控制程序的情况下，你可以：

```bash
ssh orangepi@<ip>

cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core/build
./uwnav_navd --config ../config/nav_daemon.yaml
```

直接在终端中实时观察导航状态随时间的变化，用于水池实验的初步验证。

### 4.2 通过下游进程读取共享内存后自定义“终端 UI”

如果之后需要更“好看”的终端界面（例如 curses 风格的表格、趋势条形图等），可以：

1. 在控制端或单独的调试程序中，通过 `NavStatePublisher` 的 shm 布局读取最新 `NavState`；
2. 用 Python/curses 或 C++ 自己刷新一个终端 TUI。

当前版本已经为此准备好了共享内存的标准接口，**不需要动 `nav_core` 的主逻辑**，只需要写一个简单的“NavState 监视器”即可。

---

## 5. 当前版本的边界与下一步规划

当前 C++ 导航子系统（v1）侧重于：

* IMU + DVL 的基础采集与过滤；
* 轻量级在线 2D/2.5D 导航解算（ENU 轨迹）；
* NavState 共享内存发布 + 二进制日志记录；
* 在终端输出关键导航状态（便于在线水池调试）。

暂时 **不包含**：

* 控制闭环（MPC / PID 等在其他子项目中实现）；
* ESKF / 多传感器（深度 / USBL）高级融合；
* 因子图平滑的完整实现与基于平滑轨迹的停机策略；
* 图形化 UI 或 GCS 级展示。

下一步典型演进方向：

1. 在现有在线导航基础上，接入 ESKF 模块，对 IMU / DVL / 深度进行联合估计；
2. 完善 `GraphSmoother2D` 与 `NavHealthMonitor`，实现“在线轨迹 vs 平滑轨迹”自动对比；
3. 增强终端和上位机的可视化能力，例如：

   * 写一个专门的 `nav_monitor` TUI 程序；
   * 在 GCS UI 中通过 UDP/SHM 展示 NavState 轨迹。

---

如需在 README 中强调“终端可以实时看到导航状态”，可以根据实际 `nav_daemon.cpp` 中的日志格式，替换示例输出内容，保持与现场行为一致。

```
::contentReference[oaicite:0]{index=0}
```
