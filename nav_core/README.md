

````md
# nav_core —— 水下机器人在线导航 C++ 子系统

`nav_core` 是 UnderwaterRobotSystem 中的「在线导航子项目」，运行在香橙派（OrangePi）等板卡上，负责：

- 通过串口采集 **IMU（WitMotion HWT9073-485, Modbus-RTU）** 与 **DVL（Hover H1000, PD6/EPD6 协议）** 的原始数据；
- 在板端进行 **IMU / DVL 实时预处理**：
  - IMU：去零偏、扣重力、统一机体系为 body=FRD，输出“线加速度”；
  - DVL：仅保留 BottomTrack + BottomLock 数据，区分 BI/BS 体速 与 BE 地速，做噪声地板与幅值门控；
- 基于 IMU 预处理后的线加速度 + 角速度，配合 DVL BI/BE 速度，运行 **ESKF（Error-State Kalman Filter）在线导航解算**：
  - 输出 ENU 位置 / 速度 / 姿态（roll, pitch, yaw）；
  - 使用 DVL BI 体速 + 当前 yaw 做水平速度约束；
  - 使用 DVL BE ENU 垂向速度做 vU 约束；
- 将当前导航状态打包成 `shared::msg::NavState`：
  - 一方面落盘为二进制日志，便于离线 Python/因子图分析；
  - 另一方面通过 **共享内存（seqlock）发布给控制进程**，作为控制端的导航输入；
- 在终端打印必要的运行日志与导航状态概要，便于现场水池实验和调试。

当前版本的目标是：  
**在板端得到质量可控的 ENU 轨迹与姿态解算结果，为控制闭环和离线平滑提供基础，不直接参与控制回路。**

---

## 1. 目录结构概览

与本子项目相关的目录结构如下（仅列出主要文件）：

```text
nav_core/
├── CMakeLists.txt
├── config
│   ├── eskf2d.yaml              # ESKF 配置（2D/2.5D 版本，可扩展到 full INS）
│   └── nav_daemon.yaml          # 导航守护进程配置
├── docs
│   ├── device_test_and_debug.md # 设备调试说明（IMU/DVL 串口、波特率、自检流程）
│   ├── filters
│   │   ├── eskf_design.tex      # ESKF 设计文档（LaTeX 源）
│   │   └── eskf_design.pdf      # ESKF 设计说明（PDF）
│   ├── imu_dvl_preprocess_and_fusion_guidelines.md
│   │                             # IMU & DVL 预处理 / 融合指南
│   ├── nav_core_runtime.md      # 运行时说明（部署 / 监控 / 故障排查）
│   └── update_guidelines.md     # 更新指引（面向后续维护与新人）
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
│       │   ├── eskf.hpp                 # ESKF 对外接口（统合入口）
│       │   ├── eskf_math.hpp            # ESKF 专用数学工具（skew / quat / 小角度等）
│       │   ├── eskf_config_yaml.hpp     # 从 YAML 读取 EskfConfig
│       │   ├── graph_smoother_2d.hpp    # 因子图平滑（实验性，默认不编译）
│       │   ├── nav_health_monitor.hpp   # 导航健康监控（实验性）
│       │   └── online_estimator.hpp     # 旧版在线估计器（保留接口，默认不使用）
│       ├── filters
│       │   └── filter_common.hpp        # 误差状态维度 / 索引等通用定义
│       ├── io
│       │   ├── bin_logger.hpp
│       │   ├── log_packets.hpp
│       │   ├── nav_state_publisher.hpp
│       │   └── replay                   # 预留的回放接口
│       └── preprocess
│           ├── dvl_rt_preprocessor.hpp  # DVL 实时预处理（BI/BE 门控 + 噪声地板）
│           └── imu_rt_preprocessor.hpp  # IMU 实时预处理（bias + 重力 + 低通等）
├── README.md
├── src
│   └── nav_core
│       ├── app
│       │   ├── nav_daemon_config.cpp    # 解析 nav_daemon.yaml
│       │   ├── nav_daemon_runner.cpp    # 导航主循环实现（run_nav_daemon）
│       │   ├── nav_daemon.cpp           # main，仅做 wiring / 调用 runner
│       │   └── tools
│       │       └── DVL_selftest.cpp     # DVL 自检工具（uwnav_dvl_selftest）
│       ├── core
│       │   ├── logging.cpp
│       │   └── timebase.cpp
│       ├── drivers
│       │   ├── dvl_driver.cpp
│       │   ├── dvl_protocol.cpp
│       │   └── imu_driver_wit.cpp
│       ├── estimator
│       │   ├── eskf_config_yaml.cpp
│       │   ├── eskf_core.cpp            # 构造 / reset / state 导出等核心逻辑
│       │   ├── eskf_propagate.cpp       # IMU 传播（使用线加速度 + 角速度）
│       │   ├── eskf_update_dvl.cpp      # DVL XY / Z 更新（BI + BE）
│       │   ├── eskf_update_yaw.cpp      # yaw 伪观测更新
│       │   ├── eskf.cpp                 # 统合入口 + 管线注释
│       │   ├── graph_smoother_2d.cpp    # 因子图平滑实现（默认不参与 CMake）
│       │   ├── nav_health_monitor.cpp   # 导航健康监控（预留）
│       │   └── online_estimator.cpp     # 旧版估计器实现（暂不在 nav_daemon 中使用）
│       ├── io
│       │   ├── bin_logger.cpp
│       │   ├── nav_state_publisher.cpp
│       │   └── replay                   # 预留
│       └── preprocess
│           ├── dvl_rt_preprocessor.cpp
│           └── imu_rt_preprocessor.cpp
└── third_party
    └── witmotion
        ├── REG.h
        ├── wit_c_sdk.c
        └── wit_c_sdk.h
````

编译后，会生成：

* 静态库：`libnav_core.a`（供其他工程复用）；
* 导航守护进程：`uwnav_navd`（通常安装 / 运行路径为 `build/bin/uwnav_navd`）；
* DVL 自检工具：`uwnav_dvl_selftest`。

---

## 2. 当前 C++ 导航子系统已具备的功能

### 2.1 设备驱动层

#### 2.1.1 IMU 驱动 `ImuDriverWit`

* 使用厂家 WitMotion C SDK（`third_party/witmotion/wit_c_sdk.c`）封装，实现：

  * 串口打开 / 配置（OrangePi 上通常为 `/dev/ttyUSB0`，波特率 230400）；
  * Modbus-RTU 读寄存器循环；
  * 通过 C 回调解码寄存器数组 `sReg[]`。
* 对上层输出统一的 `ImuFrame`：

  * 单调时间戳 `mono_ns`（Monotonic Clock）；
  * 系统时间戳 `est_ns`（System Clock），用于与其他设备对齐；
  * 机体系角速度 `ang_vel[3]`（rad/s，body=FRD）；
  * 机体系线加速度 `lin_acc[3]`（预处理前为“含重力”）；
  * 欧拉角 `euler[3]`（roll / pitch / yaw, rad，来自 IMU 内部解算）；
  * 温度 / 状态字等可选字段；
  * 有效标志 `valid` 和状态 `status`。
* 支持基本幅值检查与异常帧过滤（通过配置项限制明显失真的数据）。

#### 2.1.2 DVL 驱动 `DvlDriver`

* 通过串口阻塞读取 PD6/EPD6 文本帧（Hover H1000，常用波特率 115200）；
* 使用 `dvl_protocol.*` 解析为结构化数据 `ParsedLine`：

  * 底跟踪 / 水团；
  * 坐标系：BI/BS（Instrument/Ship frame），BE（Earth frame）；
  * 速度（ve/vn/vu）、位移（dE/dN/dU）、高度、FOM、A/V 有效标志等；
* 对上层输出 `DvlFrame`：

  * 原始解析字段（为日志与调试保留）；
  * 统一的时间戳 `mono_ns` / `est_ns`；
  * BI/BS 体速度（后续映射为 body=FRD 速度）；
  * BE ENU 速度（特别是垂向 vU，用于 ESKF 垂向更新）；
  * 锁底 `bottom_lock`、跟踪类型 `track_type`、质量指标 `quality`；
  * 有效标志 `valid`。
* 支持通过命令接口控制 DVL：

  * `sendCommandCZ()`：停机，避免空气中 ping 损伤换能器；
  * `sendCommandCS(ping_rate, avg_count)`：下水之后启动 ping；
* 为 `uwnav_dvl_selftest` 工具提供底层支持，用于单独验证 DVL 串口 / CZ/CS 行为。

---

### 2.2 实时预处理层

本层的目标是：**把“硬件原始输出”统一转换为 ESKF 能够直接使用的、物理语义清晰的数据流**。

#### 2.2.1 IMU 实时预处理 `ImuRtPreprocessor`

实现于 `preprocess/imu_rt_preprocessor.*`，对 `ImuFrame` 做以下处理：

* 坐标与单位统一：

  * 角速度单位统一为 rad/s，机体系固定为 body=FRD（X 前 / Y 右 / Z 下）；
  * 加速度统一为 m/s²；
* 零偏估计与去除：

  * 在配置的静止时间窗内（例如开机前若干秒），检测 gyro/acc 是否处于静止；
  * 对静止窗数据估计加速度 bias 与陀螺 bias；
  * 后续实时数据减去 bias，得到零偏校正后的测量；
* 重力扣除：

  * 使用当前 roll/pitch（来自 IMU 内部解算或外部姿态源），估计重力在 body=FRD 下的分量；
  * 从加速度中扣除重力，输出“线加速度 a_linear”；
* 噪声处理：

  * 对线加速度做 deadzone / 一阶低通，抑制高频噪声；
  * 对异常突变做限幅保护，避免积分漂移被单次异常拉偏；
* 输出结构 `ImuSample`：

  * `ang_vel[3]`：零偏校正后的角速度；
  * `lin_acc[3]`：**已经去零偏、扣重力的线加速度**；
  * `euler[3]`：roll/pitch/yaw（可选，用于诊断与 yaw 伪观测）；
  * 同步的 `mono_ns` / `est_ns`、温度、状态等。

> 重要：
> 由于 IMU 预处理已经完成「去零偏 + 扣重力」，ESKF 内部不再重复做这些操作，而是直接配置 `imu_acc_is_linear=true`，把输入 `lin_acc` 视为线加速度。

#### 2.2.2 DVL 实时预处理 `DvlRtPreprocessor`

实现于 `preprocess/dvl_rt_preprocessor.*`，对 `DvlFrame` 做以下处理：

* 只考虑 BottomTrack：

  * 仅保留 `is_bottom_track()==true` 的帧；
  * 可配置 `require_bottom_lock`，强制锁底后才使用；
  * 可配置 `require_valid_flag`，要求 A/V 标志为 `A` 才视为有效；
* 区分 BI/BS 与 BE：

  * **BI / BS（Instrument / Ship frame）**：

    * 仅信任水平速度（ve/vn）；
    * 映射为 body=FRD 下的水平速度 `v_body_xy_mps`（X 前 / Y 右）；
    * 对水平速度应用噪声地板与最大速度限制（`max_speed_xy_mps` / `noise_floor_xy_mps`）；
  * **BE（Earth frame）**：

    * 只关注 ENU 垂向速度 `vU`；
    * 对 vU 应用噪声地板与最大速度限制（`max_speed_z_mps` / `noise_floor_z_mps`）；
* 汇总门控结果：

  * 若 BI/BS 提供了合法的水平速度 → `has_body_xy=true`；
  * 若 BE 提供了合法的垂向速度 → `has_enu_z=true`；
  * 若两者都不存在或被拒绝，则视为 `reason_code |= kReject_NoVelocity`；
  * 若配置 `enable_gating=true`，只在 `reason_code==0` 且存在任一测量时返回 `gated_ok=true`；
* 输出结构 `DvlRtOutput`（在 nav_daemon 内部再映射为 `DvlSample`）：

  * `has_body_xy` + `v_body_xy_mps`：用于 ESKF 的水平更新（BI + yaw-only）；
  * `has_enu_z` + `v_enu_z_mps`：用于 ESKF 的垂向 vU 更新；
  * `bottom_lock`、`quality_raw`、`reason_code`、`gated_ok` 等诊断信息。

---

### 2.3 ESKF 在线导航估计器 `EskfFilter`

实现分布在：

* `estimator/eskf_math.*`：ESKF 专用数学工具（skew、四元数运算、小角度四元数等）；
* `estimator/eskf_core.*`：构造 / reset / state 导出 / 协方差初始化等；
* `estimator/eskf_propagate.*`：IMU 传播（`propagate_imu`）；
* `estimator/eskf_update_dvl.*`：`update_dvl_xy` / `update_dvl_z`；
* `estimator/eskf_update_yaw.*`：`update_yaw`；
* `estimator/eskf.cpp`：对外统一入口 + 管线说明。

#### 2.3.1 状态定义与误差维度

误差状态维度在 `filters/filter_common.hpp` 中统一定义：

* 误差维度：`ESKF_ERR_DIM = 15`；

* 误差状态向量 δx 排列为：

  ```text
  δx = [δp(0-2), δv(3-5), δθ(6-8), δba(9-11), δbg(12-14)]ᵀ
  ```

* 名义状态包含：

  * 位置 `p_enu`（E, N, U）；
  * 速度 `v_enu`（vE, vN, vU）；
  * 姿态四元数 `q_nb`（nav←body）及其 RPY 表示 `rpy_nb_rad`；
  * 加速度零偏 `ba_mps2`；
  * 陀螺零偏 `bg_rad_s`。

坐标系约定：

* 机体系：body=FRD（X 前 / Y 右 / Z 下）；
* 导航系：ENU（East / North / Up），深度 `depth = -U`（向下为正）；
* 重力：g 向下，对应 ENU.z 为负。

#### 2.3.2 IMU 传播 `propagate_imu`

输入：**IMU 预处理后的 `ImuSample`**，满足：

* `ang_vel[3]`：已扣除 gyro bias 的角速度；
* `lin_acc[3]`：已去 bias、扣重力的线加速度（body=FRD）；
* `mono_ns`：单调时间戳（用于计算 dt）。

传播步骤（简述）：

1. 根据 `mono_ns` 与上一次时间计算 `dt`，并做 `dt_min` / `dt_max` 检查；
2. 使用零偏校正后的陀螺测量 `omega_corr` 积分姿态：

   * 小角度时使用一阶近似；
   * 一般情况使用轴角 → 四元数；
3. 使用当前姿态 `R_nb` 将线加速度从 body→ENU，若 `imu_acc_is_linear=false` 则内部补重力（当前工程配置为 true，即不再补重力）；
4. 积分速度与位置，包含：

   * 水平速度限幅；
   * 垂向速度限幅；
   * 可选的速度泄漏（防止积分无限生长）；
5. 构建线性化矩阵 F、G，根据 IMU 噪声 / bias 漂移参数构造离散过程噪声 Qd，传播协方差。

#### 2.3.3 DVL 水平更新 `update_dvl_xy`

测量来自 DVL BI/BS 帧（经 `DvlRtPreprocessor` 预处理）：

* 仅在 `valid && has_body_xy && bottom_lock && BottomTrack` 时启用；
* 使用当前状态 yaw（来自 ESKF 姿态）+ `math::body_to_enu_yaw_only` 将 BI 体速投影到 ENU，得到 `[vE, vN]`；
* 对剔除噪声地板后的水平速度：

  * 若测量速度幅值小于阈值 → 视为 ZUPT（水平速度接近 0）；
  * 进入 2D ESKF 更新（含 ZUPT 模式 / 非 ZUPT 模式）；
* 支持 NIS / 速度比值门控，支持软膨胀 R（噪声协方差），并记录详细诊断信息。

这一策略体现了当前工程假设：**水平速度主要相信 BI 体速 + 当前 yaw，IMU 提供姿态，DVL 提供地速大小与方向约束。**

#### 2.3.4 DVL 垂向更新 `update_dvl_z`

测量来自 DVL BE 帧（Earth frame）：

* 仅在 `valid && has_enu_vel && bottom_lock && BottomTrack` 且配置 `enable_dvl_z_update=true` 时启用；
* 使用 ENU 垂向速度 `vU` 作为观测，对 ESKF 中的 vU（以及间接的 pU、姿态、bias）进行校正；
* 带有 NIS 门控、S 正定性检查等防御逻辑。

#### 2.3.5 yaw 伪观测 `update_yaw`

用于注入外部航向信息（例如 IMU 内部解算 yaw 或离线估计结果）：

* 衡量测量 yaw 与当前状态 yaw 的差值，做 wrap 到 [-π, π]；
* 使用一维量测模型，只作用在误差姿态 δθz 上；
* 进行标准的 Kalman 更新，并支持 NIS / S 检查与拒绝策略；
* 有利于抑制纯 IMU 积分导致的 yaw 漂移。

---

### 2.4 IO 与对外接口

#### 2.4.1 二进制日志写入 `BinLogger`

* 根据 `nav_daemon.yaml` 中的 `logging` 配置自动创建目录：

  * base_dir（如 `data`）下按日期分目录：`data/YYYY-MM-DD/nav`；
* 可按开关记录：

  * IMU 预处理后的 `ImuSample`；
  * DVL 预处理后的 `DvlSample`；
  * ESKF 内部更新诊断（如启用）；
  * 导航状态 `NavState` 快照；
* 日志结构在 `io/log_packets.hpp` 中定义，兼容离线 Python 工具与后续 C++ 回放器。

#### 2.4.2 共享内存发布 `NavStatePublisher`

* 通过 POSIX shm 暴露 `shared::msg::NavState` 布局；
* 使用 seqlock 协议保证读取一致性；
* 控制端只需按照统一 shm 名称（如 `/rov_nav_state_v1`）即可零拷贝读取最新导航状态。

---

## 3. 导航守护进程 `uwnav_navd` 的运行与行为

导航主程序相关文件：

* `app/nav_daemon.cpp`：`main()`，**只做 wiring，不写业务逻辑**；
* `app/nav_daemon_runner.cpp`：`run_nav_daemon(...)`，实现主循环；
* `app/nav_daemon_config.*`：从 YAML 读取配置。

### 3.1 构建示例

在 UnderwaterRobotSystem 根目录下（包含 `shared/` 和 `Underwater-robot-navigation/nav_core/`）：

```bash
cd Underwater-robot-navigation/nav_core
mkdir -p build
cd build

cmake ..          # 可加 -DCMAKE_BUILD_TYPE=RelWithDebInfo 等
make -j4
```

完成后，通常会得到：

```text
build/
├── libnav_core.a
└── bin
    ├── uwnav_navd
    └── uwnav_dvl_selftest
```

### 3.2 启动示例

在 `nav_core/build` 目录下运行：

```bash
./bin/uwnav_navd \
  --config      ../config/nav_daemon.yaml \
  --eskf-config ../config/eskf2d.yaml
```

其中：

* `--config`：导航守护进程配置（串口、预处理参数、日志、共享内存等）；
* `--eskf-config`：ESKF 参数配置（噪声、初值、NIS 门控、DVL Z 更新开关等）。

### 3.3 主循环逻辑（简述）

`run_nav_daemon(...)` 主循环（配置频率，如 20 Hz）大致流程：

1. 从共享状态中取出最近一帧 IMU / DVL 原始帧：

   * IMU/DVL 驱动在后台线程中持续填充 `SharedSensorState`；
2. 时间检查：

   * 计算 IMU / DVL 的“数据年龄”（当前 monotonic_now - last_mono_ns）；
   * 若超过配置的最大容忍时间，认为掉线或暂不可用；
3. **IMU 管线：Frame → ImuSample → ESKF 传播**

   * 将 `ImuFrame` 映射为 `ImuSample`（字段复制、时间戳补全）；
   * 交给 `ImuRtPreprocessor` 做实时预处理（bias + 重力 + 低通）；
   * 若预处理成功，调用 `EskfFilter::propagate_imu()` 完成一次 IMU 传播；
   * 如配置开启日志，则通过 `BinLogger` 记录 `ImuSample`；
4. **DVL 管线：Frame → DvlSample → DvlRtPreprocessor → ESKF 更新**

   * 将 `DvlFrame` 映射为 `DvlSample`（统一字段）；
   * 交给 `DvlRtPreprocessor::process()`，获得水平 BI 体速 / BE 垂向速度候选；
   * 门控通过后，调用：

     * `eskf.update_dvl_xy()` 进行水平更新；
     * 若 `enable_dvl_z_update=true` 则进一步 `eskf.update_dvl_z()`；
   * 如配置开启日志，则记录预处理后的 `DvlSample`；
5. **ESKF → NavState（唯一出口）**

   * 调用 `eskf.state()` 获得名义状态；
   * 映射为 `shared::msg::NavState`：

     * 位置 pos(E,N,U)、速度 vel(E,N,U)、姿态 rpy；
     * 深度 `depth = -U`；
     * 健康状态与标志位（IMU 是否超时、DVL 是否在线）；
6. **对外发布与落盘**

   * 若 `NavStatePublisher` 初始化成功，则将 `NavState` 写入共享内存；
   * 若日志开关开启，则落盘当前 `NavState`；
7. **终端状态打印**

   * 每若干循环打印一次简要状态，包括：

     * 当前 t_ns；
     * pos(E,N,U)、depth、yaw；
     * health / status_flags；
   * 方便在 SSH 终端查看实时导航情况。

---

## 4. 导航信息的终端输出方式

### 4.1 通过 `uwnav_navd` 自身日志

导航守护进程会周期性打印状态概要，例如：

```text
[nav_daemon] NAV t_ns=1234567890123 pos(E,N,U)=(+0.23, -0.15, -0.05) depth=0.05 yaw=+1.57 health=1 flags=0x0003
```

典型信息包含：

* 当前 ENU 位置与深度；
* 当前 yaw（rad）；
* 导航健康状态（IMU/DVL 是否在线）；
* 状态标志位（如 IMU_OK / DVL_OK）。

这类日志非常适合在水池实验中直接通过 SSH 观察：

```bash
ssh orangepi@<ip>
cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core/build
./bin/uwnav_navd --config ../config/nav_daemon.yaml --eskf-config ../config/eskf2d.yaml
```

### 4.2 通过单独的“监视器程序”读取共享内存

如果需要更丰富的终端 UI（如表格、趋势、颜色高亮），可以：

1. 在上位机或板端写一个小程序，通过 `NavStatePublisher` 的 shm 布局读取最新 `NavState`；
2. 使用 C++/ncurses 或 Python/curses 定期刷新画面。

当前 `nav_core` 已经提供统一的共享内存接口，无需修改导航主循环，只需新写一个“NavState 监控程序”。

---

## 5. 当前版本的边界与下一步规划

当前 C++ 导航子系统聚焦于：

* IMU + DVL 的板端实时采集与预处理；
* 基于 ESKF 的 ENU 位置 / 速度 / 姿态在线解算；
* 共享内存发布 + 二进制日志记录；
* 基本的终端监控能力。

暂未完全纳入本版本的内容包括：

* 深度传感器 / USBL / DVL 里程（dE/dN/dU）等更多传感器的联合 ESKF 融合；
* C++ 版因子图平滑（`graph_smoother_2d.*`）与 `NavHealthMonitor` 的上线；
* 基于平滑轨迹 vs 在线轨迹偏差的自动停机策略；
* 图形化 GCS 级轨迹可视化。

下一步典型演进方向：

1. 在现有 ESKF 框架中，逐步接入深度 / USBL / DVL 里程因子，形成完整 3D/6DoF INS+辅助导航；
2. 打通 C++ 因子图平滑链路，支持在线日志 → 离线平滑 → 精度评估 → 导航健康监控；
3. 针对控制系统需求，定义 NavState 的扩展字段与 QoS 指标（延迟、抖动、置信度等）；
4. 为新人补充更多文档，例如：

   * “如何在 offline_nav 中用 Python 重现 nav_core 的 ESKF 轨迹”；
   * “如何根据 IMU/DVL 实验数据调参 EskfConfig”。

---

```

