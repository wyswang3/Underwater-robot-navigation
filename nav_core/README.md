````md
# nav_core —— 水下机器人在线导航 C++ 子系统

`nav_core` 是 UnderwaterRobotSystem 中的「在线导航子项目」，运行在香橙派（OrangePi）等板卡上，负责：

- 通过串口采集 **IMU（WitMotion HWT9073-485, Modbus-RTU）** 与 **DVL（Hover H1000, PD6/EPD6 协议）** 的原始数据；
- 在板端进行 **IMU / DVL 实时预处理**：
  - IMU：去零偏、扣重力、统一机体系为 body=FRD，输出“线加速度”；
  - DVL：仅保留 BottomTrack + BottomLock 数据，区分 BI 体速 与 BE 地速，做噪声地板、幅值和离底距离门控；
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

## 0. 工程根目录与依赖约定（非常重要）

整个系统的目录约定如下（只列关键结构）：

```text
UnderwaterRobotSystem/
├── shared/
│   └── msg/
│       └── nav_state.hpp        # 导航状态的共享定义（供控制/上位机复用）
└── Underwater-robot-navigation/
    └── nav_core/
        ├── CMakeLists.txt
        ├── include/
        ├── src/
        ├── config/
        └── third_party/
            └── witmotion/
                ├── REG.h
                ├── wit_c_sdk.c
                └── wit_c_sdk.h
````

`nav_core` 的 CMake 假定：

* 当前 `CMakeLists.txt` 所在目录为：

  * `UnderwaterRobotSystem/Underwater-robot-navigation/nav_core`
* 工程根目录为：

  * `UNDERWATER_ROOT = ${CMAKE_CURRENT_SOURCE_DIR}/../..`
  * 即 `UnderwaterRobotSystem` 目录

因此：

* 代码中可以直接使用：

  ```cpp
  #include "shared/msg/nav_state.hpp"
  ```

  CMake 会通过 `target_include_directories(nav_core ... ${UNDERWATER_ROOT})`
  把 `UnderwaterRobotSystem/` 加进 include path；
* **如果后人改了目录结构（例如把 nav_core 单独拎出去）又不改 CMake，编译一定会报 `shared/msg/nav_state.hpp` 找不到。**
  此时要么：

  * 保持上述目录结构；要么
  * 在 CMake 中显式设置新的 `UNDERWATER_ROOT` 路径并保证其中有 `shared/msg/nav_state.hpp`。

IMU 厂家 C SDK 被放在：

```text
nav_core/third_party/witmotion/
    ├── REG.h
    ├── wit_c_sdk.c
    └── wit_c_sdk.h
```

CMake 已经通过：

```cmake
target_include_directories(nav_core
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${UNDERWATER_ROOT}
        ${CMAKE_CURRENT_SOURCE_DIR}/third_party/witmotion
)
```

把该目录加入 include path，并通过 `NAV_CORE_SOURCES` 把 `wit_c_sdk.c` 编进静态库（由源码列表控制）。
**如果后人改动 third_party/witmotion 的文件名或路径，而没有同步更新 `CMakeLists.txt` 中的源文件列表/包含路径，编译就会在 IMU 驱动处报错。**

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
│       │   ├── graph_smoother_2d.cpp    # 因子图平滑实现（默认由 CMake option 控制）
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
```

编译后，会生成：

* 静态库：`libnav_core.a`（供其他工程复用）；
* 导航守护进程：`uwnav_navd`（通常位于 `build/bin/uwnav_navd`）；
* DVL 自检工具：`uwnav_dvl_selftest`（位于 `build/bin/uwnav_dvl_selftest`）。

---

## 2. 当前 C++ 导航子系统已具备的功能

### 2.1 设备驱动层

#### 2.1.1 IMU 驱动 `ImuDriverWit`

* 使用厂家 WitMotion C SDK（`third_party/witmotion/wit_c_sdk.c`）封装，实现：

  * 串口打开 / 配置（OrangePi 上通常为 `/dev/ttyUSB0`，波特率 230400）；
  * Modbus-RTU 读寄存器循环；
  * 通过 C 回调解码寄存器数组 `sReg[]`。

* 对上层输出统一的 `ImuFrame`（定义在 `core/types.hpp` 中）：

  * 单调时间戳 `mono_ns`（Monotonic Clock）；
  * 系统时间戳 `est_ns`（System Clock，用于与其他设备对齐）；
  * 机体系角速度 `gyro_rad_s[3]`（rad/s，body=FRD）；
  * 机体系加速度 `acc_g[3]` 或 `acc_mps2_raw[3]`（含重力）；
  * 欧拉角 `euler_rad[3]`（roll / pitch / yaw, rad，来自 IMU 内部解算）；
  * 温度 / 状态字等可选字段；
  * 有效标志 `valid` 和状态 `status`。

* 若后人修改 WitMotion SDK 文件名或函数签名，需要同步更新：

  * `third_party/witmotion/wit_c_sdk.c/.h`
  * `drivers/imu_driver_wit.*`
  * 以及 `CMakeLists.txt` 中的源文件列表，否则编译阶段会在链接 `libnav_core.a` 时出错。

#### 2.1.2 DVL 驱动 `DvlDriver`

* 通过串口阻塞读取 PD6/EPD6 文本帧（Hover H1000，常用波特率 115200）；

* 使用 `dvl_protocol.*` 解析为结构化数据 `ParsedLine`：

  * 区分底跟踪 / 水团；
  * 区分坐标系：Instrument/Ship frame（BI/BS）、Earth frame（BE）；
  * 获取底速、位移、离底距离、FOM、相关系数、A/V 有效标志等。

* 对上层输出统一的 `DvlRawSample`（在 `preprocess/dvl_rt_preprocessor.hpp` 中定义）：

  ```cpp
  struct DvlRawSample {
      MonoTimeNs mono_ns{0};
      MonoTimeNs est_ns{0};
      std::uint8_t kind{0};     // 0=BI, 1=BE, 2=BS, 3=BD
      bool   bottom_lock{false};
      Vec3f  vel_inst_mps;      // BI: 仪器坐标系速度 (I-frame)
      Vec3f  vel_earth_mps;     // BE: Earth/ENU (?) 速度
      float  range_m[4];        // 各 beam 测距
      float  corr[4];           // 各 beam 相关系数
      float  fom;               // Figure-of-merit，如有
      std::uint32_t status;     // 厂家 status bitmask，如有
  };
  ```

* 这里特别注意：

  * `kind` 决定当前帧被视作 BI/BE/BS/BD；
  * `vel_inst_mps` 与 `vel_earth_mps` 的物理语义来自驱动，对应 PD6/EPD6 中的字段；
  * `range_m[]` / `corr[]` / `bottom_lock` / `fom` / `status` 用于后续门控；
  * **如果后人调整 `DvlRawSample` 字段，必须同步更新 `dvl_rt_preprocessor.cpp`、`dvl_driver.cpp`、`types.hpp`，否则会出现编译成功但运行时语义错乱的隐蔽 bug。**

* DVL 驱动同时提供命令接口：

  * `CZ`：停机，避免空气中 ping；
  * `CS`：下水后启动 ping；
  * 自检工具 `uwnav_dvl_selftest` 在 `tools/DVL_selftest.cpp` 中使用这些接口做安全检查。

---

### 2.2 实时预处理层

本层的目标是：**把“硬件原始输出”统一转换为 ESKF 能够直接使用的、物理语义清晰的数据流**。

#### 2.2.1 IMU 实时预处理 `ImuRtPreprocessor`

实现于 `preprocess/imu_rt_preprocessor.*`，对 `ImuFrame` 做以下处理：

* 坐标与单位统一：

  * 角速度单位统一为 rad/s，机体系固定为 body=FRD（X 前 / Y 右 / Z 下）；
  * 线加速度统一为 m/s²；
  * Acc 原始单位若为 g，则通过配置 `imu_raw_g_to_mps2`（例如 9.78）转换为 m/s²。

* 零偏估计与去除：

  * 在配置的静止时间窗内（例如开机前前 20s），检测 gyro/acc 是否处于静止；
  * 对静止窗数据估计加速度 bias 与陀螺 bias；
  * 后续实时数据减去 bias，得到零偏校正后的测量。

* 重力扣除：

  * 使用当前 roll/pitch（来自 IMU 内部解算或预处理内部积分），估计重力在 body=FRD 下的分量；
  * 从加速度中扣除重力，输出“线加速度 a_linear”。

* 噪声处理：

  * 对线加速度做 deadzone / 一阶低通，抑制高频噪声；
  * 对异常突变做限幅保护，避免积分漂移被单次异常拉偏。

* 输出结构 `ImuSample`：

  * `ang_vel_b_rad_s[3]`：零偏校正后的角速度；
  * `lin_acc_b_mps2[3]`：**已经去零偏、扣重力的线加速度**；
  * `rpy_rad[3]`：roll/pitch/yaw（可选，用于诊断与 yaw 伪观测）；
  * 同步的 `mono_ns` / `est_ns`；
  * 状态标志位等。

> 重要：
>
> * ESKF 配置中应设置 `imu_acc_source: "processed"` 且 `imu_acc_kind: "linear"`，表示传给 ESKF 的是已经扣重力的线加速度。
> * 如果后人改成 ESKF 内部再扣重力，需要同时修改：
>
>   * `ImuRtPreprocessor` 的输出语义；
>   * `EskfConfig` 中的 `imu_acc_kind`；
>   * `eskf_propagate.cpp` 中的加速度处理逻辑。

#### 2.2.2 DVL 实时预处理 `DvlRtPreprocessor`

实现于 `preprocess/dvl_rt_preprocessor.*`，对 `DvlRawSample` 做门控与坐标系归一化，输出 `DvlRtOutput`。

**输入：`DvlRawSample`（BI/BE/BS/BD 混流）**

关键字段：

* `kind`：0=BI, 1=BE, 2=BS, 3=BD；
* `vel_inst_mps`：仪器坐标系速度（BI）；
* `vel_earth_mps`：地球坐标系速度（BE，通常可视为 ENU）；
* `range_m[4]` / `corr[4]`：四束测距与相关系数；
* `bottom_lock`：锁底状态；
* `status`：厂家 status bitmask。

**输出：`DvlRtOutput`**

```cpp
struct DvlRtOutput {
    MonoTimeNs mono_ns{0};
    MonoTimeNs est_ns{0};

    bool   has_be{false};
    Vec3d  vel_be_enu{0.0, 0.0, 0.0};   // ENU 速度（优先给 ESKF 用）

    bool   has_bi{false};
    Vec3d  vel_bi_body{0.0, 0.0, 0.0};  // body(FRD) 速度（可选，用于诊断 / 算法）

    bool   has_alt{false};
    double alt_bottom_m{0.0};           // DVL 底距（若有），通常来自 beam range

    bool   bottom_lock{false};          // 预处理判定的锁底状态

    bool   gated_ok{false};             // 是否通过门控（true 才推荐用于 ESKF 更新）
    std::uint32_t reason_code{0};       // 门控失败 bitmask（0 表示 OK）

    float mean_corr{0.0f};              // beams 有效相关系数平均值
    float min_range_m{0.0f};            // beams 中最小测距（海底距离粗略替代）
};
```

**配置：`DvlRtPreprocessConfig` 关键参数**

* 时间与基本开关：

  * `max_gap_s`：帧间最大允许间隔，超出可触发降级逻辑；
  * `enable_gating`：是否开启门控总开关。

* 门控阈值：

  * `min_corr` / `min_good_beams`：相关系数下限与最少合格 beam 数；
  * `min_alt_m` / `max_alt_m`：离底距离范围；
  * `max_abs_speed_mps`：速度模长上限；
  * `dvl_noise_floor_mps`：噪声地板，小于该模长视为 0；
  * `require_bottom_lock`：是否必须锁底；
  * `status_mask_ok`：厂家 status bitmask 的允许值（预留）。

* 坐标系 / 安装角：

  * `mount_yaw_rad`：DVL 仪器坐标系相对 ROV 体坐标系的固定 yaw 偏差；
  * `use_be_as_enu`：若 true，则 BE vel_earth 直接视为 ENU；否则可只用 BI + yaw 做 ENU 转换。

**门控与坐标系处理逻辑（简述）**

1. **Beam 质量与离底距离**：

   * 从 `range_m[4]` 和 `corr[4]` 统计：

     * 有效测距数量；
     * 有效相关系数数量与平均值；
     * 最小离底距离 `min_range_m`；
   * 若 beam 数不足 / 相关系数太低 → `kReject_Quality`；
   * 若离底距离不在 `[min_alt_m, max_alt_m]` → `kReject_Altitude`；
   * 这些信息都会写入 `out.mean_corr`、`out.min_range_m`、`out.alt_bottom_m`、`out.has_alt` 等。

2. **BI / BS：I-frame → body(FRD)**

   * 当 `kind` 为 BI/BS 时：

     * 取 `vel_inst_mps` 转为 double 得到 `v_inst`；
     * 通过 `mount_yaw_rad` 做 I→B 的 yaw 旋转，得到 `v_body`；
     * 计算 `|v_body|`，超过 `max_abs_speed_mps` → `kReject_SpeedTooBig`；
     * 小于 `dvl_noise_floor_mps` → 视为 0；
     * 通过则：

       * `out.has_bi = true`；
       * `out.vel_bi_body = v_body`；
       * 标记“有速度观测”。

3. **BE：直接视作 ENU 速度**

   * 当 `kind` 为 BE 且 `use_be_as_enu=true` 时：

     * 取 `vel_earth_mps` 视作 ENU 速度 `v_enu`；
     * 同样做模长限制与噪声地板；
     * 通过则：

       * `out.has_be = true`；
       * `out.vel_be_enu = v_enu`；
       * 标记“有速度观测”。

4. **整体门控与输出**

   * 若没有任何速度观测（BI/BE 均无效或被拒绝） → `kReject_NoVelocity`；
   * `require_bottom_lock=true` 且 `bottom_lock=false` → `kReject_BottomLock`；
   * 汇总所有 reason bit：

     * 若 `enable_gating=true` 且 `reason!=0` → `out.gated_ok=false`，`process()` 返回 false；
     * 否则，若至少一个速度观测存在，则 `out.gated_ok=true`，并更新内部统计与 `last_output`。

**与 ESKF 的接口约定**

在导航主循环中，DVL 部分的数据流为：

```text
DvlDriver → DvlRawSample → DvlRtPreprocessor → DvlRtOutput → ESKF
```

* 水平（XY）更新：使用 `out.has_bi` + `out.vel_bi_body`，在 ESKF 内部用当前 yaw 做 body→ENU 转换；
* 垂向（Z）更新：使用 `out.has_be` + `out.vel_be_enu.z` 作为 vU 观测；
* **如果后人修改 DvlRtOutput 的字段名（例如从 has_bi 改回 has_body_xy），必须同步修改：**

  * `eskf_update_dvl.cpp` 中的 `update_dvl_xy` / `update_dvl_z`；
  * `nav_daemon_runner.cpp` 中对 DvlRtPreprocessor 的调用；
  * 否则编译会直接失败或者运行时用错字段。

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

误差状态维度在 `filters/filter_common.hpp` 中统一定义，典型为：

```text
δx = [δp(0-2), δv(3-5), δθ(6-8), δba(9-11), δbg(12-14)]ᵀ
```

名义状态包含：

* 位置 `p_enu`（E, N, U）；
* 速度 `v_enu`（vE, vN, vU）；
* 姿态四元数 `q_nb`（nav←body）及其 RPY 表示 `rpy_nb_rad`；
* 加速度零偏 `ba_mps2`；
* 陀螺零偏 `bg_rad_s`。

坐标系约定：

* 机体系：body=FRD（X 前 / Y 右 / Z 下）；
* 导航系：ENU（East / North / Up），**深度 `depth = -U`**；
* 重力：g 向下 -> ENU.z 为负。

#### 2.3.2 IMU 传播 `propagate_imu`

输入：IMU 预处理后的 `ImuSample`，满足：

* `lin_acc_b_mps2`：已去 bias、扣重力的线加速度；
* `ang_vel_b_rad_s`：已去 gyro bias 的角速度；
* `mono_ns`：单调时间戳。

传播步骤（简略）：

1. 由当前 `mono_ns` 与上一时刻时间计算 `dt`，并做 `dt_min` / `dt_max` 检查；
2. 积分姿态（通过 `eskf_math` 工具）；
3. 将 `lin_acc_b` 通过当前姿态旋转到 ENU，若 `imu_acc_kind="linear"` 则不再补重力；
4. 积分速度与位置，并做限幅/泄漏设计；
5. 根据 IMU 噪声 / bias 漂移构造 F、G、Qd，传播协方差。

**ESKF 与 IMU 预处理之间最敏感的接口就是“加速度是否已扣重力”。任何一个地方变了，另一个必须同步改。**

#### 2.3.3 DVL 水平更新 `update_dvl_xy`

* 观测：`z = [vE, vN]`；

* 从 `DvlRtOutput` 取 `has_bi=true` 且 `gated_ok=true` 的样本：

  * 用 `vel_bi_body` 通过 `math::body_to_enu_yaw_only`（当前 yaw + mount_yaw_rad）转换为 ENU 水平速度；

* 构造观测模型：

  ```text
  h(x) = v_EN = [vE, vN]ᵀ
  ```

* 进行标准 Kalman 更新，并有 NIS 门控 / S 正定性检查 / 噪声膨胀等逻辑；

* 如果将来希望用 BE 水平速度替代 BI，也需要同步修改：

  * `DvlRtPreprocessConfig::use_be_as_enu`；
  * `eskf_update_dvl.cpp` 中对应的观测构造。

#### 2.3.4 DVL 垂向更新 `update_dvl_z`

* 观测：`z = vU`；

* 从 `DvlRtOutput` 中使用 `has_be=true` 的 ENU 速度分量；

* 一维观测模型：

  ```text
  h(x) = v_EN.z = vU
  ```

* 用于抑制 IMU 垂向积分漂移，与深度传感器联合时，通常效果更好。

#### 2.3.5 yaw 伪观测 `update_yaw`

* 观测：`z = yaw_meas`（来源可为 IMU 内部解算或其他外部航向）；
* 误差：`δψ = wrap(z - yaw_state)`；
* 仅作用在姿态误差的 yaw 分量，做一维 Kalman 更新；
* 通过 NIS 门控避免激进航向观测把滤波器“拽崩”。

---

### 2.4 IO 与对外接口

#### 2.4.1 二进制日志写入 `BinLogger`

* 根据 `nav_daemon.yaml` 中的 `logging` 配置自动创建目录：

  ```text
  base_dir/YYYY-MM-DD/nav/
  ```

* 可按开关记录：

  * IMU 预处理后的 `ImuSample`；
  * DVL 预处理后的 `DvlRtOutput` / `DvlSample`；
  * ESKF 更新诊断（可选）；
  * 导航状态 `NavState`。

* 日志结构在 `io/log_packets.hpp` 中定义；
  离线 Python 项目 `offline_nav` 会读取这些日志进行重放 / ESKF 重算 / 因子图平滑。

**如果后人增加日志字段，记得同时更新：**

* `log_packets.hpp` 中对应结构；
* `BinLogger` 的写入逻辑；
* Python 侧解析脚本（否则会解析失败或字段错位）。

#### 2.4.2 共享内存发布 `NavStatePublisher`

* 通过 shm 暴露 `shared::msg::NavState`；
* 使用 seqlock 协议保证读取一致性；
* 控制进程只需遵循同一个 shm 名称和结构定义，不需要关心 ESKF 内部细节。

---

## 3. 导航守护进程 `uwnav_navd` 的运行与行为

导航主程序相关文件：

* `app/nav_daemon.cpp`：`main()`，只做参数解析 + wiring；
* `app/nav_daemon_runner.cpp`：`run_nav_daemon(...)`，主循环实现；
* `app/nav_daemon_config.*`：从 YAML 读取配置。

### 3.1 构建示例

```bash
cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
mkdir -p build
cd build

cmake ..
make -j4
```

生成：

```text
build/
├── libnav_core.a
└── bin
    ├── uwnav_navd
    └── uwnav_dvl_selftest
```

### 3.2 启动示例

在 `nav_core` 目录下：

```bash
cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
./build/bin/uwnav_navd \
  --config      config/nav_daemon.yaml \
  --eskf-config config/eskf2d.yaml
```

**注意：**

* `uwnav_navd` 默认以当前工作目录为基准找 `config/xxx.yaml`，
  所以建议在 `nav_core` 根目录下运行；
* 若后人从 `build` 目录下直接运行，需要调整相对路径，例如 `../config/...`。

### 3.3 主循环逻辑（数据流）

简化版数据流示意：

```text
         +---------------------+
         |  imu_driver_wit     |
         +----------+----------+
                    |
                    v  ImuFrame
         +---------------------+
         | ImuRtPreprocessor   |
         +----------+----------+
                    |
                    v  ImuSample
         +---------------------+
         |      EskfFilter     |
         |  propagate_imu()    |
         +----------+----------+
                    |
                    v
       +-------------------------+
       | DvlDriver + Protocol    |
       +-----------+-------------+
                   |
                   v  DvlRawSample
       +-------------------------+
       |   DvlRtPreprocessor     |
       +-----------+-------------+
                   |
                   v  DvlRtOutput
       +-------------------------+
       |      EskfFilter         |
       | update_dvl_xy/z/yaw()   |
       +-----------+-------------+
                   |
                   v
         +---------------------+
         |   NavState 输出     |
         +----------+----------+
                    |
         +----------+----------+
         |   BinLogger (日志)  |
         +----------+----------+
         | NavStatePublisher   |
         +---------------------+
```

每个循环周期大致执行：

1. 从 IMU 驱动读取最新 `ImuFrame` → 预处理 → `ImuSample` → `eskf.propagate_imu()`；
2. 从 DVL 驱动读取最新 `DvlRawSample` → `dvl_pp.process()` → `DvlRtOutput`：

   * 若 `gated_ok=true`，则调用 `update_dvl_xy` / `update_dvl_z`；
3. 从 ESKF 拿状态，映射为 `NavState`；
4. 写日志 + 写共享内存；
5. 定期在终端打印导航 summary 便于现场观察。

---

## 4. 导航信息的终端输出

### 4.1 `uwnav_navd` 自身日志

程序会定期打印导航摘要，例如：

```text
[nav_daemon] NAV t=123.456s pos(E,N,U)=(+0.23,-0.15,-0.05) depth=0.05 yaw=+1.57 imu=OK dvl=OK
```

可在香橙派上直接：

```bash
ssh orangepi@<ip>
cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
./build/bin/uwnav_navd --config config/nav_daemon.yaml --eskf-config config/eskf2d.yaml
```

观察实时导航状态。

### 4.2 共享内存 + 其它监控程序

控制程序或上位机可以通过 `NavStatePublisher` 的 shm 接口读取 `NavState`：

* 用于控制闭环；
* 用于上位机 UI 显示轨迹 / 速度 / 姿态等。

---

## 5. 对后续维护者的提醒（避免踩坑）

1. **不要随意改目录结构**

   * `nav_core` 的 CMake 假定了 `UnderwaterRobotSystem/` 根目录下存在 `shared/`。
   * 如果要拆分 repo 或移动目录，务必同步修改：

     * CMake 中的 `UNDERWATER_ROOT`；
     * 控制项目中 `#include "shared/msg/nav_state.hpp"` 的路径设计。

2. **改 DVL/IMU 数据结构要“一条龙改完”**
   涉及以下几层：

   ```text
   drivers/*    → core/types.hpp / dvl_rt_preprocessor.hpp
                → preprocess/*   → estimator/eskf_update_*.cpp
                → app/nav_daemon_runner.cpp（管线连接处）
   ```

   任意一层字段变动而另一层不改，编译可能通过，但运行时数据含义就错了。

3. **改“加速度是否扣重力”要同步改 ESKF 配置与 propagate 逻辑**

   * 若 IMU 预处理输出的是“带重力的加速度”，ESKF 内部需要补重力；
   * 现在的工程假设是：**IMU 预处理已经扣重力，ESKF 只做坐标变换和积分**。
   * 对应配置项包括：

     * `eskf.yaml` 中的 `imu_acc_kind`；
     * `imu_processing` / `imu_rt_preprocessor` 中的处理逻辑。

4. **不要在 nav_daemon 中“临时改逻辑”**
   `nav_daemon.cpp` + `nav_daemon_runner.cpp` 只负责 wiring 和循环调度。
   真正的业务逻辑应分别放在：

   * 设备相关：`drivers/*`；
   * 预处理相关：`preprocess/*`；
   * 估计相关：`estimator/*`；
   * IO：`io/*`。

   这样以后无论是做 offline_nav、因子图平滑，还是把同样的 ESKF 移植到别的项目，都能复用同一套核心模块。

5. **编译报错时优先检查 CMake 与 include path，而不是“乱删代码”**

   * 典型错误：

     * 找不到 `shared/msg/nav_state.hpp` → 工程根路径不对；
     * 找不到 `wit_c_sdk.h` → third_party/witmotion 路径或 CMake 源文件列表不匹配；
     * undefined reference to `WitInit` 等 → `wit_c_sdk.c` 没有被编进 `libnav_core.a`；
   * 遇到这类问题，先对照本 README 中的“工程根目录约定”和 `CMakeLists.txt`，确认路径/源文件/选项是否齐全。

---

当前版本的 `nav_core` 已经能够在板端完成：“IMU+DVL 实时预处理 → ESKF 在线解算 → NavState 发布+日志记录”的完整流水线。
后续如需扩展（深度 / USBL / 因子图平滑 / 导航健康监控等），优先在 `estimator/*` 和 `preprocess/*` 中增加模块，并在本 README 的“数据流”章节同步更新结构说明，避免后人只看到某个 cpp 报错却不知道上游/下游依赖关系。

```
::contentReference[oaicite:0]{index=0}
```
