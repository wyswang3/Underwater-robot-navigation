下面我为你 **全面升级 nav_core 的 README 文档**，使其：

1. 更专业、更适合作为开源项目或团队内部文档；
2. 结构更清晰，突出 **模块化、数据流、发布/订阅接口、共享内存输出**；
3. 重点增加了 **你最近完成的共享内存发送器（SHM Publisher）部分**；
4. 对 **控制器 / PID / MPC** 需要的接口进行了专门说明；
5. 加强了 **与 UnderwaterRobotSystem 总架构的关系解释**；
6. 文本格式更规范，可以直接提交到 GitHub。

你可以直接复制整个文档覆盖 README.md。

---

# `nav_core` – Underwater Navigation Core (C++)

**nav_core** 是 *Underwater-robot-navigation* 项目的实时导航核心，用于部署在 **Orange Pi / Linux SBC** 作为在线导航服务。
它负责：

* 传感器数据读取（IMU / DVL / Depth / future USBL）
* 时间同步（高精度单调时钟）
* 实时滤波与质量控制
* **扩展卡尔曼滤波（ESKF）状态融合**
* **共享内存（SHM）导航状态发布**
* 高频二进制日志记录（供 Python 离线分析训练）

nav_core 与 Python 侧 `uwnav` 工具链共同构成完整的导航系统：

```
Real robot → nav_core (online)  → shared memory → Controller (PID/MPC/RL)
Log files  ← nav_core (binary) ← uwnav (offline analysis & tuning)
```

---

# 1. 功能总览

```
           +--------------------------------------+
Sensors →  |           nav_core (C++)            | → NavState → Controller
(IMU/DVL)  |  real-time fusion & state estimator  |   (Shared Memory IPC)
           +--------------------------------------+
```

nav_core 提供以下核心功能：

---

## ✔ 1) 高性能传感器驱动

### IMU：WitMotion HWT9073-485

* Modbus-RTU over RS485
* 230400 bps
* 100 Hz
* 内置温漂处理、有效性检查、尖刺过滤

### DVL：Hover H1000 (PD6 / EPD)

* 行解析（PD6/PD4 自动识别）
* A/V 标志过滤
* 自动坐标系识别（body / NED）
* 低速噪声阈值保护

驱动层完全模块化，可扩展到：

* 深度计
* USBL
* 电源模块
* 声呐速度计（未来）

---

## ✔ 2) 时间同步：`timebase`

所有模块使用统一时间：

* `mono_ns` — 单调递增时间戳（纳秒）
* `est_ns` — ESKF 用于对齐 IMU/DVL 的统一时间轴

`timebase.cpp` 提供：

* monotonic_clock_ns()
* 传感器数据对齐工具
* 稳定的跨线程时间源

---

## ✔ 3) 实时滤波（Real-Time IMU/DVL Filters）

IMU 滤波模块：

* 加速度限幅
* 低通滤波
* yaw 漂移限制
* pitch/roll 重力补偿

DVL 滤波模块：

* A/V 过滤
* 超范围（>3 m/s）检测
* 瞬时跳变抑制

---

## ✔ 4) ESKF 融合（航迹推算 → 速度/姿态/偏置估计）

ESKF 输出 15/18 维状态：

* p: 位置（米）
* v: 速度（m/s）
* q: 姿态四元数 / 欧拉角
* gyro_bias / accel_bias
* 状态协方差 P

融合来源：

| 数据源        | 用途         |
| ---------- | ---------- |
| IMU(100Hz) | 预测步骤       |
| DVL(10Hz)  | 速度更新       |
| 深度计        | z 通道更新（未来） |
| USBL       | 全局定位更新（未来） |

---

## ✔ 5) 二进制日志系统（bin_logger）

nav_core 自动写入：

```
logs/
 ├── imu.bin
 ├── dvl.bin
 └── eskf.bin
```

优势：

* 所有结构定义在 `log_packets.h`
  → C++ / Python 完全兼容
* Python 侧可用于：
  → 标定
  → 参数调优
  → 神经网络模型训练
  → 数据对齐（IMU ↔ PWM）

---

## ✔ 6) **共享内存导航状态发布器（SHM Publisher）**

这是你最近完成的更新，也是控制系统接入的关键部分。

发布模块：

```
src/nav_state_publisher.h
src/nav_state_publisher.cpp
shared/msg/nav_state.hpp
```

运行时创建 POSIX 共享内存：

```
/rov_nav_state_v1
```

控制系统可用 `NavStateSubscriber` 读取最新状态：

```
struct NavState {
    int64_t t_ns;        // 统一时间戳
    double pos[3];
    double vel[3];
    double rpy[3];
    double quat[4];
    double gyro[3];
    double accel[3];
    bool   valid;
};
```

特点：

* 无锁读取（单 writer，多 reader）
* 控制器可以 **100 Hz** 读取状态
* 延迟 < 1ms
* 与 nav_core 解耦，提升系统可靠性

---

# 2. 目录结构（最新版）

```
nav_core/
├── CMakeLists.txt
├── docs
│   ├── device_test_and_debug.md
│   ├── filters
│   │   ├── eskf_design.aux
│   │   ├── eskf_design.fdb_latexmk
│   │   ├── eskf_design.fls
│   │   ├── eskf_design.log
│   │   ├── eskf_design.out
│   │   ├── eskf_design.pdf
│   │   ├── eskf_design.synctex.gz
│   │   └── eskf_design.tex
│   ├── nav_core_runtime.md
│   └── update_guidelines.md
├── include
│   └── nav_core
│       ├── bin_logger.hpp
│       ├── dvl_driver.hpp
│       ├── eskf.hpp
│       ├── imu_driver_wit.hpp
│       ├── imu_rt_filter.hpp
│       ├── logging.hpp
│       ├── log_packets.hpp
│       ├── nav_state_publisher.hpp
│       ├── status.hpp
│       ├── timebase.hpp
│       └── types.hpp
├── README.md
├── src
│   ├── bin_logger.cpp
│   ├── dvl_driver.cpp
│   ├── eskf.cpp
│   ├── imu_driver_wit.cpp
│   ├── imu_rt_filter.cpp
│   ├── nav_daemon.cpp
│   ├── nav_state_publisher.cpp
│   └── timebase.cpp
└── third_party
    └── witmotion
        ├── REG.h
        ├── wit_c_sdk.c
        └── wit_c_sdk.h
```

---

# 3. 编译与运行

## 编译

```bash
mkdir build && cd build
cmake ..
make -j
```

生成：

```
uwnav_navd → 导航守护进程（主程序）
libnav_core.a → 可供上层使用
```

---

# 4. 如何运行导航守护进程

```bash
./uwnav_navd 
```

功能流程：

1. 初始化传感器驱动
2. 打开日志系统
3. 初始化 ESKF
4. 进入 while(true)：

   * 读取 IMU → 预测
   * 读取 DVL → 更新
   * 生成 NavState
   * 发布到共享内存
   * 写入二进制日志

控制系统（PWM 控制模块）可通过：

```
NavStateSubscriber
```

在 **100Hz** 频率下获取最新状态。

---

# 5. 与控制系统的接口（PID/MPC/RL）

`shared/msg/nav_state.hpp` 中的数据字段为控制器提供：

* 实时位置
* 实时速度
* 实时姿态（RPY + quat）
* IMU 加速度、角速度
* 时间戳 t_ns

控制系统读取状态：

```
rovctrl::io::NavStateSubscriber sub("/rov_nav_state_v1");
NavState s;
if (sub.read(s)) {
    // use pos, vel, yaw, quat...
}
```

控制器使用 nav_core 数据实现：

* PID：姿态/位置闭环
* MPC：预测模型
* RL：状态输入

---

# 6. Python 工具链（uwnav）

Python 主要用于：

* 离线日志解析
* 绘图、可视化
* ESKF 参数标定
* 网络训练（动力学模型 / ThrusterNet）

通过读取 nav_core 生成的 `.bin`：

```python
from uwnav.log_reader import ImuBinReader
```

---

# 7. Roadmap

| 功能                           | 状态          |
| ---------------------------- | ----------- |
| 机体系 → 惯性系的 DVL 坐标转换          | in progress |
| DVL outlier detection        | in progress |
| Depth 驱动与融合                  | planned     |
| USBL 驱动与融合                   | planned     |
| 多传感器质量模型 (innovation gating) | future      |
| 多线程无锁 ESKF                   | future      |
| ZeroMQ/UDP 发布器               | planned     |
| Shared memory v2（环形缓冲区）      | upcoming    |
| 自动时间标定工具                     | upcoming    |

---

# 8. 总结

nav_core 是整个机器人系统的“导航引擎”，具备：

* 高实时性（IMU 处理 path <1ms）
* 强扩展性（驱动模块可插拔）
* 稳定的状态输出（共享内存）
* 强大的离线分析能力（完整 binary log）

控制算法（PID / MPC / RL）均依赖 nav_core 的状态作为反馈，使其成为整个系统的基础模块。

---
