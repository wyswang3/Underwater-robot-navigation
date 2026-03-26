````md
# 0. 本文档能帮你什么？

如果你是第一次接触本项目，这份 README 可以让你：

* 快速理解**项目整体结构**、Python & C++ 的分工以及传感器数据流向；
* 学会在 Orange Pi 上**编译与运行 C++ 在线导航子系统 `nav_core`**；
* 学会使用 Python 工具链**采集 / 校验 / 对齐 / 可视化 IMU / DVL / 电源数据**；
* 明白**当前工程进度**：哪些模块已稳定可用，哪些仍在迭代；
* 初步知道遇到常见问题（串口 / 波特率 / YAML 配置 / ESKF 无输出）时该改哪里、查什么。

如果你已经跟进了一段时间，这份文档则是：

* 项目“鸟瞰图 + 索引”：告诉你某个功能属于哪一层、在哪个目录、用了哪些配置文件；
* 一个“给新人看的参考”：方便你把整个 repo 甩给新人，让他自己就能先跑起来基本功能。

---

# 1. 项目定位（What is this system for?）

本系统是一套面向 **水下 ROV/AUV 的导航数据采集 + 在线导航解算** 的工程项目，由两部分构成：

* **Python 侧：数据采集 / 快速检查 / 简单对齐与可视化**

  - 串口读取 IMU / DVL / 电压电流（Volt32）数据；
  - 写入结构化日志文件，做基础数据清洗、可视化与 sanity check；
  - 提供一些“工具脚本”，简化调试与标定（比如切 IMU 波特率、检查 DVL 原始输出等）。

* **C++ 侧（`nav_core`）：运行在 Orange Pi 等 SBC 上的实时在线导航核心**

  - IMU（WitMotion HWT9073-485, Modbus-RTU）驱动；
  - DVL（Hover H1000, PD6/EPD6）驱动；
  - 实时预处理（IMU 去零偏 / 扣重力、DVL BottomTrack 门控与噪声地板）；
  - 基于 IMU + DVL 的误差状态卡尔曼滤波（ESKF），输出 ENU 轨迹 / 速度 / 姿态；
  - 导航状态通过共享内存发布给控制系统，同时记录二进制日志，为离线分析和 ML 提供数据。

项目的工程目标是：

> 在真实水池 / 海试环境中，从零开始打通“**传感器 → 在线导航 → 日志 → 离线分析 / 学习**”整条链路，  
> 为后续 MPC / RL 控制、深度学习动力学辨识等工作提供一条稳定的数据“河道”。

---

# 2. 项目目录结构（更新版）

当前仓库的大致结构如下（省略了一些次要文件）：

```text
.
├── apps
│   ├── acquire
│   │   ├── DVL_logger.py          # DVL 串口采集长期日志
│   │   ├── imu_logger.py          # IMU 串口采集长期日志
│   │   └── Volt32_logger.py       # 电压电流采集板记录程序
│   ├── __init__.py
│   └── tools
│       ├── dvl_data_verifier.py   # DVL 日志格式 / 内容快速检查
│       ├── imu_baud_switch.py     # 修改 IMU 波特率等设备配置的小工具
│       ├── imu_data_verifier.py   # IMU 日志快速检查
│       ├── imu_xy_zero.py         # 简单的 XY 静态校准工具
│       ├── multisensor_postproc.py# 旧版多传感器对齐 & 可视化脚本
│       ├── tmux_telemetry_manager.py
│       └── volt32_data_verifier.py# 电源数据日志检查
├── config
│   └── devices
│       ├── DVL.yaml               # DVL 串口 / 波特率等配置（Python 采集端使用）
│       └── imu.yaml               # IMU 串口 / 波特率配置
├── data                            # 默认数据输出根目录（按日期再分子目录）
├── docs
│   ├── 传感器读取程序                 # 针对 apps/acquire 系列脚本的说明
│   ├── 命令行                        # 常用命令行片段
│   ├── design
│   │   ├── architecture.md         # 整体软件架构说明
│   │   ├── dataset_spec.md         # 数据集格式与字段说明
│   │   └── eskf_design.md          # 早期 ESKF 设计草案（现已迁移到 nav_core/docs 中）
│   ├── protocols
│   │   ├── HoverH1000_PD6.md       # DVL 协议说明
│   │   └── imu_witmotion_modbus.md # IMU Modbus 协议说明
│   ├── sensors
│   │   ├── dvl_hover_h1000_setup.md
│   │   └── imu_hwt9073_notes.md
│   ├── timebase_spec.md           # 时间戳 / 时基设计（MonoNS / EstNS）
│   └── tools_overview.md          # Python 工具脚本索引
├── git_auto.py
├── git-auto.sh
├── nav_core                        # C++ 在线导航子系统（重点）
│   ├── CMakeLists.txt
│   ├── config
│   │   ├── eskf2d.yaml             # ESKF 参数（2D/2.5D 版）
│   │   └── nav_daemon.yaml         # 导航守护进程配置（串口、日志、频率等）
│   ├── docs
│   │   ├── device_test_and_debug.md
│   │   ├── filters
│   │   │   ├── eskf_design.tex
│   │   │   └── eskf_design.pdf
│   │   ├── imu_dvl_preprocess_and_fusion_guidelines.md
│   │   ├── nav_core_runtime.md
│   │   └── update_guidelines.md
│   ├── include
│   │   └── nav_core
│   │       ├── app
│   │       ├── core
│   │       ├── drivers
│   │       ├── estimator
│   │       ├── filters
│   │       ├── io
│   │       └── preprocess
│   ├── README.md                   # nav_core 子系统的详细说明（已全面更新）
│   ├── src
│   │   └── nav_core
│   │       ├── app                 # nav_daemon main + runner + config
│   │       ├── core                # logging / timebase
│   │       ├── drivers             # IMU / DVL 驱动实现
│   │       ├── estimator           # ESKF 核心 + DVL / yaw 更新
│   │       ├── io                  # 二进制日志 + shm 发布
│   │       └── preprocess          # IMU / DVL 实时预处理
│   └── third_party
│       └── witmotion               # IMU 厂家 C SDK（C 源码）
├── nav_core.zip                    # 早期版本归档（建议只读）
├── pyproject.toml                  # Python 项目配置（可 pip install -e .）
├── README.md                       # 本文件（整体工程说明）
├── requirements.txt                # Python 依赖
├── shared
│   └── msg
│       └── nav_state.hpp           # C++ NavState 共享结构（控制 / 导航共用）
├── tools
│   ├── project_size_report.py
│   └── project_size_report.txt
├── uwnav                           # Python 包：传感器驱动 / IO / 基础融合
│   ├── drivers
│   │   ├── dvl
│   │   ├── imu
│   │   └── __init__.py
│   ├── fusion
│   │   ├── __init__.py
│   │   └── meas_models
│   ├── io
│   │   ├── data_paths.py
│   │   └── timebase.py
│   ├── nav
│   ├── preprocess
│   └── sensors
│       ├── dvl_hover_h1000.py
│       ├── imu.py
│       └── __init__.py
└── uwnav.egg-info                  # Python 安装元数据
````

> 说明：
>
> * 更详细的 C++ 导航子系统结构，请参见 `nav_core/README.md`（已根据最新 ESKF / 预处理代码全面更新）；
> * 更复杂的“离线 ESKF / 因子图平滑 / 轨迹评估”目前放在单独的 `offline_nav` 仓库中，这里只保持最基础的 Python 对齐和可视化脚本。

---

# 3. Python 环境安装与基本用法

建议使用 Anaconda / Miniconda 在 PC 或 Orange Pi 上创建虚拟环境，例如：

```bash
conda create -n uwnav_env python=3.10
conda activate uwnav_env
```

在仓库根目录安装依赖：

```bash
pip install -r requirements.txt
```

（或者开发模式安装）

```bash
pip install -e .
```

安装成功后，`uwnav` 包、`apps/acquire/*` 与 `apps/tools/*` 都可以直接在命令行调用。

---

# 4. C++ 在线导航子系统 `nav_core` 编译与安装

以下内容是对 `nav_core/README.md` 中“编译部分”的简化版，适合新手快速上手，更细节请看子目录 README。

## 4.1 Orange Pi 上安装构建依赖

```bash
sudo apt update
sudo apt install -y git build-essential cmake g++ pkg-config
```

## 4.2 编译 `nav_core`

```bash
cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
mkdir -p build
cd build

cmake ..               # 可根据需要加 -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

编译完成后，一般会得到：

```text
nav_core/build/
├── libnav_core.a
└── bin
    ├── uwnav_navd         # 在线导航守护进程
    └── uwnav_dvl_selftest # DVL 自检工具（CZ/CS 安全测试）
```

> 常见坑：
>
> * 如果编译时报 `shared/msg/nav_state.hpp` 找不到，请检查当前工程是否在 `UnderwaterRobotSystem` 根目录下，目录层级是否与本 README 所示一致；
> * 如果链接时报 IMU 相关 undefined reference，多半是 `third_party/witmotion/wit_c_sdk.c` 没有包含在 CMake 的源文件列表中（请对照 `nav_core/CMakeLists.txt`）。

---

# 5. C++ 导航守护进程运行方式（最新）

当前版本的 `uwnav_navd` 已经**不再通过命令行参数直接指定串口和波特率**，而是通过 YAML 配置文件统一管理。

## 5.1 配置文件路径

`nav_core/config/` 下有两个核心配置：

* `nav_daemon.yaml`：导航守护进程配置，包括：

  * IMU / DVL 串口名（如 `/dev/ttyUSB0`、`/dev/ttyUSB1`）与波特率；
  * IMU / DVL 预处理参数（噪声地板、限幅、离底距离等）；
  * 主循环频率、日志目录、是否启用共享内存发布等。

* `eskf2d.yaml`：ESKF 参数，包括：

  * IMU 噪声、bias 漂移、重力大小；
  * DVL XY / Z 更新开关与噪声协方差；
  * yaw 伪观测的噪声设置；
  * 初始姿态 / 速度 / 协方差等。

这些文件在仓库中已有模板，可根据实际设备情况进行修改。

## 5.2 启动示例（在 Orange Pi 上）

```bash
cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core

./build/bin/uwnav_navd \
  --config      config/nav_daemon.yaml \
  --eskf-config config/eskf2d.yaml
```

导航守护进程将：

1. 打开并配置 IMU & DVL 串口；
2. 启动 IMU / DVL 采集线程；
3. 在主循环中：

   * 对 IMU 帧做实时预处理（去零偏、扣重力等）并驱动 ESKF 传播；
   * 对 DVL 帧做实时预处理（BottomTrack 门控、BI/BE 区分）并驱动 ESKF 更新；
   * 将当前名义状态导出为 `shared::msg::NavState`，写入共享内存；
   * 同时写二进制日志到 `nav_daemon.yaml` 中配置的目录；
4. 在终端周期性打印导航状态概要（位置、深度、yaw、IMU/DVL 在线状态等）。

> 如果程序报错类似：
>
> ```text
> [nav_daemon_config][WARN] cannot open or parse yaml file: config/nav_daemon.yaml : bad file
> ```
>
> 大概率是：
>
> * 当前工作目录不对（程序在 `build` 目录跑，却用相对路径 `config/...`）；
> * YAML 文件有语法错误（缩进错误、tab 混用空格等）。

一般建议在 `nav_core` 根目录下运行，使用 `config/nav_daemon.yaml` 这样的相对路径。

---

# 6. Python 采集与快速检查（PC / Orange Pi 通用）

在实际接入 C++ 导航之前，通常会用 Python 完成传感器“摸底”和简单实验。

## 6.1 IMU 采集：`apps/acquire/imu_logger.py`

典型流程：

1. 确认 `config/devices/imu.yaml` 中串口名和波特率正确；

2. 运行：

   ```bash
   conda activate uwnav_env
   python apps/acquire/imu_logger.py --config config/devices/imu.yaml
   ```

3. 程序会将 IMU 数据按日期写入 `data/YYYY-MM-DD/imu/` 下的 CSV / BIN 文件；

4. 可使用 `apps/tools/imu_data_verifier.py` 快速检查统计特性、噪声水平等。

## 6.2 DVL 采集：`apps/acquire/DVL_logger.py`

注意：DVL 必须在水中运行，且建议始终用安全的 CZ/CS 流程。

1. 设置 `config/devices/DVL.yaml` 中的串口和波特率；

2. 运行：

   ```bash
   python apps/acquire/DVL_logger.py --config config/devices/DVL.yaml
   ```

3. 程序启动时会：

   * 发送 CZ（停机）确保安全；
   * 下水后按配置发送 CS 启动 ping；

4. 日志写入 `data/YYYY-MM-DD/dvl/`；

5. 用 `apps/tools/dvl_data_verifier.py` 检查 PD6 帧解析是否正常、BottomTrack 是否稳定。

## 6.3 电源采集：`apps/acquire/Volt32_logger.py`

* 将 Volt32（或类似电压电流采集卡）接入串口；
* 运行脚本记录电压、电流等数据，以便后续做推力 / 功率相关分析。

---

# 7. Python 后处理：对齐 + 可视化 + 初步分析

对于早期实验，本仓库内的脚本 `apps/tools/multisensor_postproc.py` 可以完成：

* IMU / DVL / 电源日志的基础对齐；
* 输出对齐后的 CSV（例如 `aligned_*.csv`）；
* 自动生成若干简单的可视化图（加速度、角速度、速度、功率随时间变化等）。

典型用法（示例）：

```bash
python apps/tools/multisensor_postproc.py \
  --imu-log   data/2026-01-10/imu/xxx.csv \
  --dvl-log   data/2026-01-10/dvl/xxx.csv \
  --volt-log  data/2026-01-10/volt32/xxx.csv \
  --out-dir   out/2026-01-10_multisensor/
```

当前更高阶的离线导航（包括：

* 用 Python ESKF 复现实验轨迹；
* 建因子图做平滑与轨迹重建；
* 为深度学习准备特征对齐的训练集；

已迁移到独立仓库 `offline_nav`，这里不展开，避免内容混淆。

---

# 8. 当前工程进度概览（2026 年初）

| 模块                          | 状态          | 说明                                              |
| --------------------------- | ----------- | ----------------------------------------------- |
| IMU Python 采集（apps/acquire） | 已投入实测使用     | 支持长期运行，配套有基础检验脚本与笔记文档                           |
| DVL Python 采集（apps/acquire） | 已投入实测使用     | 支持安全 CZ/CS 流程，有 Hover H1000 专用协议解析              |
| Volt32 电源采集                 | 已投入使用       | 可用来构建推力-功率实验数据集                                 |
| C++ IMU 驱动（nav_core）        | 稳定          | 基于 WitMotion C SDK，支持 230400 波特率，100Hz 采样       |
| C++ DVL 驱动（nav_core）        | 稳定          | 支持 PD6/EPD6 解析，区分 BI/BE/BS/BD                   |
| IMU 实时预处理（nav_core）         | 工程版已上线      | 完成 bias 估计、扣重力、deadzone/低通，输出线加速度               |
| DVL 实时预处理（nav_core）         | 工程版已上线      | BI/BE 区分 + BottomTrack 门控 + 噪声地板 + 离底距离判定       |
| 在线 ESKF（nav_core）           | 2D/2.5D 版可用 | ENU 平面轨迹 + 垂向速度约束，已用于水池实验                       |
| NavState 共享内存发布             | 已实现、待联调控制端  | 可被控制程序/上位机直接读取                                  |
| 二进制日志 & Python 离线重现         | 已打通基础链路     | nav_core → bin_log → offline_nav/Python 重算      |
| 离线因子图平滑（独立 offline_nav 仓库）  | 初步版本可用      | 支持 2D 图优化与 ESKF 结果对比                            |
| 深度学习动力学辨识（独立仓库）             | 正在推进        | 利用对齐后的 PWM/IMU/DVL/Volt32 数据训练 LSTM / Hybrid 模型 |

简而言之：

* “**传感器 → Python 采集 → C++ 在线导航 → 日志**”这条链路已经贯通，并在水池实验中运行过多次；
* 更进一步的“离线平滑 / 深度学习 / 控制闭环”正在独立仓库中迭代，这里只提供基础的数据入口。

---

# 9. 常见问题（FAQ，更新版）

### Q1：C++ 工程编译时报找不到 `shared/msg/nav_state.hpp`？

**原因：**

* 当前 `nav_core` 的 CMake 假定工程根目录为 `UnderwaterRobotSystem`，且其下有 `shared/msg/nav_state.hpp`；
* 如果你单独拎出 `nav_core` 目录或改了上层目录结构，就会导致 include path 不正确。

**解决：**

* 按本 README 的层级整理工程目录；
* 或者修改 `nav_core/CMakeLists.txt` 中的根路径（`UNDERWATER_ROOT`），指向实际的 `shared/` 所在位置。

---

### Q2：`uwnav_navd` 报错 “cannot open or parse yaml file: config/nav_daemon.yaml : bad file”？

**原因：**

* 当前工作目录不对；
* YAML 文件缩进 / 语法有问题。

**解决：**

1. 确认在 `nav_core` 根目录运行：

   ```bash
   cd UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
   ./build/bin/uwnav_navd --config config/nav_daemon.yaml --eskf-config config/eskf2d.yaml
   ```

2. 用 `yamllint` 或在线 YAML 校验工具检查 `nav_daemon.yaml` 语法。

---

### Q3：DVL 解析不到 BottomTrack 数据，或者 `kReject_Quality` 很多？

**常见原因：**

* DVL 不在水里 / 下水深度不足；
* 航速 / 姿态使得波束反射效果差；
* PD6 配置不正确或波特率错误。

**建议排查顺序：**

1. 用最简单的串口方式查看原始数据：

   ```bash
   sudo cat /dev/ttyUSB1
   ```

2. 确认可以看到 PD6 帧；

3. 用 `apps/acquire/DVL_logger.py` 先采一段数据，再用 `dvl_data_verifier.py` 看看底跟踪比例和相关系数；

4. 若原始数据就不稳定，优先从机械安装 / 声学条件排查，而不是改代码。

---

### Q4：ESKF 没有明显位移，轨迹几乎停在原点？

典型情况：

* IMU 预处理阶段检测为“长期静止”，线加速度被强烈抑制；
* DVL 数据门控后基本被拒绝（`gated_ok=false`），导致没有有效速度观测；
* ESKF 没有足够的激励。

排查建议：

1. 在 nav_core 的 bin 日志中，先只看 IMU：

   * 检查线加速度分布是否合理，是否全部被 deadzone 压成 0；
2. 再看 DVL：

   * 确认 `DvlRtOutput` 中 `has_bi` / `has_be` 的比例，`reason_code` 是否总是非 0；
3. 确认 ESKF 的 `eskf2d.yaml` 中，确实启用了 DVL XY/Z 更新，并且噪声协方差不是极端大。

---

### Q5：Python 侧脚本找不到 `uwnav` 包？

**原因：**

* 没有在当前虚拟环境中用 `pip install -e .` 安装本项目；
* 或者在不同目录直接用 `python xxx.py` 运行脚本，Python 的 `sys.path` 没有指向项目根目录。

**解决：**

* 激活虚拟环境后，在项目 root 运行一次：

  ```bash
  pip install -e .
  ```

* 之后在任意目录运行 `python -m uwnav.xxx` / `python apps/acquire/imu_logger.py` 均可。

---

# 10. 贡献与开发建议

如果你准备在本仓库上继续开发，推荐的方式是：

1. **先读文档：**

   * `docs/design/architecture.md` —— 整体结构；
   * `nav_core/README.md` —— 在线导航 C++ 子系统说明；
   * `docs/timebase_spec.md` —— 时间同步与时基；
   * `docs/sensors/*` & `docs/protocols/*` —— 硬件层细节。

2. **再跑通一条完整链路：**

   * 在 PC 上，用 Python 采一小段 IMU / DVL 数据并画图；
   * 在 Orange Pi 上，跑起 `uwnav_navd`，看终端导航日志是否正常；
   * 把 C++ 导航的 bin 日志拷回 PC，在 `offline_nav` 仓库中做一次离线重放。

3. **最后再改代码：**

   * 新增传感器 → 优先写 Python 采集和文档，再接入 C++；
   * 改 ESKF → 同时修改 `eskf2d.yaml` 和 `nav_core/estimator/*`；
   * 改数据结构 → 确保 drivers / preprocess / estimator / io / Python 工具这条链路上的所有结构定义都同步更新。

本 README 只负责给你一个“全局地图”。
具体到 nav_core 的实现细节、ESKF 数学推导、IMU/DVL 预处理规范，请以 `nav_core/README.md` 和 `nav_core/docs/*` 为准。

```

```
