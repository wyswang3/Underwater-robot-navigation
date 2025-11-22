
---

# tools_overview.md

**Underwater Robot Navigation Project – Tools Layer Overview**
**（工具层总体说明文档 · 正式版）**

---

## 1. 工具层简介

`apps/tools/` 目录包含本项目的**基础数据采集工具**，用于在实验环境中稳定、可靠地获取原始传感器数据（IMU / 电压&电流板 / DVL 等），为导航定位模块、动力学建模模块、控制系统模块提供高质量输入。

工具层的核心目标包括：

* 稳定采集与记录传感器数据
* 提供实时调试、异常检测、滤波、文件轮换能力
* 实验阶段快速验证硬件与通信
* 构建可追踪、可复现实验的数据生产流水线
* 作为 uwnav/ 内部导航模块的基础输入层

此目录的脚本不直接参与导航或滤波计算，它们专注于数据采集和工程实用性。

---

## 2. 目录结构

```
apps/tools/
│
├── imu_realtime_pipeline.py      # 高速 IMU 实时采集 + 滤波管线
├── volt32_logger.py              # 多通道电压/电流采集 + Rolling CSV
└── tmux_telemetry_manager.py     # 一键 tmux 启动多路传感器采集管理器
```

未来拟加入：

* `dvl_data_verifier.py`（Hover H1000 DVL：统一时间基记录器）

---

## 3. 工具脚本详细说明

---

### 3.1 IMU Realtime Pipeline

**文件：`imu_realtime_pipeline.py`**
**功能：IMU 高速实时采集 + 滑动窗口滤波 + 双 CSV 输出**

适用设备：
WitMotion HWT9073-485 / WT901C-485 等 Modbus IMU（经 485→USB 转换）

**核心能力：**

* 以 230400 波特率进行高频采集
* 回调中抽取加速度/角速度/欧拉角/磁场
* 数据以结构化 dict 推入队列，由消费者线程处理
* 写入两类 CSV 文件：

  1. 原始数据（raw）
  2. 滤波后数据（filtered）

**工程特性：**

* 自动调试输出；空闲超时报警
* 滑动窗口实时滤波（RealTimeIMUFilter）
* 定期 flush 写盘
* 优雅退出机制（signal handler）
* 可用于：

  * IMU 通信调试
  * 滤波器研究
  * 采集深度学习训练用数据集

输出示例：

```
timestamp, Ax, Ay, Az, Gx, Gy, Gz, HX, HY, HZ, Roll, Pitch, Yaw
...
```

---

### 3.2 Volt32 Logger

**文件：`volt32_logger.py`**
**功能：电压/电流采集卡日志器（多通道 + 文件轮换）**

适用设备：
STM32 + 16/32 路 ADC 数据采集板

**核心能力：**

* 从串口读取一行包含多通道电压/电流的 ASCII 数据
* 支持 N=16 或未来扩展 N=32 通道
* 使用 RollingCSVWriter 自动处理：

  * 按大小滚动（默认 50MB）
  * 按天滚动（跨天自动新文件）
  * 自动清理超过 N 天（默认 7 天）旧日志

**工程特性：**

* 容错解析（空格/Tab 混合）
* 后台周期性刷盘
* 可视化调试输出
* 支持交互式键盘命令退出
* 专为长时间记录（数小时–数天）设计

输出示例：

```
Timestamp, CH0, CH1, ..., CH15
...
```

适用场景：

* 推进器功率–推力实验
* 电源系统健康监测
* 动力学建模的功率输入记录

---

### 3.3 tmux Telemetry Manager

**文件：`tmux_telemetry_manager.py`**
**功能：多传感器采集脚本管理器（tmux session orchestration）**

**作用：**
为 IMU 与电压记录器提供“一键启动、多窗口同步运行”的能力。

**核心能力：**

* 创建 tmux 会话
* 自动左右分屏：

  * pane 1：IMU 采集
  * pane 2：Volt32 Logger
* 智能解析脚本路径（支持 apps/ 下执行）
* 自动串口扫描（`--list-ports`）
* 可选择 attach 显示界面或后台运行
* 支持扩展：

  * 添加 DVL Pane
  * 添加 USBL Pane

典型启动命令：

```bash
python3 tmux_telemetry_manager.py \
  --imu-script imu_realtime_pipeline.py --imu-port /dev/ttyUSB1 --imu-baud 230400 \
  --volt-script volt32_logger.py        --volt-port /dev/ttyUSB0 --volt-baud 115200
```

极大提升实验效率，是现场调试、池试的必备工具。

---

## 4. 工具层与整体系统架构关系

```
                  +------------------------------+
                  |          apps/tools/         |
                  |  (数据采集 / 调试 / 校验层)  |
                  +---------------+--------------+
                                  |
                        传感器原始数据 logs/
                                  |
                                  v
        +----------------------------------------------+
        |                   data/                      |
        |   imu/   volt/   dvl/   usbl/   ...         |
        | (按传感器类型分类保存原始 CSV / TB 数据)     |
        +--------------------+-------------------------+
                             |
                             | 输入数据流
                             v
        +----------------------------------------------+
        |                    uwnav/                    |
        |    (驱动层 + 时间基 + 数据同步 + ESKF)      |
        |     IMU → ESKF ← DVL / USBL / 深度传感器     |
        +--------------------+-------------------------+
                             |
                             | 位姿/速度/状态
                             v
        +----------------------------------------------+
        |          控制层 / MPC / RL / 轨迹生成         |
        +----------------------------------------------+
```

---

## 5. 数据分类保存（下一版将启用）

当前工具脚本默认把数据写到 `data/` 下。
为未来导航系统的多传感器融合，建议采用如下目录结构：

```
data/
  yyyy-mm-dd/
    imu/
    dvl/
    volt/
    usbl/
    camera/
```

之后你确认目录格式后，我将：

* 更新所有工具脚本的默认输出路径
* 增加自动创建按日期划分的多层目录
* 更新 tmux_telemetry_manager，自动写入对应 sensor 子目录
* 生成新的 `tools_overview.md（v2）`

---

## 6. 工程建议（正式版）

1. **RollingCSVWriter 可抽象为公共 I/O 工具**
   放入 `uwnav.io.logging`，减少重复代码。

2. **加入统一时间基（timebase.stamp）改造 IMU 与 Volt 工具**
   现在 DVL 已支持，IMU 与 Volt 也建议对齐。

3. **增加一个 multi-sensor 一键三路采集 manager（IMU+DVL+Volt）**
   未来数据更加一致。

4. **所有工具脚本建议在 `--outdir` 中自动选择按日期/传感器分类的目录**
   方便后续融合/训练。

---

