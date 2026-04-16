# Underwater-robot-navigation

`Underwater-robot-navigation` 是当前项目的 navigation runtime / sensor binding / replay diagnostics 仓库。

它主要回答三件事：

1. 传感器数据怎样进入在线导航 runtime
2. `NavState` 怎样形成、发布并提供给控制链
3. 事故窗口怎样被记录、导出、回放和对照

## 当前阶段 / Current Status

这个仓当前已经不是早期“只做 IMU 串口试验”的状态，而是：

- `nav_core` 已形成 C++ 在线导航主线
- 设备绑定、重连、stale、health audit 已进入运行时语义
- `NavState` / `nav_state.bin` / `nav_timing.bin` 已成为控制链和复盘链的关键输入
- replay / compare / incident tooling 已形成最小闭环

当前仍在持续推进的重点是：

- reconnect / bench 负路径收口
- replay 证据质量和 compare 自动化
- 更稳定的设备识别与健康诊断

## 仓库用途 / What This Repo Is For

当前仓库负责：

- IMU / DVL 等传感器接入与协议适配
- 在线导航守护进程 `uwnav_navd`
- 时间语义、有效性语义、健康语义
- 导航日志、incident bundle、最小 replay compare

当前仓库不负责：

- final motor control
- GCS / GUI
- final PWM execution
- 顶层 operator bring-up orchestration

## 当前主链路

在线导航：

```text
IMU / DVL
  -> drivers
  -> preprocess
  -> estimator
  -> NavState publish
  -> control side consumer
```

复盘与诊断：

```text
nav_timing.bin + nav_state.bin + control CSV + telemetry timeline/events
  -> incident bundle
  -> uwnav_nav_replay
  -> replay_compare.py
```

设备与 bench 排障：

```text
device binding / reconnect state machine
  + usb_serial_snapshot.py
  + nav_timing.bin
  -> diagnosis and incident export
```

## 当前技术思路 / Current Engineering Thinking

这个仓的路线不是“做一个纯算法目录”，而是：

- runtime first
- time semantics first
- contract and replay first
- sensor protocol and reconnect semantics 必须落到真实运行链

换句话说，这里更强调：

- 状态传播可信
- 时间解释一致
- 故障能复盘

而不是只强调滤波公式本身。

## 目录结构 / Repository Layout

- `docs/`
  - 导航侧中文基线文档
- `nav_core/`
  - C++ 在线导航核心
- `uwnav/`
  - Python 轻量工具和驱动辅助代码
- `apps/`
  - 采集与小工具
- `config/`
  - 设备与采集配置

## 推荐阅读顺序

先读文档：

1. `docs/文档总览.md`
2. `docs/导航运行总览.md`
3. `nav_core/README.md`
4. `docs/时间融合与接口约定.md`
5. `docs/调试回放与排障.md`

再读代码：

1. `nav_core/src/nav_core/app/nav_daemon.cpp`
2. `nav_core/src/nav_core/app/nav_daemon_runner.cpp`
3. `nav_core/drivers/`
4. `nav_core/preprocess/`
5. `nav_core/estimator/`
6. `nav_core/io/`
7. `nav_core/tools/`

## Build `nav_core`

```bash
cd <Underwater-robot-navigation repo root>/nav_core
cmake -S . -B build
cmake --build build -j4
```

常用目标：

- `build/bin/uwnav_navd`
- `build/bin/uwnav_dvl_selftest`
- `build/bin/uwnav_nav_replay`
- `build/bin/uwnav_imu_modbus_probe`

## 当前边界 / Current Boundaries

这个仓当前最重要的边界是：

- 输出权威导航状态，但不做最终控制裁决
- 保留 replay / diagnostics 能力，但还不是完整原始帧级 replay 平台
- 可以扩展外围工具，但不能把核心 nav runtime 迁成 Python / ROS2 authority

## 文档入口

当前最直接的导航侧说明在：

- `docs/文档总览.md`
- `docs/导航运行总览.md`
- `docs/时间融合与接口约定.md`
- `docs/调试回放与排障.md`
- `docs/演进记录与维护约束.md`
