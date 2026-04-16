# nav_core

`nav_core` 是当前项目的 online navigation runtime core。

它运行在 OrangePi 等板端设备上，负责把真实串口侧的 IMU / DVL 输入转成控制链可消费的 `NavState`，并同时输出支撑 replay / diagnostics 的日志。

## 当前进度 / Current Status

当前 `nav_core` 已经完成的关键收口包括：

- `uwnav_navd` 成为当前在线导航主入口
- `NavState` 输出、`nav_state.bin`、`nav_timing.bin` 已形成稳定运行链
- 设备绑定、重连、stale 和 health audit 已进入实际 runtime
- 最小 replay 注入与 compare 工具已经能支撑事故窗口复盘

当前还没有要声称完成的部分：

- 原始 IMU / DVL frame-level replay
- 顶层统一构建平台
- 完整任务层或全自动导航工作站

## 模块用途 / What `nav_core` Owns

`nav_core` 负责：

- 实时驱动 IMU / DVL
- 做预处理和融合
- 生成权威导航状态
- 输出控制链可消费的 SHM
- 保留足够日志支撑 replay / compare

它不负责：

- 控制器
- GUI
- GCS
- final PWM execution

## 当前运行链路

```text
IMU / DVL serial input
  -> drivers
  -> preprocess
  -> estimator
  -> shared::msg::NavState
  -> nav_state.bin / nav_timing.bin / SHM publish
```

## 当前工程思路 / Current Engineering Thinking

`nav_core` 的设计重点不是“把滤波器写出来”而已，而是：

- 让时间语义可解释
- 让状态传播能被控制链安全消费
- 让设备异常和 replay 证据保留下来

所以这里的关键不是单个算法函数，而是：

- `drivers`
- `preprocess`
- `estimator`
- `io`
- `tools`

如何共同形成一条可运行、可诊断、可复盘的链路。

## 目录结构

- `config/`
  - `nav_daemon.yaml`、`eskf.yaml`
- `include/nav_core/`
  - app / core / drivers / preprocess / estimator / io
- `src/nav_core/`
  - 各模块实现
- `tools/`
  - incident bundle、timeline merge、replay compare、timing parse
- `tests/`
  - C++ / Python 最小回归

## 推荐阅读顺序

如果你是运行时开发者：

1. `src/nav_core/app/nav_daemon.cpp`
2. `src/nav_core/app/nav_daemon_runner.cpp`
3. `drivers/`
4. `preprocess/`
5. `estimator/`
6. `io/`
7. `tools/`

如果你是代码学习者：

1. 先看主循环
2. 再看 `NavState` 输出语义
3. 再看 timing log 和 replay tool
4. 最后再看滤波器细节

## Build

```bash
cd <Underwater-robot-navigation repo root>/nav_core
cmake -S . -B build
cmake --build build -j4
```

常用目标：

- `uwnav_navd`
- `uwnav_dvl_selftest`
- `uwnav_nav_replay`
- `uwnav_imu_modbus_probe`

## 常用运行方式

在线导航：

```bash
./build/bin/uwnav_navd \
  --config ./config/nav_daemon.yaml \
  --eskf-config ./config/eskf.yaml
```

最小 replay：

```bash
./build/bin/uwnav_nav_replay <incident_bundle_dir>
```

## 文档入口

更系统的导航侧说明在：

- `../docs/文档总览.md`
- `../docs/导航运行总览.md`
- `../docs/时间融合与接口约定.md`
- `../docs/调试回放与排障.md`
