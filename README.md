# Underwater-robot-navigation

`Underwater-robot-navigation` 是当前项目里的导航与传感器仓。

它同时覆盖三类能力：

- 传感器采集与基础检查
- `nav_core` 在线导航运行时
- 日志、incident bundle、replay compare 等最小复盘工具

如果你想理解“IMU/DVL 数据怎么进入系统、怎么变成控制可消费的导航状态、事故窗口怎么导出和重放”，应该从这个仓开始。

## 1. 这个仓库现在负责什么

当前仓库的主职责包括：

- 采集 IMU、DVL、电源等实验数据
- 在板端运行在线导航守护进程 `uwnav_navd`
- 输出权威 `NavState`、`nav_timing.bin` 和 `nav_state.bin`
- 提供事故窗口导出、incident bundle 和最小 replay compare 工具
- 为控制仓提供导航输入与复盘材料

## 2. 当前已经验证可用的主线

在线导航主线：

```text
IMU / DVL
  -> drivers
  -> preprocess
  -> ESKF
  -> NavState publisher
  -> control side consumer
```

时间与复盘主线：

```text
nav_timing.bin + nav_state.bin + control CSV + telemetry timeline/events
  -> merge_robot_timeline.py
  -> incident bundle
  -> uwnav_nav_replay
  -> replay_compare.py
```

设备与台架主线：

```text
device binding / reconnect state machine
  + usb_serial_snapshot.py
  + nav_timing.bin
  -> bench diagnosis and incident export
```

## 3. 目录说明

- `apps/acquire/`
  - 早期或辅助的传感器采集脚本
- `apps/tools/`
  - 数据快速检查、设备配置、小工具
- `config/devices/`
  - 采集脚本层面的设备配置
- `docs/`
  - 传感器协议、时间语义、工具说明
- `nav_core/`
  - C++ 在线导航核心，当前最重要的子目录
- `uwnav/`
  - Python 包与轻量工具代码

## 4. 建议阅读顺序

对开发者：

1. 本 README
2. [nav_core/README.md](/home/wys/orangepi/UnderwaterRobotSystem/Underwater-robot-navigation/nav_core/README.md)
3. `docs/timebase_spec.md`
4. `docs/protocols/` 下的 IMU/DVL 协议文档
5. `nav_core/tools/` 和 `nav_core/tests/`

对代码学习者：

1. 先看 `nav_core` 的运行链路
2. 再看 `drivers -> preprocess -> estimator -> io` 的代码结构
3. 最后再看 Python 采集脚本和离线检查工具

## 5. 快速开始

编译 `nav_core`：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
cmake -S . -B build
cmake --build build -j4
```

常用二进制：

- `build/bin/uwnav_navd`
- `build/bin/uwnav_dvl_selftest`
- `build/bin/uwnav_nav_replay`

运行 `uwnav_navd`：

```bash
./build/bin/uwnav_navd \
  --config /home/wys/orangepi/UnderwaterRobotSystem/Underwater-robot-navigation/nav_core/config/nav_daemon.yaml \
  --eskf-config /home/wys/orangepi/UnderwaterRobotSystem/Underwater-robot-navigation/nav_core/config/eskf.yaml
```

## 6. 当前最重要的产物和工具

运行时产物：

- `nav_timing.bin`
- `nav_state.bin`
- `NavState` SHM

复盘工具：

- `nav_core/tools/merge_robot_timeline.py`
- `nav_core/tools/replay_compare.py`
- `nav_core/tools/usb_serial_snapshot.py`
- `nav_core/tools/parse_nav_timing.py`

## 7. 当前工程边界

这个仓库负责导航，不负责：

- 推进器控制
- GCS UI
- 最终硬件 PWM 输出
- 顶层统一构建

另外，当前 replay 仍然是“最小可验证方案”，不是完整原始帧级 replay 平台。

## 8. 对学习者最有价值的切入点

如果你是第一次学这个仓库，不要先从滤波公式开始。

更推荐的顺序是：

1. 先理解 `NavState` 是什么、给谁用
2. 再理解日志和时间语义为什么要分 `sensor/recv/consume/publish`
3. 再去看 `drivers`、`preprocess`、`estimator`
4. 最后再看 replay 和 compare 工具

这样你看到的会是一条完整工程链，而不是一堆数学和串口代码。
