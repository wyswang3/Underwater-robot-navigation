# nav_core

`nav_core` 是当前项目的在线导航运行时核心，使用 C++ 实现，运行在香橙派等板端设备上。

它负责把串口侧的 IMU / DVL 数据，变成控制链可消费的 `NavState`，并同时生成复盘所需的时间与状态日志。

补充入口：

- 仓级文档总览：`../docs/文档总览.md`
- 导航运行总览：`../docs/导航运行总览.md`
- 时间/融合/接口约定：`../docs/时间融合与接口约定.md`
- 调试、回放与排障：`../docs/调试回放与排障.md`

## 1. 子系统定位

`nav_core` 的核心职责是：

- 驱动 IMU / DVL 并做设备级容错
- 在实时链路上做预处理和融合
- 产生权威导航状态
- 输出给控制链的 SHM
- 同时保留足够的日志，支撑最小 replay 与 incident bundle

它不是：

- 控制器
- GUI
- 完整离线后端
- 顶层调度系统

## 2. 当前代码结构

- `config/`
  - `nav_daemon.yaml`：运行时设备、日志、发布配置
  - `eskf.yaml`：滤波器参数
- `include/nav_core/`
  - `app/`：配置与 runner 接口
  - `core/`：时间、日志、基础类型
  - `drivers/`：IMU / DVL 驱动
  - `preprocess/`：实时预处理
  - `estimator/`：ESKF 与相关逻辑
  - `io/`：bin log、SHM 发布、replay 接口
- `src/nav_core/`
  - 对应各模块实现
- `tools/`
  - 时间线合并、incident bundle、replay compare、USB 快照
- `tests/`
  - C++ 与 Python 的最小回归

## 3. 运行时主线

当前在线运行链路可以概括为：

```text
drivers
  -> preprocess
  -> estimator
  -> NavState publisher
  -> logs + SHM
```

更细一点：

```text
IMU / DVL serial input
  -> ImuDriverWit / DvlDriver
  -> imu_rt_preprocessor / dvl_rt_preprocessor
  -> ESKF propagate + update
  -> shared::msg::NavState
  -> nav_state.bin / nav_timing.bin / SHM publish
```

## 4. 当前可用二进制

编译后常用目标位于 `build/bin/`：

- `uwnav_navd`
  - 在线导航守护进程
- `uwnav_dvl_selftest`
  - DVL 自检工具
- `uwnav_nav_replay`
  - 最小 NavState replay 注入器
- `uwnav_imu_modbus_probe`
  - 对指定串口主动发送 WIT Modbus 请求，并打印原始返回

## 5. 构建方式

```bash
cd <Underwater-robot-navigation repo root>/nav_core
cmake -S . -B build
cmake --build build -j4
```

如果你只是做开发阅读，先看 `build/bin` 里是否已经有现成目标，再决定要不要重编译。

## 6. 常用运行方式

启动在线导航：

```bash
./build/bin/uwnav_navd \
  --config ./config/nav_daemon.yaml \
  --eskf-config ./config/eskf.yaml
```

最小 replay 注入：

```bash
./build/bin/uwnav_nav_replay <incident_bundle_dir>
```

说明：

- `uwnav_nav_replay` 的输入重点是 `nav_state_window.bin`
- 目标是复现状态传播，而不是精确重建原始传感器帧节奏

## 7. 关键运行产物

当前最重要的运行产物有三类。

状态与时间日志：

- `nav_timing.bin`
- `nav_state.bin`

控制可消费接口：

- `NavState` SHM

事故与复盘工具：

- incident bundle
- replay compare 报告

## 8. 工具链说明

`tools/` 目录是当前最实用的一组工程工具：

- `merge_robot_timeline.py`
  - 合并 nav / control / telemetry 事件并导出事故包
- `replay_compare.py`
  - 对照原始事故窗口与 replay 结果
- `usb_serial_snapshot.py`
  - 记录真实串口与 by-id 节点变化
- `parse_nav_timing.py`
  - 解析 `nav_timing.bin`
- `uwnav_imu_modbus_probe`
  - 直接对 IMU 候选串口发 `0x50 0x03 0x0034 0x000f` 读请求，输出 TX/RX 十六进制

这里的重点不是“做大平台”，而是把最小复盘闭环做实。

## 9. 推荐阅读顺序

如果你是开发者：

1. `src/nav_core/app/nav_daemon.cpp`
2. `src/nav_core/app/nav_daemon_runner.cpp`
3. `drivers/`
4. `preprocess/`
5. `estimator/`
6. `io/`
7. `tools/` 和 `tests/`

如果你是代码学习者：

1. 先看 `nav_daemon_runner.cpp`，理解主循环
2. 再看 `shared::msg::NavState` 的输出语义
3. 再看时间日志和 replay 工具
4. 最后再看滤波器细节

## 10. 当前已验证的工程边界

目前这套 `nav_core` 已经覆盖：

- 时间语义收口
- 设备绑定和重连状态机
- daemon stale 行为验证
- incident bundle 导出
- 最小 replay 注入
- replay compare

但还没有覆盖：

- 原始 IMU / DVL 帧级 replay
- 完整命令链重放
- 顶层统一构建

## 11. 测试与学习建议

对这个子系统最有价值的学习方式，不是只读公式，而是同时看：

- 一个运行命令
- 一份日志产物
- 一个 SHM 输出
- 一个 replay / compare 工具

这样你会更快明白为什么 `nav_core` 长成现在这样，也更容易理解代码里的“时间语义”和“状态传播语义”。
