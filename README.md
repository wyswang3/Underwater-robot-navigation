# Underwater-robot-navigation

`Underwater-robot-navigation` 是当前项目的导航 runtime、传感器协议、时序语义和最小 replay/diagnostics 仓库。

对开发者来说，这个仓最重要的是回答四个问题：

1. 传感器数据怎样进入导航 runtime
2. `NavState` 怎样形成并发布
3. 时间/有效性/重连语义怎样落地
4. 事故窗口怎样被导出、复盘和对照

## 1. Technical Scope

This repo currently owns:

- 采集 IMU、DVL、电源等实验数据
- 在板端运行在线导航守护进程 `uwnav_navd`
- 输出权威 `NavState`、`nav_timing.bin` 和 `nav_state.bin`
- 提供事故窗口导出、incident bundle 和最小 replay compare 工具
- 为控制仓提供导航输入与复盘材料

It does not own:

- final motor control
- GCS/UI
- final PWM execution
- top-level bring-up orchestration

## 2. Main Runtime Pipelines

Online navigation:

```text
IMU / DVL
  -> drivers
  -> preprocess
  -> ESKF
  -> NavState publisher
  -> control side consumer
```

Timing and replay:

```text
nav_timing.bin + nav_state.bin + control CSV + telemetry timeline/events
  -> merge_robot_timeline.py
  -> incident bundle
  -> uwnav_nav_replay
  -> replay_compare.py
```

Device and bench diagnosis:

```text
device binding / reconnect state machine
  + usb_serial_snapshot.py
  + nav_timing.bin
  -> bench diagnosis and incident export
```

## 3. Documentation Policy

This repo's docs should primarily explain technical design and low-level implementation boundaries.

Start with:

1. `docs/文档总览.md`
2. `docs/导航运行总览.md`
3. `nav_core/README.md`
4. `docs/时间融合与接口约定.md`
5. `docs/调试回放与排障.md`
6. `docs/演进记录与维护约束.md`

Operator startup and multi-repo deployment docs should stay in the system doc repo, not become the primary entry here.

## 4. Recommended Reading Order

如果你要先理解“导航仓为什么是现在这条工程路线”，建议按这个顺序读：

1. `docs/文档总览.md`
2. `nav_core/README.md`
3. `docs/导航运行总览.md`
4. `docs/时间融合与接口约定.md`
5. `docs/调试回放与排障.md`

然后再进入具体代码和协议。

## 5. Directory Layout

- `apps/acquire/`
  - 早期或辅助的传感器采集脚本
- `apps/tools/`
  - 数据快速检查、设备配置、小工具
- `config/devices/`
  - 采集脚本层面的设备配置
- `docs/`
  - 导航侧中文基线文档，控制在 6 份以内
- `nav_core/`
  - C++ 在线导航核心，当前最重要的子目录
- `uwnav/`
  - Python 包与轻量工具代码

## 6. Recommended Code Reading Order

For runtime behavior:

1. `nav_core/src/nav_core/app/nav_daemon.cpp`
2. `nav_core/src/nav_core/app/nav_daemon_runner.cpp`
3. `nav_core/drivers/`
4. `nav_core/preprocess/`
5. `nav_core/estimator/`
6. `nav_core/io/`

For replay and diagnosis:

1. `nav_core/tools/merge_robot_timeline.py`
2. `nav_core/tools/replay_compare.py`
3. `nav_core/tools/usb_serial_snapshot.py`
4. `nav_core/tools/parse_nav_timing.py`

## 7. Build `nav_core`

编译 `nav_core`：

```bash
cd <Underwater-robot-navigation repo root>/nav_core
cmake -S . -B build
cmake --build build -j4
```

常用二进制：

- `build/bin/uwnav_navd`
- `build/bin/uwnav_dvl_selftest`
- `build/bin/uwnav_nav_replay`

## 8. Run `uwnav_navd`

Run the online daemon:

```bash
./build/bin/uwnav_navd \
  --config ./config/nav_daemon.yaml \
  --eskf-config ./config/eskf.yaml
```

## 9. Most Important Runtime Artifacts

运行时产物：

- `nav_timing.bin`
- `nav_state.bin`
- `NavState` SHM

Replay / diagnosis tools:

- `nav_core/tools/merge_robot_timeline.py`
- `nav_core/tools/replay_compare.py`
- `nav_core/tools/usb_serial_snapshot.py`
- `nav_core/tools/parse_nav_timing.py`

## 10. Engineering Boundaries

这个仓库负责导航，不负责：

- 推进器控制
- GCS UI
- 最终硬件 PWM 输出
- 顶层统一构建

另外，当前 replay 仍然是“最小可验证方案”，不是完整原始帧级 replay 平台。

## 11. Documentation Cleanup Rule

这个仓会继续保留一些历史实验和阶段复盘，但它们不再作为开发者主入口。

优先保留的文档类型是：

- runtime architecture
- sensor protocol
- timestamp and stale semantics
- replay / compare / incident tooling
- reconnect and health audit

优先降级或删除的文档类型是：

- 只记录一次实验命令的草稿
- 已脱离当前 runtime 的旧采集流程
- 生成型中间文件
- 与当前仓职责无关的杂项命令备忘

## 12. Best Learning Path

不要先从滤波公式开始。更好的顺序是：

1. 先理解 `NavState` 是什么、给谁用
2. 再理解日志和时间语义为什么要分 `sensor/recv/consume/publish`
3. 再去看 `drivers`、`preprocess`、`estimator`
4. 最后再看 replay 和 compare 工具

这样你看到的会是一条完整工程链，而不是一堆数学和串口代码。
