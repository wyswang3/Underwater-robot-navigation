# 本地改动说明（2026-03-12）

## 仓库

- 路径：`UnderwaterRobotSystem/Underwater-robot-navigation`
- 本轮功能提交：`f6ff98f 收紧导航状态发布语义并补充状态机测试`

## 本轮改动目标

本仓库本轮聚焦解决导航主循环的 P0 问题：

- 未初始化时不再发布伪正常导航状态
- IMU 缺失或过期时不再静默输出“看似合法”的数值
- DVL 缺失明确进入 `DEGRADED`
- NavState 发布前增加显式状态门控
- NavState SHM 发布器切换到统一共享契约

## 关键改动

### 1. 新增导航发布状态机辅助模块

新增：

- `nav_core/include/nav_core/app/nav_runtime_status.hpp`
- `nav_core/src/nav_core/app/nav_runtime_status.cpp`

该模块把 `NavState` 发布前的工程语义集中处理，避免 `nav_daemon_runner.cpp` 中散落默认值。

主要职责：

- 判断 ESKF 数值是否有限
- 映射名义状态到 `NavState` 运动学字段
- 统一填充 `valid/stale/degraded/nav_state/fault_code/sensor_mask/status_flags`

### 2. 主循环发布门控收紧

`nav_daemon_runner.cpp` 中原先直接把 ESKF 输出映射成 `health=OK`、`IMU_OK` 的做法已移除，改为根据真实条件驱动状态：

- 无 IMU 或未传播：`UNINITIALIZED`
- 对准未完成：`ALIGNING`
- IMU stale：`INVALID`
- 数值非有限：`INVALID`
- DVL 缺失：`DEGRADED`
- 条件满足后才进入 `OK`

### 3. NavStatePublisher 统一 SHM 契约

发布端不再使用私有 header/layout，而是统一复用 `shared/shm/nav_state_shm.hpp`。

这项修改用于消除：

- 发布端和 gateway 读取端布局不一致
- `magic/version/payload_size/payload_align` 漂移

### 4. 测试接入

`nav_core/CMakeLists.txt` 增加 `NAV_CORE_BUILD_TESTS` 选项，并接入：

- `test_nav_runtime_status`

测试覆盖了以下最小 P0 场景：

- 冷启动未初始化不再输出伪正常状态
- 对准未完成不再 `valid=1`
- IMU stale 进入 `INVALID`
- DVL 缺失进入 `DEGRADED`

## 本轮编译与测试

已验证：

- 编译：`uwnav_navd`
- 测试：`nav_core_test_nav_runtime_status`

结果：通过

## 当前残余

- 仓库内仍残留 `nav_core/build/` 构建产物和 CTest 临时文件，未纳入本轮提交。
- IMU 串口稳定路径、重枚举识别、断连重连还未进入本仓库整改。
- `nav_daemon_runner.cpp` 仍偏长，后续建议继续拆分 `init/wiring` 与 `loop/publish`。

## 远程提交前建议

- 先确认下游 `shared` 协议版本与当前 NavState 语义一致。
- 再连同 gateway / 控制仓库一起审核，确保 `valid/stale/fault` 语义是端到端闭合的。
