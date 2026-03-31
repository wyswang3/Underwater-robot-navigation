---
# 上电测试与故障排查指南

**Underwater-robot-navigation 项目**
适用分支：`feature/IMU-DVL-serial`
目标平台：Orange Pi / Linux SBC
版本：v1.0

---

## 目录

1. [文档目的](#文档目的)
2. [系统组成与依赖关系](#系统组成与依赖关系)
3. [上电前检查清单](#上电前检查清单)
4. [上电顺序（必须遵守）](#上电顺序必须遵守)
5. [硬件识别与端口检查](#硬件识别与端口检查)
6. [IMU 单独测试流程](#imu-单独测试流程)
7. [DVL 单独测试流程（Hover H1000）](#dvl-单独测试流程hover-h1000)
8. [全链路测试（IMU + DVL + ESKF）](#全链路测试imu--dvl--eskf)
9. [日志格式与验证方法](#日志格式与验证方法)
10. [常见故障与排查](#常见故障与排查)
11. [安全注意事项](#安全注意事项)
12. [建议的实验顺序](#建议的实验顺序)

---

# 1. 文档目的

本指南用于指导开发人员和测试人员在**陆地/水池环境**下验证导航系统是否正常运行。包括：

* 传感器供电/通信是否正常
* IMU（HWT9073-485）是否以 100Hz 正常输出
* DVL（Hover H1000）是否能够正确解析 PD6 数据
* C++ nav_core 的实时导航链路是否正常运行
* 日志（IMU / DVL / ESKF）是否正确生成

本指南覆盖 **上电 → 单传感器测试 → 全链路验证 → 故障排查** 全流程。

---

# 2. 系统组成与依赖关系

```
IMU (HWT9073-485 / Modbus RTU 230400)
       ↓  
       → imu_driver_wit.cpp
              ↓ 100Hz
             ESKF ← DVL (Hover H1000 / PD6 115200)
                     ↑ 10Hz
nav_daemon.cpp —— 整合实时导航
       ↓  
bin_logger —— imu.bin / dvl.bin / eskf.bin
```

导航系统依赖两个关键传感器：

| 传感器             | 通信                    | 频率      | 驱动文件                 |
| --------------- | --------------------- | ------- | -------------------- |
| IMU HWT9073-485 | RS485 / Modbus 230400 | 100 Hz  | `imu_driver_wit.cpp` |
| DVL Hover H1000 | 串口 115200             | 5–10 Hz | `dvl_driver.cpp`     |

---

# 3. 上电前检查清单

使用本指南前请检查：

### 3.1 电气检查

* DVL 是否接入 **24V 稳定供电**
* IMU RS485 A/B 是否接对（灰/白线确认）
* Orange Pi USB 接口是否正常

### 3.2 软件检查

```bash
git status
git branch
```

确保正在运行：

```
feature/IMU-DVL-serial
```

### 3.3 串口权限

若非 root，执行：

```bash
sudo usermod -a -G dialout $USER
```

---

# 4. 上电顺序（必须遵守）

为避免损坏 DVL，必须遵循以下顺序：

1. 确保 **DVL 完全浸没在水中**
2. 打开 DVL 电源（24V）
3. 插入 IMU → Orange Pi（USB/RS485）
4. 启动 Orange Pi
5. 最后插入 DVL USB → Orange Pi
   （防止 DVL 在空气中干转）

若必须空中测试，**持续时间必须 < 2 秒**。

---

# 5. 硬件识别与端口检查

上电后检查设备是否被识别：

```bash
ls /dev/ttyUSB*
```

若识别成功，应看到：

```
/dev/ttyUSB0
/dev/ttyUSB1
```

确认具体映射：

```bash
dmesg | grep ttyUSB
```

输出示例：

```
FTDI converter now attached to ttyUSB0     <-- 多为 IMU
CP210x converter now attached to ttyUSB1   <-- DVL
```

---

# 6. IMU 单独测试流程

### 6.1 启动 IMU 驱动

```bash
cd ~/Underwater-robot-navigation/nav_core/build

sudo ./uwnav_navd \
  --imu-port /dev/ttyUSB0 \
  --imu-baud 230400 \
  --dvl-port /dev/null \
  --log-dir ./data
```

预期输出：

```
[IMU] serial opened @230400
[IMU] started
```

### 6.2 日志验证

```bash
ls -lh data/imu.bin
```

检查文件非 0 字节。

十六进制查看：

```bash
hexdump -C data/imu.bin | head
```

---

# 7. DVL 单独测试流程（Hover H1000）

**DVL 必须在水中，否则禁止长时间测试。**

### 7.1 启动 DVL 驱动

```bash
sudo ./uwnav_navd \
  --imu-port /dev/null \
  --dvl-port /dev/ttyUSB1 \
  --dvl-baud 115200 \
  --log-dir ./data
```

预期输出：

```
[DVL] serial opened @115200
```

### 7.2 检查日志

```bash
ls -lh data/dvl.bin
hexdump -C data/dvl.bin | head
```

---

# 8. 全链路测试（IMU + DVL + ESKF）

### 8.1 启动 nav_daemon

```bash
sudo ./uwnav_navd \
  --imu-port /dev/ttyUSB0 \
  --imu-baud 230400 \
  --dvl-port /dev/ttyUSB1 \
  --dvl-baud 115200 \
  --log-dir ./data
```

预期控制台输出：

```
[IMU] started ...
[DVL] started ...
[navd] Started navigation daemon
```

### 8.2 查看所有日志文件

```bash
ls -lh data
```

你应看到：

| 文件       | 内容                   |
| -------- | -------------------- |
| imu.bin  | 100Hz IMU 原始数据       |
| dvl.bin  | 10Hz DVL 速度数据（PD6解析） |
| eskf.bin | ESKF 输出状态（位置/速度/姿态）  |

日志文件非 0 字节代表正常工作。

---

# 9. 日志格式与验证方法

二进制日志文件（POD struct dump）支持 Numpy 直接读取。

示例 (IMU)：

```python
import numpy as np

dt = np.dtype([
    ("mono_ns","<i8"),
    ("est_ns","<i8"),
    ("acc","<f4",3),
    ("gyro","<f4",3),
    ("euler","<f4",3),
    ("temp","<f4"),
    ("valid","u1"),
    ("pad","u1",3),
])

imu = np.fromfile("imu.bin", dtype=dt)
print(imu[:5])
```

---

# 10. 常见故障与排查

## 10.1 IMU 无数据

症状：

* imu.bin 大小为 0
* 控制台无 IMU 回调输出
* `uwnav_navd` 反复打印 `IMU opened but no parseable frame arrived ...`

排查方法：

```bash
sudo hexdump -C /dev/ttyUSB0 | head
```

若全是 `FF FF` 或 `00 00`：

* 99% 为 **RS485 A/B 接反**

解决：交换 A/B 线。

从 2026-03-31 起，`uwnav_navd` 在“串口已打开但 IMU 没有可解析帧”时会额外输出：

* `IMU serial diagnostic: kind=... rx_bytes=... summary=...`
* `IMU serial preview(text): ...` 或 `IMU serial preview(hex): ...`

同一版本开始，IMU 设备绑定不再只依赖 `port`/`binding` 的硬编码路径：

* 启动时会先用 **115200 被动观察** 候选串口，优先识别 Volt32 这类会自动回传 `CHn:` 文本的设备；
* 对未命中 Volt32 特征的串口，再用 **IMU 配置波特率主动发送 WIT Modbus 读寄存器请求**；
* 只有观察到合法 `0x50 0x03 ... CRC` 回复的串口，才会被当作 IMU 口绑定。

因此：

* `driver.port`、`candidate_paths`、`expected_vid/pid/serial` 现在主要作为 **排序提示和诊断信息**；
* 真正决定“哪个串口是 IMU”的依据，已经变成 **串口行为特征 + Modbus 探测结果**；
* C++ 驱动的串口初始化也已对齐 Python：使用 raw `8N1`，避免此前字符位设置错误导致“Python 可读、C++ 无法解析”的差异。

如果 `kind=volt32_ascii`，通常说明 **IMU/电压板串口发生了重枚举跳变**，当前绑定到的并不是 WIT IMU 口，而是 Volt32 文本电压采集口。此时优先检查：

* `/dev/serial/by-id` 软链接是否变化
* IMU 与 Volt32 的 USB-RS485 转接器是否对调
* `nav_events.csv` 中是否出现 `serial_protocol_diagnostic`

同一版本开始，`uwnav_navd` 还会把 ESKF 审查结果单独汇总成低频告警：

* stderr 会在根因变化时打印 `health audit changed: health=... cause=... summary=...`
* `nav_events.csv` 会同步记录 `nav_health_audit_changed`

根因字段目前分为：

* `transport_timing`：样本 stale / out-of-order / 心跳超时，更像串口重连、时序链路或线程调度问题
* `sensor_input`：预处理或 gating 大量拒绝，更像传感器原始数据本身异常
* `estimator_consistency`：传感器仍在线，但 DVL XY/Z 更新接受率和 NIS 长时间异常，更像 ESKF 参数或观测模型需要调整
* `estimator_numeric`：ESKF 状态出现 NaN/Inf，属于算法数值级异常

排障顺序建议：

1. 先看 `cause`，决定优先排查传感器、串口链路还是 ESKF 参数。
2. 再看 `summary` 里的计数和比率，确认是哪一条输入流在持续拉低导航质量。
3. 最后结合 `nav_publish_state_changed`、`serial_protocol_diagnostic` 和原始 binlog 做细查。

如果当前已经确认 IMU 在某个串口上，但还想直接看“它到底回了什么原始字节”，可以使用新增的主动探测工具：

```bash
cd /home/wys/orangepi/UnderwaterRobotSystem/Underwater-robot-navigation/nav_core
./build/bin/uwnav_imu_modbus_probe \
  --port /dev/ttyUSB1 \
  --baud 230400 \
  --addr 0x50 \
  --attempts 3 \
  --reply-timeout-ms 120
```

说明：

* `uwnav_imu_modbus_probe` 默认按标准 Modbus RTU 的 CRC 线序发送（lo byte, hi byte）。
* 如果现场设备实现存在历史漂移，可追加 `--crc-order both` 让工具自动两种顺序都试一次。

工具会输出：

* 发送出去的 Modbus 请求十六进制
* 收到的原始返回 `rx_dump`
* 当前协议分类结果：`imu_modbus_reply` / `volt32_ascii` / `other_ascii` / `other_binary`

如果需要把原始返回单独保存，追加：

```bash
--output /tmp/imu_usb1_probe.bin
```

---

## 10.2 DVL 无数据或 PD6 解码失败

症状：

```
parsed_fail 激增
dvl.bin 大小极小
```

排查：

* DVL 是否在水中？
* 使用 cat 查看：

```bash
sudo cat /dev/ttyUSB1
```

能否看到 PD6 帧，例如：

```
:BE,+12,+5,+0,A
```

---

## 10.3 nav_daemon 运行后立刻退出

可能原因：

* 端口不存在
* 权限不足（需要 sudo）
* DVL 断电或空中保护导致串口无数据

排查：

```bash
dmesg | tail -20
```

---

## 10.4 ESKF 无输出（eskf.bin 为 0 字节）

原因：

* 未收到第一帧有效 IMU 数据
* DVL 全为无效帧（V）
* IMU 姿态角恒定（设备未移动）

推荐解决：

* 手动移动 IMU，产生明显姿态变化
* 检查 IMU 安装角与配置是否一致

---

## 10.5 IMU 解析错误（出现 NAN / Inf）

常见原因：

* RS485 线过长、接触不良
* 上位机波特率设置错误
* 设备仍在回复字节，但回复内容不是当前 WIT Modbus 寄存器窗口

检查波特率：

```bash
stty -F /dev/ttyUSB0
```

如果控制台已经打印 `IMU serial preview(hex): 50 03 1e ...` 这类 Modbus 十六进制帧，但仍没有 IMU 回调，优先检查：

* `slave_addr` 是否与实物一致
* 设备是否处于 WIT Modbus 模式，而不是旧 `0x55` 同步帧模式
* 当前请求的寄存器窗口是否与实物固件兼容

---

# 11. 安全注意事项

| 风险         | 对策                    |
| ---------- | --------------------- |
| **DVL 空转** | 必须在水中工作，不超过 2 秒空转     |
| 电压波动损坏 DVL | 使用稳定 24V 电源           |
| RS485 线接反  | 会导致无数据，但不会烧毁          |
| USB 电源不足   | DVL 不建议从 SBC USB 端口供电 |
| 水池实验安全     | 避免金属掉入池中              |

---

# 12. 建议的实验顺序

1. **IMU-only 测试**（100 Hz）
2. **DVL-only 测试**（10 Hz）
3. **ESKF（IMU-only）**
4. **真实 IMU + DVL 联合运行**
5. **短时间水池实验（稳定性验证）**
6. **长时间水池实验（完整数据 pipeline 测试）**

---
