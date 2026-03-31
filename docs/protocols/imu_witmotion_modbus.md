
---

# WitMotion HWT9073-485 通信协议说明（Modbus-RTU）

**文档位置：** `docs/protocols/imu_witmotion_modbus.md`
**适配设备：** WitMotion HWT9073-485 / HWT9051 / HWT9059 系列
**通信方式：** RS-485（半双工）
**协议：** Modbus-RTU（厂家扩展格式）
**驱动文件：** `nav_core/src/imu_driver_wit.cpp`
**SDK 文件：** `nav_core/third_party/witmotion/wit_c_sdk.c`

---

# 1. 硬件通信参数

| 项目    | 值                     |
| ----- | --------------------- |
| 电气接口  | RS-485 半双工            |
| 默认协议  | Modbus-RTU            |
| 默认地址  | `0x50`                |
| 默认波特率 | **230400**（本项目中已统一修改） |
| 数据格式  | 8N1（8 数据位、无校验、1 停止位）  |

---

# 2. 数据读取方式总览

HWT9073 使用一种非常“特别”的 Modbus 模式：

* 不使用标准 Modbus 功能码 `0x03`
* 而是通过 *厂家的 SDK* 生成 Modbus 帧
* 将每个字节通过串口送回 IMU
* IMU 回复一段“寄存器更新事件”式的数据结构
* 最终由 `wit_c_sdk.c` 内部的状态机解析到全局寄存器数组 `sReg[]`

因此驱动的核心流程是：

```
WitReadReg(AX, 15)         # 请求读取 AX 开始的 15 个寄存器
↓
SDK 生成 Modbus 字节流
↓
onSerialWrite() 写到串口
↓
IMU 返回 Modbus 帧
↓
驱动逐字节执行：
    WitSerialDataIn(byte)
↓
厂家 SDK 内部解析
↓
触发注册的回调函数：（C → C++）
    regUpdate(start_reg, count)
↓
在 sReg[] 中更新数据
↓
驱动将寄存器转为物理量（rad/s、m/s²）
```

---

# 3. 寄存器布局（REG.h）

以下寄存器宏由官方提供，位于：

```
nav_core/third_party/witmotion/REG.h
```

重点寄存器：

| 名称               | 描述          | 类型      |
| ---------------- | ----------- | ------- |
| `AX, AY, AZ`     | 加速度（单位：raw） | `int16` |
| `GX, GY, GZ`     | 角速度（raw）    | `int16` |
| `HRoll, LRoll`   | Roll 高/低字   | `int16` |
| `HPitch, LPitch` | Pitch 高/低字  | `int16` |
| `HYaw, LYaw`     | Yaw 高/低字    | `int16` |
| `TEMP905x`       | 温度          | `int16` |

官方寄存器读取法（SDK 原示例）：

```cpp
a[i]   = sReg[AX + i] / 32768.0 * 16.0;      // unit: g
w[i]   = sReg[GX + i] / 32768.0 * 2000.0;    // unit: deg/s

Angle[i] = (
    (uint32_t)sReg[HRoll + 2*i] << 16 |
    (uint16_t)sReg[LRoll + 2*i]
) / 1000.0;                                  // unit: deg

Temp = sReg[TEMP905x] / 100.0;               // Celsius
```

---

# 4. 本项目中的读取流程（nav_core 实现）

驱动文件：

```
nav_core/src/imu_driver_wit.cpp
```

## 4.1 轮询策略

设备内部更新率默认 **100Hz**，因此我们的驱动以相同频率查询：

* 每 10ms 执行一次 `WitReadReg(AX, 15)`
* 设置一个 **5ms 接收窗口**，使用 `select()` + `read()` 抽取响应字节
* 每个字节交给 `WitSerialDataIn(byte)`

说明：
当前在线 runtime 轮询的是 `0x34..0x42`，覆盖 acc / gyro / mag / 高精度姿态角；
`TEMP905x(0x43)` 不在这一轮轮询包内，因此温度当前不作为在线链路判据。

供参考：

```cpp
const int poll_hz     = 100;
const int period_ms   = 1000 / poll_hz;
const int recv_win_ms = 5;
```

---

# 5. 数据格式与物理量转换

## 5.1 加速度（g → m/s²）

原始格式：`int16 raw`

转换规则：

```
acc[g] = raw / 32768 * 16
acc[m/s²] = acc[g] * 9.80665
```

## 5.2 角速度（deg/s → rad/s）

```
gyro[dps] = raw / 32768 * 2000
gyro[rad/s] = gyro[dps] * (π/180)
```

## 5.3 姿态角（deg → rad）

高低寄存器拼为 32bit：

```
angle[deg] = ((hi << 16) | lo) / 1000
angle[rad] = angle[deg] * (π / 180)
```

---

# 6. 回调流程（C SDK → C++）

SDK 内部维护全局寄存器数组：

```c
int16_t sReg[REGSIZE];
```

当寄存器更新时，会触发注册的回调：

```cpp
WitRegisterCallBack(&ImuDriverWit::regUpdateBridge);
```

桥接为 C++ 成员函数：

```cpp
void ImuDriverWit::onRegUpdate(uint32_t start_reg, uint32_t num);
```

由该函数将一次寄存器更新转为：

* `ImuRawRegs`（记录更新信息）
* `ImuFrame`（结构化物理量 + 时间戳）

---

# 7. 输出数据结构（ImuFrame）

文件：`nav_core/include/nav_core/types.h`

```cpp
struct ImuFrame {
    int64_t mono_ns;      // 单调时间戳（高稳定度）
    int64_t est_ns;       // 系统估计时间戳（可与日志对齐）

    float lin_acc[3];     // m/s²
    float ang_vel[3];     // rad/s
    float euler[3];       // rad

    float temperature;    // °C

    bool  valid;
    int   status;
};
```

---

# 8. 过滤逻辑

在 `ImuDriverWit::makeFrameFromRegs()` 中实现基础过滤：

### 过滤机制

| 检查项       | 条件   |                |                   |    |   |    |                               |
| --------- | ---- | -------------- | ----------------- | -- | - | -- | ----------------------------- |
| 加速度范围     |      | ax             | ,                 | ay | , | az | < `max_abs_accel`（默认 50 m/s²） |
| 角速度范围     |      | wx             | ,                 | wy | , | wz | < `max_abs_gyro`              |
| 欧拉角范围（可选） |      | roll/pitch/yaw | < `max_euler_abs` |    |   |    |                               |
| NaN / inf | 自动过滤 |                |                   |    |   |    |                               |

过滤配置结构：

```cpp
struct ImuFilterConfig {
    bool enable_accel = true;
    bool enable_gyro  = true;
    bool enable_euler = false;

    double max_abs_accel = 50.0;
    double max_abs_gyro  = 10.0;
    double max_euler_abs = 3.5;
};
```

---

# 9. 与 nav_daemon 的集成

`nav_core/src/nav_daemon.cpp` 将：

1. 加载配置（未来支持 YAML）
2. 启动 IMU 驱动：

   ```cpp
   ImuDriverWit imu("/dev/ttyUSB0", 230400, 0x50, cfg, imu_callback);
   imu.start();
   ```
3. 将 IMU 数据传递给：

   * ESKF（预测步）
   * 数据记录系统（bin_logger）

IMU → ESKF 的典型频率为 **100Hz**。

---

# 10. 常见问题（FAQ）

### Q1. 为什么我们不用标准 Modbus 功能码 0x03？

A：
WitMotion 的 Modbus 版本是“带状态机的扩展 Modbus”，必须使用官方 SDK 做解析。
不适合自己写裸 Modbus。

---

### Q2. 为什么不能用一个线程读串口、一个线程解解析？

A：
SDK 要求 **严格按字节顺序调用** `WitSerialDataIn()`。
多线程会破坏顺序。

---

### Q3. 是否可以提高到 200Hz？

A：
可以，但要求：

* IMU 输出率设置为 200Hz（寄存器配置）
* 串口波特率 >= 230400

目前 100Hz 是最稳定配置。

---

# 11. 附录：常用 SDK API

| 函数                           | 说明          |
| ---------------------------- | ----------- |
| `WitInit(mode, addr)`        | 初始化 SDK     |
| `WitSerialWriteRegister(cb)` | 注册串口写函数     |
| `WitRegisterCallBack(cb)`    | 注册寄存器更新回调   |
| `WitReadReg(start, count)`   | 请求读取寄存器     |
| `WitSerialDataIn(byte)`      | 往 SDK 输入一字节 |
| `WitDeInit()`                | 反初始化        |

---

# 12. Python 通信实现（uwnav + apps）

本章节介绍如何利用 Python 对 WitMotion HWT9073-485 IMU 进行通讯、采集与调试。
Python 侧 **不参与实时导航**，作用主要是：

* 数据采集
* 实验管线（pipelines）
* 数据质量验证、可视化
* 离线融合（Python 版 ESKF 原型）
* 调参、对比 nav_core 的输出

Python 与 C++ 的 IMU 读取方式不同：

| 语言                | 方式                                      | 优点         | 缺点                |
| ----------------- | --------------------------------------- | ---------- | ----------------- |
| **C++（nav_core）** | 官方 WitMotion C SDK（推荐）                  | 高性能；实时性；稳定 | 无法完全自定义协议         |
| **Python**        | 1) 直接读取原始 Modbus 帧<br>2) 串口字节流仿真 SDK 行为 | 灵活；易调试     | 性能不足以跑 100Hz 实时导航 |

下面介绍三种 python 通信方式。

---

# 12.1 Python 的串口初始化（PySerial）

文件位置示例：

```
apps/acquire/imu_logger.py
uwnav/drivers/imu/wit_modbus.py （可选）
```

### 基础打开方式

```python
import serial

ser = serial.Serial(
    port="/dev/ttyUSB0",
    baudrate=230400,
    parity=serial.PARITY_NONE,
    bytesize=serial.EIGHTBITS,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.01
)
```

### Windows 示例

```python
ser = serial.Serial("COM7", 230400, timeout=0.01)
```

Python 的 `timeout` 必须短，否则会阻塞读取线程。

---

# 12.2 Python 方法 1：使用厂家 ASCII 模式（最简单）

WitMotion 大部分 IMU **支持 ASCII 模式**输出（如 `+0.12,-0.03,9.81`），但 9073-485 并不总是默认开启。
如果你执行过 Modbus 配置指令（多数情况），ASCII 会被关闭。

ASCII 输出格式示例（类似 9051 系列）：

```
AX=0.01, AY=0.02, AZ=0.03
GX=0.10, GY=0.20, GZ=0.30
Angle=20.1, 3.2, -45.0
Temp=27.5
```

处理方式：

```python
line = ser.readline().decode(errors="ignore").strip()

if "AX" in line:
    parts = line.split(",")
    ax = float(parts[0].split("=")[1])
    ay = float(parts[1].split("=")[1])
    ...
```

**优点：** 解析简单
**缺点：** 不适用于你当前设备的高频实时导航（已被关闭）

---

# 12.3 Python 方法 2：直接解析 Modbus 帧（推荐给 Python 工具）

这是与你 C++ 版本一致的方式，也与 Wit SDK 工作方式一致，但我们不直接用 SDK，而是自己解析 Modbus RTU 帧。

Modbus 帧结构：

```
[Slave Addr][Function][Reg Start][Reg Count][CRC Lo][CRC Hi]
[Data...]
```

示例读取 15 个寄存器：

```python
# 读寄存器指令（使用标准 Modbus 0x03）：
req = bytes([
    0x50,        # addr
    0x03,        # read holding register
    0x00, 0x34,  # start reg = AX = 0x34
    0x00, 0x0f,  # count = 15
    crc_lo, crc_hi
])
ser.write(req)
```

然后读取服务器返回的 Modbus 帧，解析每个寄存器（2字节）：

```python
resp = ser.read(2 + 2 + 15*2 + 2)
```

并拆解成：

```python
values = []
for i in range(15):
    high = resp[3 + i*2]
    low  = resp[3 + i*2 + 1]
    values.append((high<<8) | low)
```

接着你可以用与 C++ 一样的公式转换成物理量：

```python
acc_g = values[AX - base] / 32768.0 * 16.0
gyro_dps = values[GX - base] / 32768.0 * 2000.0
roll_deg = ((values[HRoll]*65536 + values[LRoll]) / 1000.0)
```

---

# 12.4 Python 方法 3：仿真官方 C SDK 的字节流解析器

你也可以直接模仿 WitMotion C SDK 的完整状态机（Python 版），使用：

* 状态机（FSM）
* 字节缓冲
* 校验（crc16）
* 完整帧校验
* 触发寄存器更新回调

这样 Python 与 C++ 行为 **完全一致**，非常适合：

* 多传感器后处理（multisensor_postproc）
* 与 C++ nav_core 输出做对比验证

示例解析结构：

```python
class WitParser:
    def __init__(self):
        self.buf = bytearray()
        self.state = 0   # 等待头 / 读取长度 / CRC 校验 / ...
        self.sReg = [0]*256

    def input_byte(self, b):
        # 模仿 wit_c_sdk.c 中 WitSerialDataIn()
        ...
```

---

# 13. Python IMU 采集脚本（apps/acquire/imu_logger.py）

当前推荐的“采集脚本”结构如下：

```python
def main():
    ser = serial.Serial("/dev/ttyUSB0", 230400, timeout=0.005)
    parser = WitParser()     # 或者 “方法 2”的解析器

    while True:
        data = ser.read(128)
        for b in data:
            frame = parser.input_byte(b)
            if frame is not None:
                # frame = {acc, gyro, euler, temperature}
                save_csv(frame)
```

可直接写入：

```
data/YYYY-MM-DD/imu/raw_XXXX.csv
```

这与 C++ nav_daemon 的设计完全兼容。

---

# 14. Python 与 C++ 的职责分层（非常关键）

| 目标            | 推荐语言                              | 原因                 |
| ------------- | --------------------------------- | ------------------ |
| 实时导航（100Hz）   | **C++（nav_core）**                 | 实时性 & 稳定性          |
| 数据采集          | Python apps/acquire               | 灵活、快速开发            |
| 数据质量检查        | Python apps/tools                 | 可视化能力强             |
| 多传感器对齐        | Python tools/multisensor_postproc | SciPy/NumPy        |
| 融合算法原型        | Python uwnav/fusion               | 快速迭代               |
| 正式 ESKF       | C++ nav_core                      | 船上部署               |
| 离线学习（LSTM/RL） | Python                            | PyTorch/TensorFlow |

这种分层让你的系统变得：

* **稳定（C++）**
* **灵活（Python）**
* **易维护（清晰职责）**

---
