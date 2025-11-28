---

# **统一时间基使用规范（Timebase Specification）**

版本：v1.0
适用范围：IMU / DVL / 深度计 / 电压采集 / USBL / 主控程序 / 数据采集工具
对应实现：`uwnav.timebase`（C++）与 `uwnav.io.timebase`（Python）

---

# **1. 统一时间的目标**

在多传感器融合（IMU + DVL + Depth + USBL）的系统中，每个传感器：

* 以**不同频率**工作
* 具有**不同的通信延迟**
* 数据到达**不同线程**
* 消息经过**不同缓冲队列**、中间处理流程
* 数据可能受**系统负载影响**而抖动

传统做法如直接使用 `time.time()` 或 `system_clock`，会导致：

* 时间不连续、跳变
* 传感器之间时间轴不一致
* 软件线程调度导致时延不确定
* 同一帧数据在不同脚本/程序下记录不同时间戳
* DVL、IMU、深度计之间的配准失败
* ESKF/MPC 的预测与实际步长不一致
* 地下/水下无 NTP/无网络时，无法同步精确时间

因此，在没有外部高精度时钟（GNSS/NTP）的情况下，需要建立**本系统自己的统一时间基（Timebase）**，为所有传感器提供一致的“系统内时间”。

---

# **2. 时间基设计原则**

系统采用两种时间：

### **2.1 单调时间（MonoNS）**

* 来源：`std::steady_clock`（C++）
* 或 `time.monotonic_ns()`（Python）
* 单调递增，无跳变
* 适用于融合/预测/积分：

  * IMU INS 积分
  * ESKF 的 dt
  * MPC 的控制步长
* 纳秒级分辨率，确保时序排序唯一。

### **2.2 估计实时时间（EstNS）**

* 仅用于：

  * 日志文件
  * 可视化
  * 人类查看
  * 和外部系统（如 CSV）对齐
* 来自：
  `est_ns = epoch_real_ns + (mono_now - epoch_mono_ns)`
* 可以被 resync（例如上位机提供一个新的绝对时刻）

---

# **3. 统一调用规范**

## **3.1 C++**

```cpp
#include "timebase.h"
using uwnav::timebase::stamp;
using uwnav::timebase::SensorKind;

// IMU / DVL / Depth / USBL
auto ts = stamp("imu0", SensorKind::IMU);

int64_t mono_ns = ts.host_time_ns;
int64_t est_ns  = ts.corrected_time_ns;
```

### 建议的 sensor_id：

| 传感器  | sensor_id  | SensorKind |
| ---- | ---------- | ---------- |
| IMU  | `"imu0"`   | `IMU`      |
| DVL  | `"dvl0"`   | `DVL`      |
| 深度计  | `"depth0"` | `DEPTH`    |
| 电压采集 | `"volt0"`  | `OTHER`    |
| USBL | `"usbl0"`  | `USBL`     |

---

## **3.2 Python**

```python
from uwnav.io.timebase import stamp, SensorKind

ts = stamp("imu0", SensorKind.IMU)
mono_ns = ts.host_time_ns
est_ns  = ts.corrected_time_ns
```

Python 与 C++ 完全一致。

---

# **4. 统一 CSV 输出规范**

所有传感器的 CSV / 二进制日志都应输出以下字段：

| 字段       | 解释              | 用途             |
| -------- | --------------- | -------------- |
| `MonoNS` | 单调时间（ns）        | 融合、排序、配准、dt 计算 |
| `EstNS`  | 实时估计时间（ns）      | 可读性、外部同步       |
| `MonoS`  | `MonoNS * 1e-9` | 可读秒，调试用        |
| `EstS`   | `EstNS * 1e-9`  | 可读秒，调试用        |

之后才是传感器自己的内容，如：

* IMU：Acc/Gyr/Euler
* DVL：VE/VN/VU
* 深度计：depth
* 电压采集：CH0..CH31

例如 IMU Raw：

```
MonoNS, EstNS, MonoS, EstS, AccX, AccY, AccZ, GyrX, GyrY, GyrZ
```

例如 DVL：

```
MonoNS, EstNS, MonoS, EstS, Src, East_Vel, North_Vel, Up_Vel
```

---

# **5. 统一时间的必要性**

## **5.1 传感器频率不一致导致的时间误差**

例如：

* IMU 滤波后输出 100 Hz
* DVL 输出 5–9 Hz
* 深度计输出 20–50 Hz

如果每个传感器都用自己的 system_clock 记录时间，会造成：

* 不同线程之间时间基不同
* 多个 Python 进程 / C++ 程序之间时间不一致
* Raspberry Pi 的系统时间受温度与负载跳变

融合时出现如下现象：

* DVL 一次更新可能对应 IMU 多个预测步
* INS 积分 dt 不正确
* 时间配准错位，姿态误差累积
* MPC 预测状态不稳定

统一时间基能保证：
**所有传感器都在同一时间轴上。**

---

## **5.2 去除系统时间跳变**

`system_clock` 会因为：

* 手动设置时间
* NTP 自动校时
* 进入/退出节能模式
* 内核负载变化

而发生跳变（向前/向后跳）。

这对 INS / ESKF 是致命的：

* dt<0 导致融合失败
* 角速度积分反向
* MPC 预测的时间步长失效

使用 monotonic clock 可以彻底避免。

---

## **5.3 解决不同线程/不同进程的延迟不确定性**

例如 IMU 数据来自某个线程，DVL 来自另一个线程，数据到达时间：

* 依赖内核调度
* 依赖缓冲区大小
* 依赖 CPU 忙闲程度
* 依赖串口缓存 flush 时机

统一时间基通过：

* stamp() 在入队瞬间采样时间
* 使用延迟补偿（latency_ns）

确保：

**即使处理晚了，时间也仍然准确反映“数据发生时刻”。**

---

## **5.4 日志、分析、回放需要统一时间**

为后期融合算法（ESKF/AMCL/USBL）重放数据时：

* 如果传感器各记各的时间，无法对齐
* IMU 一秒有 100 个点，但 DVL 只 5–9 个
* 回放程序无法确定哪个 IMU 样本对应哪个 DVL 样本
* 深度计突变点无法与 IMU 的加速度尖峰配对

统一时间基后：

* 深度跳变与 IMU 瞬时加速度波动可以直接匹配
* 回放顺序唯一
* 融合算法得到真实的 dt

---

## **5.5 下水测试时，多个进程分散运行**

你目前的系统实际运行方式为：

* 主程序（C++ 导航）
* 采集工具（Python）
* 其他工具（电压、电流、温度、舵机等）

这些工具**没有共同的程序起点**。
如果不统一时间，不可能保证它们之间时间对齐。

统一时间基后，无论：

* 在不同线程
* 不同脚本
* 不同程序
* 甚至不同语言（C++ / Python）

都能共享一套时间基。

---

# **6. 未来扩展：支持外部参考时钟（USBL / PPS / RTC）**

Timebase 未来可扩展：

* 接入 USBL 时间戳
* 读取海试时的 PPS（Pulse Per Second）
* 支持深度计的硬件时间同步

通过：

```cpp
resync(new_real_ns);
```

即可做到无缝切换，不破坏 mono 基准。

---

# **最终总结**

统一时间基带来的核心收益：

### **1. 融合正确（保证 dt）**

### **2. 各传感器时间轴唯一且一致**

### **3. 线程/进程调度不再影响时间记录**

### **4. 回放、绘图、调试、对齐变得可靠**

### **5. 下水实验数据可直接送入 ESKF / 神经网络 / MPC**

这是整个导航与控制系统“最底层但最重要”的基础设施之一。

---
