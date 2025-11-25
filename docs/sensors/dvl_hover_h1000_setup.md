---

# **DVL Hover H1000 使用与安装说明**

文件名：`dvl_hover_h1000_setup.md`
版本：v1.0
适用平台：水下机器人 / ROV / AUV
对应软件：`uwnav.drivers.dvl.hover_h1000`（Python / C++）

---

# **1. 概述（Overview）**

DVL Hover H1000 是一款高精度多普勒测速仪（Doppler Velocity Log），可提供：

* 东/北/上方向速度（VE/VN/VU）
* 深度（可选）
* 有效标志 A/V（Valid/Invalid）
* 可配置的 Ping Rate、Data Format（PD6 / EPD6）

它是水下导航系统中关键的“速度传感器”，通常与：

* IMU（惯性测量单元）
* 深度计
* USBL
* 声呐/视觉

一起用于融合定位（例如 ESKF）。

本系统支持 H1000 的：

* 底层串口解析 protocol（PD6/EPD6）
* 行级 logs（raw）
* 速度帧（parsed）
* 最小速度表（timebase 对齐）
* 时间统一与延迟补偿

---

# **2. 机械安装（Mechanical Mounting）**

## **2.1 安装方向**

DVL 的 *正 Z 轴* 通常指向“下方”（垂直朝向水底）。
一般安装方式：

* 头部（换能器阵列）指向水底
* 机身方向与 ROV 的机体坐标系一致（建议对正 X / Y）

## **2.2 安装位置**

建议位置：

* 底部中心位置
* 尽可能远离螺旋桨推进器
* 避免直接贴近金属结构（可能影响声学反射）

## **2.3 抗震与隔振**

DVL 需要：

* 固定牢靠
* 不被振动干扰
* 避免螺旋桨高速振动导致速度误差

---

# **3. 电气与供电（Power）**

## **3.1 额定供电**

典型供电要求（以 Hover H1000 常见版本为例）：

| 电压          | 电流      | 注意点         |
| ----------- | ------- | ----------- |
| **24 V DC** | 峰值约 1 A | 测试时务必确保电压稳定 |

你的项目规划中，已经决定：

* 由 ROV 主电源（12 V）→ DC/DC 升压模块 → 24 V
* OPI/STM32 IO 控制“电子开关”决定 DVL 上电/断电
* 上位系统安全限制：**DVL 不可在空气中长时间工作**

### **安全建议：水下开机 / 水外关机**

必须遵循：

* **入水 → 开机**（CS）
* **出水 → 立即关机**（CZ）

你已经实现：

* 物理切断供电（电子开关）＋ 软件发送 CZ 命令
* 可进一步加入**深度传感器联动规则**（深度 > 0.2 m 自动允许开机）

---

# **4. 通信接口（Serial / UART）**

## **4.1 串口参数（默认为 PD6）**

常用串口配置：

| 参数  | 值           |
| --- | ----------- |
| 波特率 | **115200**  |
| 数据位 | 8           |
| 停止位 | 1 或 2（硬件决定） |
| 校验位 | N           |
| 流控  | 可关闭         |

你的 Python/C++ 驱动均已支持：

```text
--parity N/E/O  
--stopbits 1/2  
--rtscts  
--dsrdtr  
--xonxoff  
```

## **4.2 串口命令（AT-like）**

主要命令（来自 Hover H1000）：

| 命令     | 描述                        |
| ------ | ------------------------- |
| `CS`   | Start ping                |
| `CZ`   | Stop ping                 |
| `DF=x` | Data format（0=PD6，6=EPD6） |
| `PR=x` | Ping rate（Hz）             |
| `PM=x` | 平均次数                      |
| `ST`   | 同步时间（接收 UTC）              |

你的代码已经支持自动发送：

* 启动时：**CZ → CS**
* 关闭时：**CZ**

这是正确、安全的流程。

---

# **5. 数据格式（PD6 / EPD6）**

系统已支持解析以下帧类型：

| 类型   | 方向                         | 说明                |
| ---- | -------------------------- | ----------------- |
| `BE` | bottom-track               | East / North / Up |
| `BS` | bottom-track               | Same as BE        |
| `BI` | bottom-track（integer form） |                   |
| `WE` | water-mass                 | East              |
| `WS` | water-mass                 | North             |
| `WI` | water-mass 帧               |                   |

关键字段：

* VE（mm/s）
* VN（mm/s）
* VU（mm/s）
* Valid Flag：A/V

你的解析器通过扫描整行文本找到这些帧，并解析有效整数。

---

# **6. 软件接口（Python & C++）**

## **6.1 Python（apps/acquire/DVL_logger.py）**

包含：

* `on_raw()`
* `on_parsed()`
* `MinimalSpeedWriterTB`（记录双时间戳）

用户数据格式（MinimalSpeed）：

| 字段       | 说明          |
| -------- | ----------- |
| MonoNS   | 单调时间        |
| EstNS    | 估计实时时间      |
| MonoS    | 以秒计的 MonoNS |
| EstS     | 以秒计的 EstNS  |
| Src      | BE/BS/BI 等  |
| VE/VN/VU | 速度（m/s）     |

适用于：

* 数据对齐
* ESKF 输入
* 神经网络训练（动力学辨识）

---

## **6.2 C++（nav_core::dvl_driver）**

核心函数：

* `parseLine()`：解析一个行帧
* `passFilter()`：速度有效性过滤
* `handleRawSample()`：推送到回调
* 自动打 timestamp（统一时间体系）

与 IMU 完全对齐后，具备：

* 精准 dt
* IMP/INS 优化
* 时序正确性

---

# **7. 时间统一（Timebase Integration）**

你的系统采用：

* C++：`uwnav::timebase`
* Python：`uwnav.io.timebase`

对 DVL 的 timestamp：

```python
ts = stamp("dvl0", SensorKind.DVL)
mono_ns = ts.host_time_ns
est_ns  = ts.corrected_time_ns
```

vs. IMU：

```python
ts = stamp("imu0", SensorKind.IMU)
```

它们会在同一条时间轴上运行，从而使融合时：

* DVL 5–9 Hz
* IMU 50–100 Hz

能够可靠对齐。

这对于：

* ESKF（速度测量更新）
* MPC（预测步长稳定）
* LSTM（输入窗口配准）

都是必要条件。

---

# **8. 调试与验证（Debugging & Verification）**

## **8.1 检查原始数据是否“干净”**

至少应包含：

```
:BE +123 -45 +8 A
:BS ...
:BI ...
```

如出现：

* 行缺损
* 数字格式不对
* 非法字符

你的解析器会自动跳过。

---

## **8.2 检查有效率（Valid A vs V）**

必要时可打开：

```
--raw-only
--stat-every=1
```

看 valid=A 的比例。

---

## **8.3 测试水下速度**

在水下静止时，速度应接近：

```
[0.00 ± noise]
```

噪声范围典型 ±1–5 mm/s。

---

# **9. 水下测试注意事项**

这一部分非常关键。

### **9.1 切勿在空气中长时间启动 DVL！**

风险：

* 换能器可能因无载波介质而过热
* 声波反射异常导致设备错误
* 厂家明确禁止

你已实现：

* 开机前必须先执行 CZ
* 仅在入水后执行 CS 启动
* 可通过深度计自动判断水中/空气状态

请保留此逻辑。

---

## **9.2 保持水平**

DVL 在以下情况下速度退化：

* 倾角>20–25°
* 大量气泡
* 水底泥较软
* 水草/障碍物

若为 ROV，保持姿态稳定可提升速度质量。

---

## **9.3 水底距离**

Hover H1000 的最佳工作距离（常用配置）：

| 距离      | 速度测量质量        |
| ------- | ------------- |
| 0.3–5 m | 最佳            |
| <0.2 m  | 可能过近反射不稳定     |
| >8–10 m | 信号衰减，valid 降低 |

---

# **10. 推荐的数据流程图（你的系统匹配版）**

```
DVL Serial → line → parseLine() → raw callback
                        ↓
                  passFilter()
                        ↓
                 DvlFrame (VE/VN/VU)
                        ↓
        stamp("dvl0", DVL) → (mono_ns, est_ns)
                        ↓
             MinimalSpeedWriterTB → CSV
                        ↓
             ESKF / LSTM / MPC 输入
```

---

# **11. 总结**

本文件帮助团队成员快速上手 Hover H1000，主要包含：

1. 安装、电气、供电、安全规范
2. 串口参数、命令
3. Python/C++ 驱动结构
4. 统一时间基与日志规范
5. 下水调试注意事项
6. 与 IMU 深度计融合的要求
---
