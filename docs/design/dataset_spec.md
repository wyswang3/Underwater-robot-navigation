
---

# **Dataset Specification**

**多传感器原始与融合数据集规范文档**
**Underwater Robot Navigation System**

---

## 1. 文档目的（Purpose）

本规范说明项目中所有传感器数据集的字段、时间戳格式、单位、输出目录、融合格式等内容，作为：

* 数据采集
* 离线可视化
* 状态估计（ESKF）
* 动力学建模（LSTM / Hybrid Model）
* 控制算法（MPC / RL）

的统一数据接口标准。

所有传感器数据均按：

```
data/YYYY-MM-DD/<sensor_type>/
```

分类存储，确保可追溯性与可扩展性。

---

# **2. 时间戳规范（Timebase Specification）**

系统采用 **双时间基格式**，保证在不同线程、串口、设备间保持一致。

| 字段名           | 类型     | 描述                                  |
| ------------- | ------ | ----------------------------------- |
| **MonoNS**    | int64  | 单调递增时间，纳秒（不随系统时间跳变）。适合滤波与对齐。        |
| **EstNS**     | int64  | 系统估计时间（实时时间），纳秒。用于人类可读。             |
| **MonoS**     | float  | MonoNS / 1e9                        |
| **EstS**      | float  | EstNS / 1e9                         |
| **Timestamp** | string | 人类可读时间：`YYYY-MM-DD HH:MM:SS.ssssss` |

**对齐与融合采用主时间轴：`EstS`（或 t_s_dvl）。**
跨设备主要参考的是 DVL 的 EstS。

---

# **3. IMU 数据集规范（IMU Dataset Specification）**

文件路径：

```
data/YYYY-MM-DD/imu/
    imu_raw_data_*.csv
    imu_filtered_data_*.csv
```

---

## **3.1 原始 IMU 数据（imu_raw_data_*.csv）**

| 字段名       | 单位     | 含义           |
| --------- | ------ | ------------ |
| Timestamp | string | 原始接收时间（系统时间） |
| EstS      | float  | 接收时刻（秒）      |
| AccX      | g      | 三轴加速度原始值     |
| AccY      | g      |              |
| AccZ      | g      |              |
| GyroX     | °/s    | 三轴角速度        |
| GyroY     | °/s    |              |
| GyroZ     | °/s    |              |
| Roll      | °      | 欧拉角（若开启）     |
| Pitch     | °      |              |
| Yaw       | °      |              |
| Temp      | °C     | 芯片温度         |

说明：

* 加速度单位为 g（之后会在神经网络与 ESKF 中转换为 m/s²）
* 无噪声抑制
* 原样取自 Modbus 包解析结果

---

## **3.2 滤波后 IMU 数据（imu_filtered_data_*.csv）**

| 字段名           | 单位        | 含义       |
| ------------- | --------- | -------- |
| Timestamp     | string    | 接收时间     |
| AccX_filt     | g or m/s² | 低通滤波后加速度 |
| AccY_filt     |           |          |
| AccZ_filt     |           |          |
| AsX_filt      | °/s       | 滤波后角速度   |
| AsY_filt      |           |          |
| AsZ_filt      |           |          |
| Yaw_filt(deg) | °         | 低噪声航向角   |
| t_s_imu       | float     | 时间戳：秒    |

说明：

* 滤波：窗口可调
* `t_s_imu` 用于 merge_asof 对齐 DVL 时间轴
* 对齐后将用于 ESKF、LSTM 输入

---

# **4. DVL 数据集规范（DVL Dataset Specification）**

文件路径：

```
data/YYYY-MM-DD/dvl/
    dvl_speed_min_tb_*.csv
    dvl_raw_*.txt
```

---

## **4.1 最小速度表（dvl_speed_min_tb_*.csv）**

这是当前多传感器对齐的 **主时间轴**。

| 字段名            | 单位     | 含义              |
| -------------- | ------ | --------------- |
| MonoNS         | int64  | 单调时间（纳秒）        |
| EstNS          | int64  | 系统时间（纳秒）        |
| MonoS          | float  | MonoNS / 1e9    |
| EstS           | float  | EstNS / 1e9     |
| SensorID       | string | 固定为 "DVL_H1000" |
| Src            | string | PD6 / EPD6      |
| East_Vel(m_s)  | m/s    | 东向速度            |
| North_Vel(m_s) | m/s    | 北向速度            |
| Up_Vel(m_s)    | m/s    | 上向速度            |

说明：

* 仅当三个速度轴都有数据时才写入
* `EstS` 是后处理中的参考时间轴
* 对齐 IMU / Volt 时常使用：

  * `left_on="t_s"`（来自 DVL）
  * `right_on="t_s_imu"` / `t_s_volt`

---

# **5. Volt32 数据集规范（Volt Dataset Specification）**

文件路径：

```
data/YYYY-MM-DD/volt/
    motor_data_*.csv
```

数据格式（示例）：

| 字段名       | 单位     | 含义         |
| --------- | ------ | ---------- |
| Timestamp | string | 接收时间（系统时间） |
| CH0       | V 或 A  | 根据硬件定义     |
| CH1       |        |            |
| …         |        |            |
| CH15      |        |            |
| t_s_volt  | float  | 时间戳秒       |

说明：

* 通道个数可能为 16/32
* 通道含义由硬件文档决定（可扩展到功率、电流、PWM）
* 对齐时通常采用**最近邻插值**

---

# **6. 多传感器对齐数据集（Aligned Dataset）**

文件路径：

```
data/YYYY-MM-DD/aligned/
    aligned_imu_dvl_volt_YYYY-MM-DD.csv
```

此文件由脚本：

```
apps/tools/multisensor_postproc.py
```

生成。

它包含 **IMU + DVL + Volt** 所有关键字段：

---

## **6.1 核心字段（通用）**

| 字段名  | 单位 | 来源  | 含义         |
| ---- | -- | --- | ---------- |
| t_s  | s  | DVL | 主时间轴（EstS） |
| EstS | s  | DVL | 系统估计秒      |

---

## **6.2 DVL 字段**

| 字段名            | 单位     | 含义         |
| -------------- | ------ | ---------- |
| East_Vel(m_s)  | m/s    | 东向速度       |
| North_Vel(m_s) | m/s    | 北向速度       |
| Up_Vel(m_s)    | m/s    | 上向速度       |
| Src            | string | PD6 / EPD6 |

---

## **6.3 IMU 字段**

（若最近时间点存在容差匹配）

| 字段名           | 单位  | 含义         |
| ------------- | --- | ---------- |
| AccX_filt     | g   | 滤波加速度      |
| AccY_filt     | g   |            |
| AccZ_filt     | g   |            |
| AsX_filt      | °/s | 滤波角速度      |
| AsY_filt      | °/s |            |
| AsZ_filt      | °/s |            |
| Yaw_filt(deg) | °   | 航向角        |
| t_s_imu       | s   | 原始 IMU 时间戳 |

---

## **6.4 Volt 字段**

| 字段名        | 单位     | 含义       |
| ---------- | ------ | -------- |
| CH0 ~ CH15 | V or A | 原始电压电流通道 |
| t_s_volt   | s      | Volt 时间戳 |

---

## **6.5 特殊情况说明**

| 情况           | 处理方式               |
| ------------ | ------------------ |
| 某时刻无 IMU 数据  | 对应 IMU 字段填 NaN     |
| 某时刻无 Volt 数据 | Volt 字段填 NaN       |
| DVL 缺少速度轴    | 整行不写入 speed_min_tb |

---

# **7. 单位规范（Units）**

稳定使用国际标准单位：

| 类型      | 单位                | 备注             |
| ------- | ----------------- | -------------- |
| 加速度     | g（raw）/ m/s²（后处理） | 转换系数 g * 9.78  |
| 角速度     | °/s               |                |
| 速度（DVL） | m/s               |                |
| 时间戳     | 秒                 | 浮点（EstS/MonoS） |
| 电压      | V                 |                |
| 电流      | A                 |                |
| 温度      | °C                |                |

---

# **8. 对齐规则（Alignment Rules）**

后处理使用：

```
pandas.merge_asof
```

规则：

* 以 DVL 的 `t_s_dvl` 作为左表
* 对齐 IMU: `right_on='t_s_imu'`
* 对齐 Volt: `right_on='t_s_volt'`
* 容差：默认 0.2 秒，可调
* 插值模式：nearest（最近邻）

---

# **9. 典型用途（Use Cases）**

### 9.1 ESKF 离线仿真

使用对齐后的：

```
[t, Acc, Gyro, DVL, Volt]
```

验证滤波器的：

* 噪声参数
* 门控
* DVL 外点剔除策略
* IMU 偏置估计

### 9.2 神经网络动力学辨识

使用：

* 输入：Volt → 16 通道 + IMU 加速度/角速度
* 输出：DVL（三轴速度）

用于训练 LSTM / Hybrid Dynamics Net。

### 9.3 控制器（MPC/RL）

使用：

* DVL/IMU 作观测
* 状态/动力学预测模型来自对齐数据训练结果

---

# **10. 后续扩展（Future Extensions）**

未来加入：

| 传感器          | 字段        |
| ------------ | --------- |
| Depth Sensor | 深度、深度速度   |
| USBL         | 位置（X,Y,Z） |
| Barometer    | 压力、温度     |
| Camera       | 时序时间戳对齐   |

后处理脚本将自动识别更多传感器目录：

```
data/YYYY-MM-DD/usbl/
data/YYYY-MM-DD/depth/
```

并加入扩展字段。

---

# **11. 版本说明（Version History）**

| 版本   | 日期      | 内容                     |
| ---- | ------- | ---------------------- |
| v1.0 | 2025-11 | 初版，支持 IMU + DVL + Volt |
| v1.1 | 预留      | 加 USBL + Depth 同步支持    |

---
