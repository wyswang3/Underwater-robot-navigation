# WitHighModbus

这个目录保留的是 WIT High Modbus IMU 的 early driver / protocol exploration 资料。

它的当前定位不是“项目权威导航入口”，而是：

- 记录 IMU 协议摸底和串口交互经验
- 保留早期 Python-side bring-up 代码与参考
- 作为当前 `nav_core` IMU runtime 的历史背景材料

## 当前状态 / Current Status

当前项目已经把在线 IMU 主链逐步收口到 `nav_core` 的 C++ runtime。

因此这个目录现在更适合作为：

- protocol reference
- bring-up note
- historical context

而不是默认运行入口。

## 这个目录里能看到什么

- Modbus-like serial communication 起步实现
- IMU register 读取与解析示例
- 早期 `pyserial` 路径和串口参数经验
- ROS Python 参考链接

## 当前建议

如果你只是想理解“现在项目里的 IMU 怎样进入导航主链”，优先看：

1. `../../../nav_core/README.md`
2. `../../../docs/传感器与协议说明.md`
3. `../../../docs/导航运行总览.md`

如果你想看早期协议摸底和寄存器探索，再回来看这个目录。

## 历史背景

这个目录最初用于：

- 先打通 IMU 串口通信
- 形成最小数据解析闭环
- 为后续 DVL、时间同步和融合铺路

它对理解项目早期演化很有价值，但不再代表当前运行时主线。
