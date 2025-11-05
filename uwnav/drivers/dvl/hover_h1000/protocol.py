#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
protocol.py
-----------
PD6/EPD6 协议解析与命令组帧模块。

功能：
1. 解析 PD6/EPD6 格式的文本帧，提取数据并进行单位换算。
2. 构造 16 字节命令，用于控制 DVL 设备的工作状态。

支持的语句：
- `SA` 系统姿态
- `TS` 时间与系统参数
- `BI` 体坐标系下速度
- `BE` 大地坐标系下速度
- `BD` 大地坐标系下位移与高度
"""

import numpy as np

# 解析数据的字典映射
VALID_FLAGS = {"A": True, "V": False}

# ----------------- 协议解析函数 -----------------

def parse_line(line: str):
    """
    解析一行 PD6/EPD6 数据帧。
    根据行头识别对应类型的数据，并进行单位换算。
    只返回有效的数据（例如，速度、角度、深度等）。

    :param line: 输入的一行 PD6 或 EPD6 格式的字符串
    :return: 如果数据有效，返回字段字典；否则返回 None
    """
    if not line.startswith(':'):
        return None  # 无效行

    msg_type = line[1:3]  # 提取数据类型（如 SA, TS, BE 等）
    fields = [f.strip() for f in line[3:].split(',')]  # 提取数据字段并去除空格

    # 处理不同类型的语句
    if msg_type == "SA":  # 姿态信息
        try:
            pitch, roll, heading = map(float, fields)
            return {"pitch": pitch, "roll": roll, "heading": heading, "type": "SA"}
        except ValueError:
            return None

    elif msg_type == "TS":  # 时间与系统信息
        try:
            timestamp, salinity, temperature, depth, sound_speed, status = fields
            timestamp = int(timestamp)  # 时间戳处理
            return {"timestamp": timestamp, "salinity": float(salinity), "temperature": float(temperature),
                    "depth": float(depth), "sound_speed": float(sound_speed), "status": status, "type": "TS"}
        except ValueError:
            return None

    elif msg_type == "BI":  # 体坐标系下速度
        try:
            vx, vy, vz, error, flag = fields
            vx, vy, vz = map(int, (vx, vy, vz))  # 将速度转换为整数
            return {"vx": vx / 1000.0, "vy": vy / 1000.0, "vz": vz / 1000.0, "error": error, "valid": VALID_FLAGS.get(flag, False), "type": "BI"}
        except ValueError:
            return None

    elif msg_type == "BE":  # 大地坐标系下速度
        try:
            ve, vn, vu, flag = fields
            ve, vn, vu = map(int, (ve, vn, vu))
            return {"ve": ve / 1000.0, "vn": vn / 1000.0, "vu": vu / 1000.0, "valid": VALID_FLAGS.get(flag, False), "type": "BE"}
        except ValueError:
            return None

    elif msg_type == "BD":  # 大地坐标系下位移与高度
        try:
            e, n, u, depth, timestamp = fields
            e, n, u = map(float, (e, n, u))
            depth = float(depth)
            return {"e": e, "n": n, "u": u, "depth": depth, "timestamp": timestamp, "type": "BD"}
        except ValueError:
            return None

    else:
        return None  # 不支持的语句类型

# ----------------- 命令构造函数 -----------------

def build_command(cmd: str, arg: str | int | float | None = None):
    """
    根据命令类型构造 16 字节命令帧。
    
    :param cmd: 命令类型（例如 "PR"、"DF"、"CS"）
    :param arg: 命令参数（根据不同命令类型，类型不同）
    :return: 16 字节命令帧，作为字节串
    """
    if cmd == "PR":  # 设置更新频率
        return f"PR {str(arg).ljust(16, '0')}\r".encode("ascii")
    
    elif cmd == "BX":  # 设置量程（斜距）
        return f"BX {str(arg).ljust(16, '0')}\r".encode("ascii")

    elif cmd == "DF":  # 设置输出格式
        return f"DF {str(arg).ljust(16, '0')}\r".encode("ascii")

    elif cmd == "BR":  # 设置波特率
        return f"BR {str(arg).ljust(16, '0')}\r".encode("ascii")

    elif cmd == "IP":  # 修改 IP 地址
        return f"IP {str(arg).ljust(16, '0')}\r".encode("ascii")

    elif cmd == "CS":  # 启动测量
        return "CS \r000000000000".encode("ascii")

    elif cmd == "CZ":  # 停止测量
        return "CZ \r000000000000".encode("ascii")

    else:
        return None  # 未知命令

# ----------------- 调试与验证 -----------------

if __name__ == "__main__":
    # 测试协议解析与命令生成
    test_line = ":SA,-1.76,0.07,322.57"
    parsed_data = parse_line(test_line)
    print("Parsed Data:", parsed_data)
    
    test_cmd = build_command("PR", 10)
    print("Generated Command:", test_cmd)
