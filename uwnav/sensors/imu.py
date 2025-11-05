#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu.py
------
IMU 设备抽象层，负责与 IMU 设备通信、数据获取与处理。

功能：
1. 通过串口读取 IMU 数据，支持 RS-485 Modbus 通信；
2. 将原始数据（加速度、角速度、磁场、欧拉角）进行单位转换；
3. 提供统一接口 `IMUData`，以供后续处理与融合。

依赖：`pyserial`
"""

import serial
import logging
import time
import numpy as np
from queue import Queue
import threading
from uwnav.drivers.imu.WitHighModbus import device_model

# IMU 数据结构
class IMUData:
    """
    用于存储 IMU 数据的结构体。
    """
    def __init__(self, timestamp: float, acc: np.ndarray, gyr: np.ndarray, mag: np.ndarray, angles: np.ndarray):
        self.timestamp = timestamp  # 时间戳
        self.acc = acc  # 加速度（m/s²）
        self.gyr = gyr  # 角速度（rad/s）
        self.mag = mag  # 磁场（高斯）
        self.angles = angles  # 欧拉角（弧度）

    def __repr__(self):
        return f"IMUData(timestamp={self.timestamp}, acc={self.acc}, gyr={self.gyr}, mag={self.mag}, angles={self.angles})"


class IMUInterface:
    """
    IMU 设备通信接口类，通过串口与 IMU 设备连接并获取数据。
    """

    def __init__(self, port: str, baud: int = 230400, addr: int = 0x50, callback_method=None, data_queue: Queue = None):
        """
        初始化 IMU 接口。

        :param port: 串口路径（如：/dev/ttyUSB0）
        :param baud: 波特率（默认为 230400）
        :param addr: 设备地址（默认为 0x50）
        :param callback_method: 解析后的数据回调方法
        :param data_queue: 用于将解析后的数据传递到其他模块（如数据记录、融合等）
        """
        self.port = port
        self.baud = baud
        self.addr = addr
        self.callback_method = callback_method
        self.data_queue = data_queue or Queue(maxsize=4000)  # 默认队列，存储解析后的数据
        self.serial_conn = None

    def open(self):
        """
        打开串口连接。
        """
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
            logging.info(f"IMU 串口连接成功: {self.port}，波特率 {self.baud}")
        except serial.SerialException as e:
            logging.error(f"IMU 串口连接失败：{e}")
            return False
        return True

    def close(self):
        """
        关闭串口连接。
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logging.info("IMU 串口连接已关闭")

    def read_data(self):
        """
        从串口读取 IMU 数据（加速度、角速度、磁场、欧拉角）。
        
        :return: 返回 IMU 数据行
        """
        try:
            data = self.serial_conn.readline().decode("ascii").strip()
            return data
        except serial.SerialException as e:
            logging.error(f"读取 IMU 数据失败: {e}")
            return None

    def listen(self):
        """
        持续监听 IMU 数据，解析并调用回调方法处理。
        将解析后的数据放入队列中，以供其他线程使用。
        """
        while True:
            line = self.read_data()
            if line:
                parsed_data = self.parse_data(line)
                if parsed_data and self.callback_method:
                    self.callback_method(parsed_data)

    def parse_data(self, line: str):
        """
        解析一行数据，将其转换为 IMUData 对象。

        :param line: 从串口读取的一行数据
        :return: 解析后的 IMU 数据（IMUData 实例）
        """
        try:
            values = list(map(float, line.split(',')))
            if len(values) != 12:  # 预期有12个数据项
                logging.warning(f"无效数据行：{line}")
                return None
            acc = np.array(values[0:3])  # 加速度（m/s²）
            gyr = np.array(values[3:6])  # 角速度（rad/s）
            mag = np.array(values[6:9])  # 磁场（高斯）
            angles = np.array(values[9:12])  # 欧拉角（弧度）
            timestamp = time.time()  # 获取当前时间戳
            imu_data = IMUData(timestamp, acc, gyr, mag, angles)
            
            # 将数据放入队列中，供其他线程（如记录线程、融合线程）使用
            self.data_queue.put_nowait(imu_data)
            return imu_data
        except Exception as e:
            logging.error(f"解析数据失败: {e} | 数据行：{line}")
            return None

    def start_listening(self):
        """
        启动 IMU 数据监听，持续接收并处理数据。
        """
        if self.open():
            listen_thread = threading.Thread(target=self.listen, daemon=True)
            listen_thread.start()
            logging.info("IMU 数据监听线程启动成功")
        else:
            logging.error("无法启动 IMU 数据监听线程，设备连接失败")

# ------------------- 示例用法 -------------------
if __name__ == "__main__":
    def data_callback(parsed_data):
        """
        示例数据回调函数，用于接收解析后的 IMU 数据。
        """
        logging.info(f"接收到 IMU 数据: {parsed_data}")

    # 配置设备串口参数
    port = "/dev/ttyUSB0"
    baud = 230400
    addr = 0x50

    # 初始化 IMU 接口
    imu_interface = IMUInterface(port=port, baud=baud, addr=addr, callback_method=data_callback)
    
    # 启动数据监听
    imu_interface.start_listening()

    try:
        while True:
            time.sleep(1)  # 主线程保持运行
    except KeyboardInterrupt:
        # 处理 CTRL+C 中断，停止监听并关闭连接
        imu_interface.close()
