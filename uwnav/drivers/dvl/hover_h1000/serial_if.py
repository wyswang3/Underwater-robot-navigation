#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
serial_if.py
------------
串口通信接口，负责与 DVL（Hover H1000）设备进行数据交换。

功能：
1. 通过 RS-485 串口与 DVL 设备连接（波特率默认115200，8N1 格式）；
2. 持续接收数据行，调用协议解析函数 `parse_line()` 进行解析；
3. 支持重连机制，串口连接失败时自动重新连接。

依赖：`pyserial`
"""

import serial
import time
import threading
import logging
from uwnav.drivers.dvl.hover_h1000.protocol import parse_line

# 串口接收缓冲区大小，最大队列数
BUFFER_SIZE = 100

class DVLSerialInterface:
    """
    串口通信接口类，用于与 DVL 设备建立连接，接收数据并传递给协议解析函数。
    """
    
    def __init__(self, port: str, baud: int = 115200, callback_method=None):
        """
        初始化串口通信类。
        
        :param port: 串口设备路径（如 /dev/ttyUSB0）
        :param baud: 串口波特率（默认为115200）
        :param callback_method: 数据解析后的回调方法，默认不传入
        """
        self.port = port
        self.baud = baud
        self.callback_method = callback_method
        self.serial_conn = None
        self._stop_event = threading.Event()  # 停止线程的事件标志

    def open(self):
        """
        打开串口连接。
        """
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
            logging.info(f"串口 {self.port} 连接成功，波特率 {self.baud}")
        except serial.SerialException as e:
            logging.error(f"无法打开串口 {self.port}: {e}")
            return False
        return True

    def close(self):
        """
        关闭串口连接。
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logging.info(f"串口 {self.port} 关闭成功")
        
    def read_line(self):
        """
        从串口读取一行数据，行末为 \r\n。
        
        :return: 返回一行字符串（去除结尾的 \r\n）
        """
        try:
            line = self.serial_conn.readline().decode("ascii").strip()
            return line
        except serial.SerialException as e:
            logging.error(f"串口读取错误: {e}")
            return None

    def listen(self):
        """
        串口数据监听线程，持续读取串口数据并调用协议解析函数。
        """
        while not self._stop_event.is_set():
            line = self.read_line()  # 读取一行数据
            if line:
                parsed_data = parse_line(line)  # 调用协议解析函数
                if parsed_data and self.callback_method:
                    self.callback_method(parsed_data)  # 回调处理解析后的数据
            time.sleep(0.01)  # 控制读取速度，避免过度占用 CPU

    def start_listening(self):
        """
        启动数据监听线程，开始从串口读取数据。
        """
        if self.open():
            self._stop_event.clear()  # 重置停止事件标志
            self.listener_thread = threading.Thread(target=self.listen, daemon=True)
            self.listener_thread.start()
            logging.info("数据监听线程启动成功")
        else:
            logging.error("无法启动数据监听线程，串口未打开")

    def stop_listening(self):
        """
        停止数据监听线程，关闭串口连接。
        """
        self._stop_event.set()
        self.listener_thread.join()
        self.close()
        logging.info("数据监听线程已停止")

# ------------------- 示例用法 -------------------
if __name__ == "__main__":
    def data_callback(parsed_data):
        """
        示例数据回调函数，用于接收解析后的数据。
        """
        logging.info(f"接收到的数据: {parsed_data}")

    # 串口参数配置
    port = "/dev/ttyUSB0"  # 串口路径
    baud = 115200  # 波特率

    # 初始化串口接口
    dvl_interface = DVLSerialInterface(port=port, baud=baud, callback_method=data_callback)
    
    # 启动数据监听
    dvl_interface.start_listening()

    try:
        while True:
            time.sleep(1)  # 主线程保持运行
    except KeyboardInterrupt:
        # 处理 CTRL+C 中断，停止监听并关闭串口
        dvl_interface.stop_listening()
