#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tcp_if.py
---------
TCP 通信接口，负责与 DVL（Hover H1000）设备进行数据交换。

功能：
1. 通过 TCP 连接与 DVL 设备通信（数据端口 10001，命令端口 10000）；
2. 持续接收数据并调用协议解析函数 `parse_line()`；
3. 支持发送控制命令，控制 DVL 设备的工作状态。

依赖：`socket`
"""

import socket
import threading
import logging
import time
from uwnav.drivers.dvl.hover_h1000.protocol import parse_line

class DVLTcpInterface:
    """
    TCP 客户端类，用于与 DVL 设备的通信。
    连接到设备的命令端口（10000）和数据端口（10001），
    解析数据并通过回调函数处理。
    """

    def __init__(self, ip: str, data_port: int, cmd_port: int, callback_method=None):
        """
        初始化 TCP 客户端接口。
        
        :param ip: 设备的 IP 地址
        :param data_port: 数据端口，默认为 10001
        :param cmd_port: 命令端口，默认为 10000
        :param callback_method: 数据解析后的回调函数
        """
        self.ip = ip
        self.data_port = data_port
        self.cmd_port = cmd_port
        self.callback_method = callback_method
        self.data_socket = None
        self.cmd_socket = None
        self._stop_event = threading.Event()  # 停止线程的事件标志

    def connect(self):
        """
        连接到 DVL 设备的数据端口和命令端口。
        """
        try:
            # 创建 TCP 套接字
            self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            # 连接数据端口
            self.data_socket.connect((self.ip, self.data_port))
            logging.info(f"已连接到数据端口 {self.data_port}，设备 IP: {self.ip}")

            # 连接命令端口
            self.cmd_socket.connect((self.ip, self.cmd_port))
            logging.info(f"已连接到命令端口 {self.cmd_port}，设备 IP: {self.ip}")
        except Exception as e:
            logging.error(f"无法连接到 DVL 设备：{e}")
            return False
        return True

    def close(self):
        """
        关闭 TCP 连接。
        """
        if self.data_socket:
            self.data_socket.close()
            logging.info("数据端口连接已关闭")
        if self.cmd_socket:
            self.cmd_socket.close()
            logging.info("命令端口连接已关闭")

    def send_command(self, command: bytes):
        """
        向 DVL 设备发送控制命令。
        
        :param command: 16 字节命令字节串
        """
        try:
            self.cmd_socket.sendall(command)
            logging.info(f"发送命令: {command}")
        except Exception as e:
            logging.error(f"发送命令失败: {e}")

    def read_line(self):
        """
        从数据端口读取一行数据，返回数据行（去掉换行符）。
        
        :return: 一行数据（字符串）
        """
        try:
            line = self.data_socket.recv(1024).decode("ascii").strip()  # 最大接收 1024 字节
            return line
        except Exception as e:
            logging.error(f"读取数据失败: {e}")
            return None

    def listen(self):
        """
        数据监听线程，持续读取数据并传递给回调函数进行处理。
        """
        while not self._stop_event.is_set():
            line = self.read_line()  # 读取一行数据
            if line:
                parsed_data = parse_line(line)  # 调用协议解析函数
                if parsed_data and self.callback_method:
                    self.callback_method(parsed_data)  # 调用回调函数处理解析后的数据
            time.sleep(0.01)  # 控制读取速度，避免过度占用 CPU

    def start_listening(self):
        """
        启动数据监听线程，开始从 DVL 数据端口读取数据。
        """
        if self.connect():
            self._stop_event.clear()  # 重置停止事件标志
            self.listener_thread = threading.Thread(target=self.listen, daemon=True)
            self.listener_thread.start()
            logging.info("数据监听线程启动成功")
        else:
            logging.error("无法启动数据监听线程，设备连接失败")

    def stop_listening(self):
        """
        停止数据监听线程，关闭 TCP 连接。
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

    # 配置设备 IP 和端口
    ip = "192.168.2.102"  # DVL 设备 IP 地址
    data_port = 10001     # 数据端口
    cmd_port = 10000      # 命令端口

    # 初始化 DVL TCP 接口
    dvl_interface = DVLTcpInterface(ip=ip, data_port=data_port, cmd_port=cmd_port, callback_method=data_callback)
    
    # 启动数据监听
    dvl_interface.start_listening()

    try:
        while True:
            time.sleep(1)  # 主线程保持运行
    except KeyboardInterrupt:
        # 处理 CTRL+C 中断，停止监听并关闭连接
        dvl_interface.stop_listening()
