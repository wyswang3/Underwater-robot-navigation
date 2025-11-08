#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU数据记录程序

功能：
- 读取IMU设备的数据，进行实时采集
- 保存原始IMU数据与滤波后的数据
- 采用Unix时间戳格式来标记每个数据点
"""

import time
import numpy as np
from queue import Empty, Queue
from datetime import datetime
from uwnav.drivers.imu.WitHighModbus import device_model
from uwnav.drivers.imu.WitHighModbus.filters import RealTimeIMUFilter
import csv
import os
import signal
import argparse
import threading
from typing import Tuple

# ---- 调试开关 ----
DEBUG_PRINT_EVERY = 25   # 每 N 条样本打印一次概览；0 关闭
WARN_IF_IDLE_SEC  = 3.0  # N 秒没收到样本则报警
FLUSH_PERIOD_SEC  = 0.5  # 定期刷盘周期
# -------------------

class IMUData:
    """封装IMU数据，存储原始数据和滤波后的数据"""
    def __init__(self):
        self.timestamp = 0.0
        self.acc = np.zeros(3)   # 加速度数据
        self.gyr = np.zeros(3)   # 角速度数据
        self.filtered_acc = np.zeros(3)  # 滤波后的加速度
        self.filtered_gyr = np.zeros(3)  # 滤波后的角速度
        self.yaw = 0.0           # 滤波后的偏航角

    def update(self, timestamp, acc, gyr):
        """更新IMU数据"""
        self.timestamp = timestamp
        self.acc = acc
        self.gyr = gyr

    def filter_data(self, filter_instance):
        """使用滤波器处理数据"""
        filtered_data = filter_instance.process_sample(self.timestamp, self.acc, self.gyr)
        if filtered_data:
            self.filtered_acc, self.filtered_gyr, self.yaw = filtered_data

class IMUDevice:
    def __init__(self, port, baud, address, raw_writer, fil_writer):
        self.port = port
        self.baud = baud
        self.address = address
        self.device = None
        self.filter = RealTimeIMUFilter(fs=50, cutoff=4.0, calibrate=True)
        self.sample_q = Queue(maxsize=4000)
        self.shutdown_event = threading.Event()
        self.imu_data = IMUData()  # 实例化 IMUData 类
        self._stats = {"recv": 0, "wraw": 0, "wfilt": 0, "exc": 0, "last_recv_t": 0.0}
        self.raw_writer = raw_writer  # 用于写入原始数据
        self.fil_writer = fil_writer  # 用于写入滤波后的数据

    def open_device(self):
        """打开IMU设备连接"""
        try:
            self.device = device_model.DeviceModel(
                deviceName="IMUDevice",
                portName=self.port,
                baud=self.baud,
                ADDR=self.address,
                callback_method=self.update_data
            )
            self.device.openDevice()
            self.device.startLoopRead()
            print(f"IMU device connected at {self.port} with baud {self.baud}")
        except Exception as e:
            print(f"Error opening device: {e}")

    def close_device(self):
        """关闭设备连接"""
        if self.device:
            self.device.stopLoopRead()
            self.device.closeDevice()

    def update_data(self, data):
        """数据采集回调：采集并将数据放入结构体中"""
        if self.shutdown_event.is_set():
            return
        ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        t = time.time()
        try:
            acc = np.array([data.get("AccX"), data.get("AccY"), data.get("AccZ")], dtype=float)
            gyr = np.array([data.get("AsX"), data.get("AsY"), data.get("AsZ")], dtype=float)
            self.imu_data.update(t, acc, gyr)  # 更新IMU数据结构
            self.sample_q.put_nowait(self.imu_data)
            self._stats["recv"] += 1
            self._stats["last_recv_t"] = t
            # 每25条样本打印一次概览
            if DEBUG_PRINT_EVERY and self._stats["recv"] % DEBUG_PRINT_EVERY == 0:
                print(f"[RECV] {self._stats['recv']}  acc={tuple(np.round(acc, 3))} gyr={tuple(np.round(gyr, 3))}")
        except Exception as e:
            self._stats["exc"] += 1
            print(f"[WARN] 读取数据异常: {e}")

    def monitor_data(self):
        """监控数据，检测是否有长时间没有收到新数据"""
        now = time.time()
        if self._stats["last_recv_t"] and (now - self._stats["last_recv_t"] > WARN_IF_IDLE_SEC):
            print(f"[WARN] {self._stats['recv']} 样本后 {now - self._stats['last_recv_t']:.1f}s 未收到新数据，检查端口/波特率/设备模式")

    def process_data(self):
        """数据处理：滤波并保存"""
        while not self.shutdown_event.is_set():
            try:
                imu_data = self.sample_q.get(timeout=0.1)
                imu_data.filter_data(self.filter)  # 滤波数据
                self.raw_writer.writerow([imu_data.timestamp, *imu_data.acc, *imu_data.gyr])
                self._stats["wraw"] += 1
                self.fil_writer.writerow([imu_data.timestamp, *imu_data.filtered_acc, *imu_data.filtered_gyr, imu_data.yaw])
                self._stats["wfilt"] += 1
            except Empty:
                continue

    def start(self):
        """启动数据采集、滤波和存储线程"""
        data_thread = threading.Thread(target=self.process_data)
        data_thread.daemon = True
        data_thread.start()
        return data_thread


def make_writers(out_dir: str) -> Tuple[object, csv.writer, object, csv.writer]:
    os.makedirs(out_dir, exist_ok=True)
    date_str = time.strftime("%Y%m%d")
    raw_path = os.path.join(out_dir, f"imu_raw_data_{date_str}.csv")
    fil_path = os.path.join(out_dir, f"imu_filtered_data_{date_str}.csv")
    raw_f = open(raw_path, "a", newline="", encoding="utf-8")
    fil_f = open(fil_path, "a", newline="", encoding="utf-8")
    raw_w, fil_w = csv.writer(raw_f), csv.writer(fil_f)
    if raw_f.tell() == 0:
        raw_w.writerow(["Timestamp", "AccX", "AccY", "AccZ", "AsX", "AsY", "AsZ", "HX", "HY", "HZ", "AngX", "AngY", "AngZ"])
    if fil_f.tell() == 0:
        fil_w.writerow(["Timestamp", "AccX_filt", "AccY_filt", "AccZ_filt", "AsX_filt", "AsY_filt", "AsZ_filt", "Yaw_filt(deg)"])
    
    return raw_f, raw_w, fil_f, fil_w

def main():
    out_dir = './output_data'  # 设置输出文件目录
    raw_f, raw_w, fil_f, fil_w = make_writers(out_dir)

    imu_device = IMUDevice(port='/dev/ttyUSB1', baud=230400, address=0x50, raw_writer=raw_w, fil_writer=fil_w)
    imu_device.open_device()

    imu_device.start()

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        imu_device.close_device()

if __name__ == "__main__":
    main()
