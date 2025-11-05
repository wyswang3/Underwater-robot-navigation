#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_logger.py
-------------
实时读取 IMU 数据并记录到 CSV 文件中，同时将数据传递给融合系统（如 ESKF）。

功能：
1) 启动 IMU 数据采集程序（imu.py）；
2) 从 `IMUData` 结构体中获取数据；
3) 将原始数据和滤波数据分别保存为 CSV 文件；
4) 文件名包含时间戳，避免覆盖。
"""
from typing import Tuple
import signal
import time
import csv
import os
import argparse
import threading
from datetime import datetime
from queue import Queue, Empty

# 假设 imu.py 中有 IMUDevice 和 IMUData 类
from uwnav.sensors.imu import IMUDevice, IMUData

# 共享队列
sample_q = Queue(maxsize=4000)
shutdown_event = threading.Event()

# 统计变量
_stats = {"recv": 0, "wraw": 0, "wfilt": 0, "exc": 0, "last_recv_t": 0.0}

def make_writers(out_dir: str) -> Tuple[object, csv.writer, object, csv.writer]:
    """创建并打开原始数据和滤波后的数据 CSV 文件，文件名按时间生成"""
    os.makedirs(out_dir, exist_ok=True)
    date_str = time.strftime("%Y%m%d%H%M%S")  # 文件名带上具体时间
    raw_path = os.path.join(out_dir, f"imu_raw_data_{date_str}.csv")
    fil_path = os.path.join(out_dir, f"imu_filtered_data_{date_str}.csv")
    raw_f = open(raw_path, "a", newline="", encoding="utf-8")
    fil_f = open(fil_path, "a", newline="", encoding="utf-8")
    raw_w, fil_w = csv.writer(raw_f), csv.writer(fil_f)

    if raw_f.tell() == 0:
        raw_w.writerow(["Timestamp", "AccX", "AccY", "AccZ", "GyroX", "GyroY", "GyroZ"])
        raw_f.flush()
    if fil_f.tell() == 0:
        fil_w.writerow(["Timestamp", "AccX_filt", "AccY_filt", "AccZ_filt", "GyroX_filt", "GyroY_filt", "GyroZ_filt", "Yaw_filt"])
        fil_f.flush()

    return raw_f, raw_w, fil_f, fil_w

def consumer_thread(raw_writer, raw_file, fil_writer, fil_file):
    """消费线程：从队列获取数据并写入 CSV 文件"""
    while not shutdown_event.is_set():
        try:
            item = sample_q.get(timeout=0.1)
        except Empty:
            continue

        # 保存原始数据
        raw_writer.writerow([item.timestamp, *item.acc, *item.gyr])
        _stats["wraw"] += 1

        # 保存滤波数据
        fil_writer.writerow([item.timestamp, *item.filtered_acc, *item.filtered_gyr, item.yaw])
        _stats["wfilt"] += 1

def parse_args():
    """解析命令行参数"""
    ap = argparse.ArgumentParser(description="IMU 数据采集与记录")
    ap.add_argument("--outdir", default="./data", help="输出目录")
    return ap.parse_args()

def main():
    """主函数：初始化设备，启动数据采集与记录线程"""
    args = parse_args()
    out_dir = args.outdir
    raw_f, raw_w, fil_f, fil_w = make_writers(out_dir)

    # 启动 IMU 数据采集程序（imu.py）
    imu_device = IMUDevice(port='/dev/ttyUSB1', baud=230400, address=0x50, callback_method=add_to_queue)
    imu_device.open_device()
    imu_device.start()

    # 启动消费线程
    cons_th = threading.Thread(target=consumer_thread, args=(raw_w, raw_f, fil_w, fil_f), daemon=True)
    cons_th.start()

    # 注册退出信号
    def on_signal(sig, frame):
        print("\n[SYS ] 收到退出信号，准备关闭…")
        shutdown_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    try:
        while not shutdown_event.is_set():
            time.sleep(0.5)  # 主线程保持运行
    except KeyboardInterrupt:
        shutdown_event.set()

    cons_th.join()

def add_to_queue(imu_data):
    """将IMU数据添加到队列中"""
    try:
        sample_q.put_nowait(imu_data)
    except Exception as e:
        print(f"Error adding data to queue: {e}")

if __name__ == "__main__":
    main()
