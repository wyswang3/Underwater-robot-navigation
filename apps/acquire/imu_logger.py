#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_logger.py
-------------
实时读取 IMU 数据并记录到 CSV 文件中。

功能：
1) 从队列获取 IMU 数据；
2) 将数据分别保存为原始数据 CSV 和滤波后数据 CSV；
3) 数据记录和存储不会影响数据的解析和融合处理。
"""

import argparse
import os
import time
import csv
import signal
import threading
from datetime import datetime
from queue import Queue, Empty
from typing import Tuple

# 共享队列
sample_q = Queue(maxsize=4000)
shutdown_event = threading.Event()

# 统计变量
_stats = {"recv": 0, "wraw": 0, "wfilt": 0, "exc": 0, "last_recv_t": 0.0}

def make_writers(out_dir: str) -> Tuple[Tuple[object, csv.writer], Tuple[object, csv.writer]]:
    """创建并打开原始数据和滤波后的数据 CSV 文件，文件名按日期生成"""
    os.makedirs(out_dir, exist_ok=True)
    date_str = time.strftime("%Y%m%d")
    raw_path = os.path.join(out_dir, f"imu_raw_data_{date_str}.csv")
    fil_path = os.path.join(out_dir, f"imu_filtered_data_{date_str}.csv")
    raw_f = open(raw_path, "a", newline="", encoding="utf-8")
    fil_f = open(fil_path, "a", newline="", encoding="utf-8")
    raw_w, fil_w = csv.writer(raw_f), csv.writer(fil_f)

    if raw_f.tell() == 0:
        raw_w.writerow(["Timestamp", "AccX", "AccY", "AccZ", "AsX", "AsY", "AsZ", "HX", "HY", "HZ", "AngX", "AngY", "AngZ"])
        raw_f.flush()
    if fil_f.tell() == 0:
        fil_w.writerow(["Timestamp", "AccX_filt", "AccY_filt", "AccZ_filt", "AsX_filt", "AsY_filt", "AsZ_filt", "Yaw_filt(deg)"])
        fil_f.flush()

    return (raw_f, raw_w), (fil_f, fil_w)

def consumer_thread(raw_writer, raw_file, fil_writer, fil_file):
    """消费线程：从队列获取数据并写入 CSV 文件"""
    while not shutdown_event.is_set():
        try:
            item = sample_q.get(timeout=0.1)
        except Empty:
            continue

        raw_writer.writerow([item["timestamp"], item["acc"][0], item["acc"][1], item["acc"][2], 
                             item["gyr"][0], item["gyr"][1], item["gyr"][2], item["mag"][0], 
                             item["mag"][1], item["mag"][2], item["angles"][0], item["angles"][1], item["angles"][2]])
        _stats["wraw"] += 1

        # 这里可以添加滤波处理，滤波后的数据写入另一个 CSV 文件
        # 例如：
        # filt_data = apply_filter(item)
        # fil_writer.writerow(filt_data)
        _stats["wfilt"] += 1

def parse_args():
    """解析命令行参数"""
    ap = argparse.ArgumentParser(description="IMU实时采集与记录")
    ap.add_argument("--outdir", default=".", help="输出目录")
    return ap.parse_args()

def main():
    """主函数：初始化设备，启动数据采集与记录线程"""
    args = parse_args()
    out_dir = args.outdir
    (raw_f, raw_w), (fil_f, fil_w) = make_writers(out_dir)

    # 启动消费线程
    cons_th = threading.Thread(target=consumer_thread, args=(raw_w, raw_f, fil_w, fil_f), daemon=True)
    cons_th.start()

    # 注册退出信号
    def on_signal(sig, frame):
        print("\n[SYS ] 收到退出信号，准备关闭…")
        shutdown_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # 模拟数据获取
    # 此处调用实际的数据采集函数（比如 imu_interface 或其他）
    try:
        while not shutdown_event.is_set():
            time.sleep(0.5)  # 模拟程序运行
    except KeyboardInterrupt:
        shutdown_event.set()

    cons_th.join()

if __name__ == "__main__":
    main()
