#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dvl_logger.py
-------------
实时读取 DVL 数据并记录到 CSV 文件中。

功能：
1) 从队列获取 DVL 原始数据；
2) 将原始数据保存为 CSV 文件；
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

# 假设 dvl_hover_h1000.py 中有 DVLDevice 类
from uwnav.sensors.dvl_hover_h1000 import DVLDevice

# 共享队列
sample_q = Queue(maxsize=4000)
shutdown_event = threading.Event()

# 统计变量
_stats = {"recv": 0, "wraw": 0, "exc": 0, "last_recv_t": 0.0}

def make_writers(out_dir: str) -> Tuple[object, csv.writer]:
    """创建并打开原始数据 CSV 文件，文件名按时间生成"""
    os.makedirs(out_dir, exist_ok=True)
    date_str = time.strftime("%Y%m%d%H%M%S")  # 文件名带上具体时间
    raw_path = os.path.join(out_dir, f"dvl_raw_data_{date_str}.csv")
    raw_f = open(raw_path, "a", newline="", encoding="utf-8")
    raw_w = csv.writer(raw_f)

    if raw_f.tell() == 0:
        raw_w.writerow(["Timestamp", "East_Vel", "North_Vel", "Up_Vel", "Status", "Raw_Frame"])
        raw_f.flush()

    return raw_f, raw_w

def consumer_thread(raw_writer, raw_file):
    """消费线程：从队列获取数据并写入 CSV 文件"""
    while not shutdown_event.is_set():
        try:
            item = sample_q.get(timeout=0.1)
        except Empty:
            continue

        # 保存原始数据
        raw_writer.writerow([item["timestamp"], item["east_vel"], item["north_vel"], item["up_vel"], item["status"]])
        _stats["wraw"] += 1

def parse_args():
    """解析命令行参数"""
    ap = argparse.ArgumentParser(description="DVL实时采集与记录")
    ap.add_argument("--outdir", default="./data", help="输出目录")
    return ap.parse_args()

def main():
    """主函数：初始化设备，启动数据采集与记录线程"""
    args = parse_args()
    out_dir = args.outdir
    raw_f, raw_w = make_writers(out_dir)

    # 启动 DVL 数据采集程序（dvl_hover_h1000.py）
    dvl_device = DVLDevice(device_type="ethernet", ip="192.168.2.102", data_port=10001, callback_method=dvl_device.add_to_queue)
    dvl_device.start_listening()

    # 启动消费线程
    cons_th = threading.Thread(target=consumer_thread, args=(raw_w, raw_f), daemon=True)
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

if __name__ == "__main__":
    main()
