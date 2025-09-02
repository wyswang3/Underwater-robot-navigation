#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件名：imu_realtime_pipeline.py

功能概述：
- 从 IMU 设备实时采集数据（通过 device_model.DeviceModel）
- 将采集到的样本以非阻塞方式投递到队列（Queue）
- 消费者线程从队列取样本：写入“原始数据 CSV”，并在 100ms 窗口内做实时滤波（RealTimeIMUFilter），
  将滤波后结果写入另一个 CSV
- 优雅退出：捕获 Ctrl+C / SIGTERM，按顺序停止读取、关闭设备与文件句柄

设计要点：
- 回调只做“轻活”（打包入队），避免阻塞串口读线程、降低丢包风险
- 生产者-消费者解耦：采集与落盘/滤波分离，线程更安全
- 文件名按“日期”命名，天然实现“按日”轮换（与之前脚本一致）
"""

import os
import sys
import time
import csv
import signal
import threading
from collections import deque
from datetime import datetime
from queue import Queue, Empty

import numpy as np

# 依赖你项目中的模块：DeviceModel（串口收发与协议解析）、实时滤波器
from uwnav.drivers.WitHighModbus import device_model
from uwnav.drivers.WitHighModbus.filters import RealTimeIMUFilter

# ========= 配置区域 =========
PORT = "/dev/ttyUSB0"      # 串口端口（Windows 请改为 'COMx'）
BAUD = 115200              # 波特率
ADDR = 0x50                # 设备地址（依厂商协议）
FS   = 50                  # 采样频率 Hz（传给滤波器）
FILTER_CUTOFF = 4.0        # 低通截止频率（Hz）
BATCH_WINDOW_S = 0.10      # 滤波批处理窗口长度（秒），100ms
PRINT_EVERY_N = 25         # 每 N 条打印一行原始数据，防止刷屏
# ==========================

# 退出事件（全局）：各线程据此收尾
shutdown_event = threading.Event()

# 生产者->消费者的队列：容量给足实时性，满了会丢最旧项
sample_q: "Queue[dict]" = Queue(maxsize=2000)


def make_writers(base_dir: str):
    """
    根据当前日期创建/打开两个 CSV：
    - 原始数据：imu_raw_data_YYYYMMDD.csv
    - 滤波后数据：imu_filtered_data_YYYYMMDD.csv
    若是新文件则写入表头；否则追加写入。
    """
    date_str = time.strftime("%Y%m%d")
    raw_path = os.path.join(base_dir, f"imu_raw_data_{date_str}.csv")
    fil_path = os.path.join(base_dir, f"imu_filtered_data_{date_str}.csv")

    try:
        raw_f = open(raw_path, "a", newline="", encoding="utf-8")
        fil_f = open(fil_path, "a", newline="", encoding="utf-8")
    except IOError as e:
        print(f"[ERR ] 打开日志文件失败：{e}")
        sys.exit(1)

    raw_w = csv.writer(raw_f)
    fil_w = csv.writer(fil_f)

    if raw_f.tell() == 0:
        # 原始数据 CSV 表头（含磁场与欧拉角）
        raw_w.writerow([
            "Timestamp",
            "AccX", "AccY", "AccZ",
            "AsX", "AsY", "AsZ",
            "HX", "HY", "HZ",
            "AngX", "AngY", "AngZ"
        ])
        raw_f.flush()
        print(f"[RAW ] 创建原始数据文件: {raw_path}")
    else:
        print(f"[RAW ] 追加原始数据文件: {raw_path}")

    if fil_f.tell() == 0:
        # 滤波后 CSV 表头
        fil_w.writerow([
            "Timestamp",
            "AccX_filt", "AccY_filt", "AccZ_filt",
            "AsX_filt", "AsY_filt", "AsZ_filt",
            "Yaw_filt(deg)"
        ])
        fil_f.flush()
        print(f"[FILT] 创建滤波后数据文件: {fil_path}")
    else:
        print(f"[FILT] 追加滤波后数据文件: {fil_path}")

    return (raw_f, raw_w), (fil_f, fil_w)


def updateData(dev):
    """
    供 DeviceModel 调用的回调函数（运行在串口读线程）。
    —— 只做“轻活”：采样→打包→入队，不做 IO/复杂计算，避免阻塞串口读线程。
    """
    if shutdown_event.is_set():
        return

    ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")  # 可读时间戳
    t = time.time()                                           # 单调时间，用于滤波器

    try:
        # 取 6 轴主数据
        acc = np.array([dev.get("AccX"), dev.get("AccY"), dev.get("AccZ")], dtype=float)
        gyr = np.array([dev.get("AsX"),  dev.get("AsY"),  dev.get("AsZ")], dtype=float)
        # 其余原始字段
        HX, HY, HZ = dev.get("HX"), dev.get("HY"), dev.get("HZ")
        AngX, AngY, AngZ = dev.get("AngX"), dev.get("AngY"), dev.get("AngZ")
    except Exception:
        # 解析失败则跳过该帧
        return

    # 非阻塞入队；若队列满，则丢弃一项最旧数据以保证实时性
    package = {
        "t": t,
        "ts": ts_str,
        "acc": acc,
        "gyr": gyr,
        "HX": HX, "HY": HY, "HZ": HZ,
        "AngX": AngX, "AngY": AngY, "AngZ": AngZ
    }
    try:
        sample_q.put_nowait(package)
    except:
        try:
            sample_q.get_nowait()  # 丢弃最旧
        except Empty:
            pass
        sample_q.put_nowait(package)


def consumer_thread(raw_writer, raw_file, fil_writer, fil_file):
    """
    消费者线程：
    1) 从队列取样本，立即写入“原始 CSV”
    2) 将队列内样本维护在一个 100ms 的滑动窗口中
    3) 让窗口内样本逐个通过实时滤波器，取最后一个有效输出写入“滤波后 CSV”
    """
    rt_filter = RealTimeIMUFilter(fs=FS, cutoff=FILTER_CUTOFF, calibrate=True)
    window: deque = deque()
    printed = 0

    while not shutdown_event.is_set():
        try:
            item = sample_q.get(timeout=0.2)
        except Empty:
            continue

        # 1) 原始 CSV 落盘
        acc, gyr = item["acc"], item["gyr"]
        raw_writer.writerow([
            item["ts"],
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
            item["HX"], item["HY"], item["HZ"],
            item["AngX"], item["AngY"], item["AngZ"],
        ])
        raw_file.flush()

        # 适度打印，避免刷屏
        printed += 1
        if printed % PRINT_EVERY_N == 0:
            print(f"[RAW ] {item['ts']}  Acc={tuple(np.round(acc,3))}  Gyr={tuple(np.round(gyr,3))}")

        # 2) 维护滑动窗口（长度 BATCH_WINDOW_S）
        window.append((item["t"], acc, gyr))
        while window and (item["t"] - window[0][0] > BATCH_WINDOW_S):
            window.popleft()

        # 3) 窗口批处理滤波：从头到尾喂给滤波器，记录最后一个有效输出
        last = None
        for (t, a, g) in window:
            out = rt_filter.process_sample(t, a, g)
            if out is not None:
                last = out

        if last is not None:
            filt_acc, filt_gyr, yaw_deg = last
            fil_writer.writerow([item["ts"], *filt_acc, *filt_gyr, yaw_deg])
            fil_file.flush()


def main():
    # 输出目录：脚本所在目录
    base_dir = os.path.dirname(os.path.abspath(__file__))
    (raw_f, raw_w), (fil_f, fil_w) = make_writers(base_dir)

    # 启动消费者线程（负责落盘+滤波）
    cons_th = threading.Thread(
        target=consumer_thread,
        args=(raw_w, raw_f, fil_w, fil_f),
        daemon=True
    )
    cons_th.start()

    # 信号处理：Ctrl+C 或 SIGTERM 时标记退出
    def on_signal(sig, frame):
        print("\n[SYS ] 收到退出信号，准备关闭…")
        shutdown_event.set()

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    # 初始化与启动设备读取
    dev = device_model.DeviceModel(
        deviceName="IMU实时测试设备",
        portName=PORT,
        baud=BAUD,
        ADDR=ADDR,
        callback_method=updateData
    )

    try:
        dev.openDevice()
        dev.startLoopRead()
        print(f"[SYS ] 运行中：{PORT} @ {BAUD}，按 Ctrl+C 退出")
        # 主线程仅等待退出事件
        while not shutdown_event.is_set():
            time.sleep(0.5)
    except Exception as e:
        print(f"[ERR ] 设备异常：{e}")
    finally:
        # 收尾顺序：停读 -> 关设备 -> 通知消费者结束 -> 关文件
        try:
            dev.stopLoopRead()
        except Exception:
            pass
        try:
            dev.closeDevice()
        except Exception:
            pass

        shutdown_event.set()
        cons_th.join(timeout=2.0)

        try:
            raw_f.flush(); fil_f.flush()
            raw_f.close(); fil_f.close()
        except Exception:
            pass

        print("[SYS ] 已退出。")


if __name__ == "__main__":
    main()
