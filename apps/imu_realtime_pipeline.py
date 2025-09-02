#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
imu_realtime_pipeline.py (230400 + 调试增强)
"""

import os, sys, time, csv, signal, argparse, threading
from collections import deque
from datetime import datetime
from queue import Queue, Empty
from typing import Tuple
import numpy as np

# 兼容两种导入布局
try:
    from uwnav.drivers.WitHighModbus import device_model
    from uwnav.drivers.WitHighModbus.filters import RealTimeIMUFilter
except ModuleNotFoundError:
    import device_model  # type: ignore
    from filters import RealTimeIMUFilter  # type: ignore

shutdown_event = threading.Event()
sample_q: "Queue[dict]" = Queue(maxsize=4000)

# ---- 调试开关 ----
DEBUG_PRINT_EVERY = 25   # 每 N 条样本打印一次概览；0 关闭
WARN_IF_IDLE_SEC  = 3.0  # N 秒没收到样本则报警
FLUSH_PERIOD_SEC  = 0.5  # 定期刷盘周期
# -------------------

def make_writers(out_dir: str) -> Tuple[Tuple[object, csv.writer], Tuple[object, csv.writer]]:
    os.makedirs(out_dir, exist_ok=True)
    date_str = time.strftime("%Y%m%d")
    raw_path = os.path.join(out_dir, f"imu_raw_data_{date_str}.csv")
    fil_path = os.path.join(out_dir, f"imu_filtered_data_{date_str}.csv")
    raw_f = open(raw_path, "a", newline="", encoding="utf-8")
    fil_f = open(fil_path, "a", newline="", encoding="utf-8")
    raw_w, fil_w = csv.writer(raw_f), csv.writer(fil_f)
    if raw_f.tell() == 0:
        raw_w.writerow(["Timestamp","AccX","AccY","AccZ","AsX","AsY","AsZ","HX","HY","HZ","AngX","AngY","AngZ"])
        raw_f.flush(); print(f"[RAW ] 创建: {raw_path}")
    else:
        print(f"[RAW ] 追加: {raw_path}")
    if fil_f.tell() == 0:
        fil_w.writerow(["Timestamp","AccX_filt","AccY_filt","AccZ_filt","AsX_filt","AsY_filt","AsZ_filt","Yaw_filt(deg)"])
        fil_f.flush(); print(f"[FILT] 创建: {fil_path}")
    else:
        print(f"[FILT] 追加: {fil_path}")
    return (raw_f, raw_w), (fil_f, fil_w)

# 统计
_stats = {"recv":0, "wraw":0, "wfilt":0, "exc":0, "last_recv_t":0.0}

def updateData(dev):
    """DeviceModel 回调：采样→入队；打印异常细节便于排查。"""
    if shutdown_event.is_set():
        return
    ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
    t = time.time()
    try:
        acc = np.array([dev.get("AccX"), dev.get("AccY"), dev.get("AccZ")], dtype=float)
        gyr = np.array([dev.get("AsX"),  dev.get("AsY"),  dev.get("AsZ")],  dtype=float)
        HX, HY, HZ      = dev.get("HX"),   dev.get("HY"),   dev.get("HZ")
        AngX, AngY, AngZ= dev.get("AngX"), dev.get("AngY"), dev.get("AngZ")
    except Exception as e:
        _stats["exc"] += 1
        # 打印一次关键信息帮助定位字段名/解析问题
        if _stats["exc"] <= 5 or _stats["exc"] % 100 == 0:
            try:
                # 尝试窥探可用键
                keys = getattr(dev, "data", {}).keys() if hasattr(dev, "data") else "unknown"
                print(f"[WARN] 回调取值异常: {e} | 可用键: {keys}")
            except Exception:
                print(f"[WARN] 回调取值异常: {e}")
        return

    pkg = {"t":t, "ts":ts_str, "acc":acc, "gyr":gyr,
           "HX":HX, "HY":HY, "HZ":HZ, "AngX":AngX, "AngY":AngY, "AngZ":AngZ}
    try:
        sample_q.put_nowait(pkg)
        _stats["recv"] += 1
        _stats["last_recv_t"] = t
        if DEBUG_PRINT_EVERY and (_stats["recv"] % DEBUG_PRINT_EVERY == 0):
            print(f"[RECV] {_stats['recv']}  q={sample_q.qsize()}  acc={tuple(np.round(acc,3))} gyr={tuple(np.round(gyr,3))}")
    except:
        try: sample_q.get_nowait()
        except Empty: pass
        sample_q.put_nowait(pkg)

def consumer_thread(raw_writer, raw_file, fil_writer, fil_file, fs:int, cutoff:float, window_s:float):
    rt_filter = RealTimeIMUFilter(fs=fs, cutoff=cutoff, calibrate=True)
    window: deque = deque()
    last_flush = time.time()
    last_warn  = 0.0

    while not shutdown_event.is_set():
        # 周期性刷盘（与是否取到样本无关）
        now = time.time()
        if now - last_flush >= FLUSH_PERIOD_SEC:
            try: raw_file.flush(); fil_file.flush()
            except Exception: pass
            last_flush = now

        # 没样本时顺带做“长期无样本”告警
        if _stats["last_recv_t"] and (now - _stats["last_recv_t"] > WARN_IF_IDLE_SEC) and (now - last_warn > 1.0):
            print(f"[WARN] {_stats['recv']} 样本后 {_stats['last_recv_t'] and (now - _stats['last_recv_t']):.1f}s 未收到新数据，检查端口/波特率/设备模式")
            last_warn = now

        try:
            item = sample_q.get(timeout=0.1)
        except Empty:
            continue

        acc, gyr = item["acc"], item["gyr"]
        raw_writer.writerow([item["ts"], acc[0],acc[1],acc[2], gyr[0],gyr[1],gyr[2],
                             item["HX"],item["HY"],item["HZ"], item["AngX"],item["AngY"],item["AngZ"]])
        _stats["wraw"] += 1

        window.append((item["t"], acc, gyr))
        while window and (item["t"] - window[0][0] > window_s):
            window.popleft()

        last = None
        for (t, a, g) in window:
            out = rt_filter.process_sample(t, a, g)
            if out is not None: last = out
        if last is not None:
            fa, fg, yaw = last
            fil_writer.writerow([item["ts"], *fa, *fg, yaw])
            _stats["wfilt"] += 1

def parse_args():
    dft_port = "COM5" if os.name == "nt" else "/dev/ttyUSB1"
    ap = argparse.ArgumentParser(description="IMU realtime pipeline (230400)")
    ap.add_argument("--port", default=dft_port)
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--addr", type=lambda x:int(x,0), default=0x50, help="设备地址(十六进制可用 0x50)")
    ap.add_argument("--fs", type=int, default=50)
    ap.add_argument("--cutoff", type=float, default=4.0)
    ap.add_argument("--window", type=float, default=0.10)
    ap.add_argument("--outdir", default=None)
    return ap.parse_args()

def main():
    args = parse_args()
    out_dir = args.outdir or os.path.dirname(os.path.abspath(__file__))
    (raw_f, raw_w), (fil_f, fil_w) = make_writers(out_dir)

    cons_th = threading.Thread(target=consumer_thread,
        args=(raw_w, raw_f, fil_w, fil_f, args.fs, args.cutoff, args.window),
        daemon=True)
    cons_th.start()

    def on_signal(sig, frame):
        print("\n[SYS ] 收到退出信号，准备关闭…")
        shutdown_event.set()
    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    dev = device_model.DeviceModel(
        deviceName="IMU实时测试设备",
        portName=args.port,
        baud=args.baud,        # 确保 device_model 内部使用传入的 baud 初始化串口
        ADDR=args.addr,
        callback_method=updateData
    )

    try:
        dev.openDevice()
        dev.startLoopRead()
        print(f"[SYS ] 运行中：{args.port} @ {args.baud}  fs={args.fs}Hz cutoff={args.cutoff}Hz  (Ctrl+C 退出)")
        # 运行时持续打印“没有样本”的告警，有助定位
        while not shutdown_event.is_set():
            time.sleep(0.5)
    except Exception as e:
        print(f"[ERR ] 设备异常：{e}")
    finally:
        try: dev.stopLoopRead()
        except Exception: pass
        try: dev.closeDevice()
        except Exception: pass

        shutdown_event.set()
        cons_th.join(timeout=2.0)
        try: raw_f.flush(); fil_f.flush()
        except Exception: pass
        raw_f.close(); fil_f.close()
        print(f"[STAT] recv={_stats['recv']} wraw={_stats['wraw']} wfilt={_stats['wfilt']} exc={_stats['exc']}")
        print("[SYS ] 已退出。")

if __name__ == "__main__":
    main()

