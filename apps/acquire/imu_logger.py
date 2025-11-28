#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/acquire/imu_logger.py

面向“长期运行 / 实时导航定位”的 IMU 记录器：
- 使用统一时间基 (MonoNS + EstNS)
- 自动写入 data/YYYY-MM-DD/imu/
- 输出两类文件：
    1) raw_imu_log_*.csv        → 原始 IMU 报文（易排障）
    2) min_imu_tb_*.csv         → 双时间戳 + Acc + Gyro（用于 ESKF / 后处理）

适配说明：
- 底层驱动已切换至 uwnav.sensors.imu.IMUReader
- 数据源使用 acc_raw/gyr_raw 以保持原始观测特性
"""

from __future__ import annotations

import argparse
import logging
import signal
import time
import csv
import os
from pathlib import Path
from typing import Optional

# 确保能找到 uwnav (如果未设置 PYTHONPATH)
import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from uwnav.io.timebase import SensorKind, stamp
# [修改 1] 导入新的类名
from uwnav.sensors.imu import IMUReader, IMUResult


# =====================================================================
# 最小 IMU 表：用于数据对齐与后处理
# =====================================================================

class MinimalIMUWriterTB:
    """
    写入最简版 IMU 数据（供 ESKF 和对齐使用）：
    列：
        MonoNS, EstNS, MonoS, EstS,
        AccX, AccY, AccZ,
        GyroX, GyroY, GyroZ
    """

    def __init__(self, out_dir: Path):
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._path = out_dir / f"min_imu_tb_{ts}.csv"
        self._f = open(self._path, "a", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)

        if self._f.tell() == 0:
            self._w.writerow([
                "MonoNS", "EstNS", "MonoS", "EstS",
                "AccX", "AccY", "AccZ",
                "GyroX", "GyroY", "GyroZ",
            ])

        logging.info(f"[IMU-MIN] 写入: {self._path}")
        self._n = 0

    def write(self, mono_ns, est_ns, acc, gyr):
        self._w.writerow([
            mono_ns, est_ns, mono_ns/1e9, est_ns/1e9,
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
        ])
        self._n += 1
        if self._n % 50 == 0:
            try:
                self._f.flush()
            except:
                pass

    def close(self):
        try:
            self._f.flush()
        except:
            pass
        self._f.close()
        logging.info(f"[IMU-MIN] 关闭文件: {self._path}")


# =====================================================================
# 工具：目录管理
# =====================================================================

def repo_root() -> Path:
    here = Path(__file__).resolve()
    return here.parents[2]   # apps/acquire → apps → repo_root


def default_data_root() -> Path:
    return repo_root() / "data"


def make_day_imu_dir(data_root: Path) -> Path:
    day = time.strftime("%Y-%m-%d")
    d = data_root / day / "imu"
    d.mkdir(parents=True, exist_ok=True)
    return d


# =====================================================================
# 命令行参数
# =====================================================================

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="IMU logger for long-running navigation data acquisition (timebase unified)"
    )
    ap.add_argument("--port", default="/dev/ttyUSB1", help="IMU 串口 (默认 /dev/ttyUSB1)")
    ap.add_argument("--baud", type=int, default=230400, help="波特率 (默认 230400)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x50,
                    help="Modbus 地址（hex，如 0x50）")

    ap.add_argument("--data-root", default=None, help="data 根目录（默认 repo_root/data）")
    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARN", "ERROR"], help="日志等级")
    ap.add_argument("--stat-every", type=float, default=5.0,
                    help="统计打印间隔秒数（0=不打印）")
    return ap.parse_args()


# =====================================================================
# 主程序
# =====================================================================

def main():
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # data 根目录
    data_root = Path(args.data_root).resolve() if args.data_root else default_data_root()
    data_root.mkdir(parents=True, exist_ok=True)
    day_imu_dir = make_day_imu_dir(data_root)
    logging.info(f"[IMU] data_root={data_root}")
    logging.info(f"[IMU] 今日目录={day_imu_dir}")

    # 打开原始日志文件
    ts = time.strftime("%Y%m%d_%H%M%S")
    raw_path = day_imu_dir / f"imu_raw_log_{ts}.csv"
    raw_f = open(raw_path, "a", newline="", encoding="utf-8")
    raw_w = csv.writer(raw_f)
    raw_w.writerow([
        "EstS",
        "AccX", "AccY", "AccZ",
        "GyroX", "GyroY", "GyroZ",
        "Temperature",
        "FrameType"
    ])
    logging.info(f"[IMU-RAW] 写入: {raw_path}")

    # 最小表 writer
    min_writer = MinimalIMUWriterTB(day_imu_dir)

    # 统计
    stats = {"raw": 0, "min": 0, "start_t": time.time()}

    # ========== [修改 2] 回调适配 ==========
    # 参数类型改为 IMUResult
    def imu_callback(res: IMUResult):
        ts = stamp("imu0", SensorKind.IMU)
        
        mono_ns = ts.host_time_ns
        est_ns  = ts.corrected_time_ns
        est_s   = est_ns / 1e9

        # [修改 3] 字段映射
        # IMUResult 包含 acc_raw(原始) 和 acc_filt(滤波)
        # 对于 logger，通常记录 raw 数据供后续处理
        acc = res.acc_raw
        gyr = res.gyr_raw
        
        # 新驱动暂未提供温度和帧类型，填默认值保持 CSV 格式兼容
        temp = 0.0 
        frame_type = "N/A"

        # 写原始 log
        raw_w.writerow([
            est_s,
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
            temp,
            frame_type
        ])
        stats["raw"] += 1

        # 写最小表
        min_writer.write(mono_ns, est_ns, acc, gyr)
        stats["min"] += 1

    # ========== [修改 4] 初始化 IMUReader ==========
    logging.info(f"[IMU] 初始化串口 {args.port} @ {args.baud}, Addr={hex(args.addr)}")
    imu = IMUReader(
        port=args.port,
        baud=args.baud,
        addr=args.addr,          # 参数名变更为 addr
        on_result=imu_callback,  # 参数名变更为 on_result
        csv_dir=None             # 关闭驱动自带的 CSV 落盘，使用本脚本的逻辑
    )

    # [修改 5] 方法名变更 open_device -> open
    imu.open()
    imu.start()
    logging.info("[IMU] 启动采集")

    # 信号处理
    stop = {"flag": False}

    def _on_sig(sig, frame):
        logging.info(f"[IMU] 收到信号 {sig}，准备退出…")
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    # 主循环
    last_stat_t = time.time()
    try:
        while not stop["flag"]:
            time.sleep(0.2)
            if args.stat_every > 0:
                now = time.time()
                if now - last_stat_t >= args.stat_every:
                    dt = now - stats["start_t"]
                    raw_rate = stats["raw"] / dt if dt > 0 else 0.0
                    min_rate = stats["min"] / dt if dt > 0 else 0.0
                    logging.info(
                        f"[STAT] raw={stats['raw']} ({raw_rate:.1f}/s), "
                        f"min={stats['min']} ({min_rate:.1f}/s)"
                    )
                    last_stat_t = now
    except KeyboardInterrupt:
        pass
    except Exception as e:
        logging.error(f"[IMU] 运行时异常: {e}", exc_info=True)
    finally:
        logging.info("[IMU] 关闭设备…")
        # [修改 6] 方法名变更 close_device -> close
        imu.stop() # stop 内部会调用 close
        raw_f.close()
        min_writer.close()
        logging.info("[IMU] logger 完整退出。")


if __name__ == "__main__":
    main()
