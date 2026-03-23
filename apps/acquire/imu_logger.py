#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/acquire/imu_logger.py

面向“长期运行 / 实时导航定位”的 IMU 记录器：
- 使用统一时间基 (MonoNS + EstNS)
- 自动写入 data/YYYY-MM-DD/imu/

输出两类文件：
1) imu_raw_log_*.csv
   - 明确对齐厂商原始字段集，便于“滤波/定位算法对照”
   - 字段：AccX/Y/Z, AsX/Y/Z, HX/HY/HZ, AngX/Y/Z, Temperature
2) min_imu_tb_*.csv
   - 双时间戳 + Acc/Gyro + YawDeg + EulerDeg（可用于对齐/ESKF/后处理）
"""

from __future__ import annotations

import argparse
import csv
import logging
import os
import signal
import time
from pathlib import Path
from typing import Optional, Tuple, Any

import sys
sys.path.append(str(Path(__file__).resolve().parents[2]))

from uwnav.io.timebase import SensorKind, stamp
from uwnav.io.acquisition_diagnostics import SensorRunDiagnostics
from uwnav.sensors.imu import IMUReader, IMUResult


# =====================================================================
# 工具：目录管理
# =====================================================================

def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]

def default_data_root() -> Path:
    return repo_root() / "data"

def make_day_imu_dir(data_root: Path) -> Path:
    day = time.strftime("%Y-%m-%d")
    d = data_root / day / "imu"
    d.mkdir(parents=True, exist_ok=True)
    return d


# =====================================================================
# 工具：安全转换
# =====================================================================

def _safe_float(x: Any, default: float = 0.0) -> float:
    try:
        return float(x)
    except Exception:
        return default

def _opt3(v) -> Optional[Tuple[float, float, float]]:
    if v is None:
        return None
    try:
        return (float(v[0]), float(v[1]), float(v[2]))
    except Exception:
        return None


# =====================================================================
# RAW 写入器：对齐厂商字段集
# =====================================================================

class RawIMUWriter:
    """
    RAW CSV：严格对齐厂商常见“原始报文字段集”，便于后续对比：
    AccX/Y/Z, AsX/Y/Z, HX/Y/Z, AngX/Y/Z, Temperature
    """

    def __init__(self, out_dir: Path):
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._path = out_dir / f"imu_raw_log_{ts}.csv"
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)

        self._w.writerow([
            "MonoNS", "EstNS", "MonoS", "EstS",
            "t_unix",
            "AccX", "AccY", "AccZ",
            "AsX", "AsY", "AsZ",
            "HX", "HY", "HZ",
            "AngX", "AngY", "AngZ",
            "TemperatureC",
        ])
        self._f.flush()

        logging.info(f"[IMU-RAW] 写入: {self._path}")
        self._n = 0

    def write(self,
              mono_ns: int,
              est_ns: int,
              t_unix: float,
              acc: Tuple[float, float, float],
              gyr: Tuple[float, float, float],
              mag: Optional[Tuple[float, float, float]],
              euler_deg: Optional[Tuple[float, float, float]],
              temperature_c: Optional[float]):

        hx, hy, hz = ("", "", "") if mag is None else (mag[0], mag[1], mag[2])
        ax, ay, az = ("", "", "") if euler_deg is None else (euler_deg[0], euler_deg[1], euler_deg[2])
        temp = "" if temperature_c is None else float(temperature_c)

        self._w.writerow([
            mono_ns, est_ns, mono_ns/1e9, est_ns/1e9,
            t_unix,
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
            hx, hy, hz,
            ax, ay, az,
            temp
        ])

        self._n += 1
        if self._n % 50 == 0:
            try:
                self._f.flush()
            except Exception:
                pass

    def close(self):
        try:
            self._f.flush()
        except Exception:
            pass
        self._f.close()
        logging.info(f"[IMU-RAW] 关闭文件: {self._path}")


# =====================================================================
# MIN 写入器：对齐/滤波用最小表（可扩展）
# =====================================================================

class MinimalIMUWriterTB:
    """
    MIN CSV：用于对齐/融合/ESKF：
    - 双时间戳（MonoNS, EstNS）
    - Acc/Gyro（原始）
    - YawDeg（滤波器输出，可能 None）
    - EulerDeg（来自原始报文 AngX/Y/Z，可能 None）
    """

    def __init__(self, out_dir: Path):
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        self._path = out_dir / f"min_imu_tb_{ts}.csv"
        self._f = open(self._path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)

        self._w.writerow([
            "MonoNS", "EstNS", "MonoS", "EstS",
            "AccX", "AccY", "AccZ",
            "GyroX", "GyroY", "GyroZ",
            "YawDeg",
            "AngX", "AngY", "AngZ",
        ])
        self._f.flush()

        logging.info(f"[IMU-MIN] 写入: {self._path}")
        self._n = 0

    def write(self,
              mono_ns: int,
              est_ns: int,
              acc: Tuple[float, float, float],
              gyr: Tuple[float, float, float],
              yaw_deg: Optional[float],
              euler_deg: Optional[Tuple[float, float, float]]):

        yaw = "" if yaw_deg is None else float(yaw_deg)
        ax, ay, az = ("", "", "") if euler_deg is None else (euler_deg[0], euler_deg[1], euler_deg[2])

        self._w.writerow([
            mono_ns, est_ns, mono_ns/1e9, est_ns/1e9,
            acc[0], acc[1], acc[2],
            gyr[0], gyr[1], gyr[2],
            yaw,
            ax, ay, az
        ])

        self._n += 1
        if self._n % 50 == 0:
            try:
                self._f.flush()
            except Exception:
                pass

    def close(self):
        try:
            self._f.flush()
        except Exception:
            pass
        self._f.close()
        logging.info(f"[IMU-MIN] 关闭文件: {self._path}")


# =====================================================================
# CLI
# =====================================================================

def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="IMU logger for long-running navigation data acquisition (timebase unified)"
    )
    ap.add_argument("--port", default="/dev/ttyUSB0", help="IMU 串口 (默认 /dev/ttyUSB0)")
    ap.add_argument("--baud", type=int, default=230400, help="波特率 (默认 230400)")
    ap.add_argument("--addr", type=lambda x: int(x, 0), default=0x50, help="Modbus 地址（hex，如 0x50）")

    ap.add_argument("--data-root", default=None, help="data 根目录（默认 repo_root/data）")
    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARN", "ERROR"], help="日志等级")
    ap.add_argument("--stat-every", type=float, default=5.0, help="统计打印间隔秒数（0=不打印）")
    return ap.parse_args()


# =====================================================================
# Main
# =====================================================================

def main():
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    data_root = Path(args.data_root).resolve() if args.data_root else default_data_root()
    data_root.mkdir(parents=True, exist_ok=True)
    out_dir = make_day_imu_dir(data_root)
    logging.info(f"[IMU] data_root={data_root}")
    logging.info(f"[IMU] 今日目录={out_dir}")

    diag = SensorRunDiagnostics(out_dir, "imu_capture", "IMU_HWT9073")
    diag.set_meta("port", args.port)
    diag.set_meta("baud", args.baud)
    diag.set_meta("addr", hex(args.addr))
    diag.set_meta("data_root", str(data_root))

    port_path = Path(args.port)
    if not port_path.exists():
        diag.warn_once(
            "imu_serial_path_missing",
            "configured IMU serial path does not exist before open",
            port=args.port,
        )

    raw_writer = RawIMUWriter(out_dir)
    min_writer = MinimalIMUWriterTB(out_dir)
    diag.note_file("imu_raw_csv", raw_writer._path)
    diag.note_file("imu_min_csv", min_writer._path)

    stats = {"raw": 0, "min": 0, "callback_error": 0, "start_t": time.time()}
    stop = {"flag": False}
    runtime_error = {"msg": ""}

    def _on_imu_event(level: str, code: str, message: str, **details):
        diag.bump(f"event_{level}")
        diag.event(level, code, message, **details)

    def imu_callback(res: IMUResult):
        try:
            ts = stamp("imu0", SensorKind.IMU)
            mono_ns = int(ts.host_time_ns)
            est_ns = int(ts.corrected_time_ns)

            acc = (float(res.acc_raw[0]), float(res.acc_raw[1]), float(res.acc_raw[2]))
            gyr = (float(res.gyr_raw[0]), float(res.gyr_raw[1]), float(res.gyr_raw[2]))

            mag = _opt3(res.mag_raw)
            euler_deg = _opt3(res.euler_deg)
            temperature_c = res.temperature_c

            # RAW：严格对齐字段集
            raw_writer.write(
                mono_ns=mono_ns,
                est_ns=est_ns,
                t_unix=_safe_float(res.t_unix, 0.0),
                acc=acc,
                gyr=gyr,
                mag=mag,
                euler_deg=euler_deg,
                temperature_c=temperature_c
            )
            stats["raw"] += 1
            diag.bump("raw_samples")

            # MIN：对齐/融合表
            min_writer.write(
                mono_ns=mono_ns,
                est_ns=est_ns,
                acc=acc,
                gyr=gyr,
                yaw_deg=res.yaw_deg,
                euler_deg=euler_deg
            )
            stats["min"] += 1
            diag.bump("min_samples")

            if res.yaw_deg is None:
                diag.bump("calibrating_samples")
            if mag is None:
                diag.bump("samples_without_mag")
            if temperature_c is None:
                diag.bump("samples_without_temp")
        except Exception as e:
            stats["callback_error"] += 1
            logging.warning(f"[IMU] callback write error: {e}")
            if (stats["callback_error"] == 1) or (stats["callback_error"] % 20 == 0):
                diag.warn(
                    "imu_callback_write_error",
                    "imu callback write failed",
                    error=str(e),
                    callback_error_count=stats["callback_error"],
                )

    logging.info(f"[IMU] 初始化串口 {args.port} @ {args.baud}, Addr={hex(args.addr)}")
    imu = IMUReader(
        port=args.port,
        baud=args.baud,
        addr=args.addr,
        on_result=imu_callback,
        on_event=_on_imu_event,
        csv_dir=None  # 关闭 IMUReader 自带落盘，统一由本脚本管理
    )

    try:
        imu.open()
        imu.start()
        logging.info("[IMU] 启动采集")
    except Exception as e:
        logging.error(f"[IMU] 启动失败: {e}", exc_info=True)
        raw_writer.close()
        min_writer.close()
        diag.finalize(
            "open_failed",
            f"imu open/start failed: {e}",
            imu_stats=imu.stats(),
        )
        return

    def _on_sig(sig, frame):
        logging.info(f"[IMU] 收到信号 {sig}，准备退出…")
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

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
                        f"min={stats['min']} ({min_rate:.1f}/s), imu=({imu.stats()})"
                    )
                    last_stat_t = now
    except KeyboardInterrupt:
        pass
    except Exception as e:
        runtime_error["msg"] = str(e)
        logging.error(f"[IMU] 运行时异常: {e}", exc_info=True)
    finally:
        logging.info("[IMU] 关闭设备…")
        try:
            imu.stop()
        except Exception as e:
            diag.warn("imu_stop_failed", "imu stop failed", error=str(e))

        raw_writer.close()
        min_writer.close()

        imu_stats = imu.stats()
        diag.set_meta("imu_stats", imu_stats)

        if runtime_error["msg"]:
            status = "runtime_error"
            message = f"imu logger runtime error: {runtime_error['msg']}"
        elif stats["raw"] == 0:
            status = "empty_capture"
            message = "imu capture finished without any sample"
            diag.warn("imu_empty_capture", message, port=args.port)
        else:
            status = "ok"
            message = "imu capture finished with data"

        diag.finalize(
            status,
            message,
            raw_count=stats["raw"],
            min_count=stats["min"],
            callback_error_count=stats["callback_error"],
            runtime_s=round(max(0.0, time.time() - stats["start_t"]), 3),
        )
        logging.info("[IMU] logger 完整退出。")


if __name__ == "__main__":
    main()
