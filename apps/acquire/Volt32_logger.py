#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/acquire/Volt32_logger.py

面向“长期运行 / 实时导航实验”的电压电流采集记录程序：
- 通过串口读取 Volt32 板（或类似 16/32 通道采集卡）的文本数据
- 使用统一时间基 (uwnav.io.timebase.stamp) 生成 t_s_volt
- 自动写入 data/YYYY-MM-DD/volt/
- 使用 RollingCSVWriter 实现按大小和按天轮换、自动清理旧文件
- 使用 ChannelBuffer 聚合 CH0..CH{N-1}，凑齐一帧再写入

输出 CSV 列（默认）：
    Timestamp, t_s_volt, CH0, CH1, ..., CH{N-1}
"""

from __future__ import annotations

import os
import csv
import time
import signal
import threading
import argparse
from datetime import datetime, timedelta
from pathlib import Path
from typing import List

import sys
from pathlib import Path
# 获取当前脚本的绝对路径
# .parents[0] 是当前目录, .parents[1] 是 apps, .parents[2] 是项目根目录
project_root = Path(__file__).resolve().parents[2]
sys.path.append(str(project_root))

from uwnav.io.timebase import stamp, SensorKind
from uwnav.io.acquisition_diagnostics import SensorRunDiagnostics
from uwnav.io.channel_frames import ChannelFrameBuffer, parse_channel_line
from uwnav.drivers.imu.WitHighModbus.serial_io_tools import SerialReaderThread

# ===================== RollingCSVWriter =====================

class RollingCSVWriter:
    """
    线程安全的 CSV 轮换写入器：
    - 跨天或达大小阈值即换新文件
    - 支持按行自动 flush
    - 自动清理 keep_days 之前的旧文件
    """
    def __init__(self, out_dir: Path, prefix: str, header: List[str],
                 rotate_mb: int = 50, keep_days: int = 7, autoflush_every: int = 1):
        self.out_dir = str(out_dir)
        self.prefix = prefix
        self.header = header
        self.rotate_mb = rotate_mb
        self.keep_days = keep_days
        self.autoflush_every = max(0, int(autoflush_every))

        os.makedirs(self.out_dir, exist_ok=True)
        self._file = None
        self._csv = None
        self._open_date = None
        self._lock = threading.Lock()
        self._row_counter = 0
        self.current_path = None
        self._open_new_file()

    def _build_filepath(self, now: datetime) -> str:
        name = f"{self.prefix}_{now.strftime('%Y%m%d_%H%M%S')}.csv"
        return os.path.join(self.out_dir, name)

    def _open_new_file(self):
        if self._file:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass
        now = datetime.now()
        path = self._build_filepath(now)
        self._file = open(path, "a", newline="", encoding="utf-8")
        self._csv = csv.writer(self._file)
        self._csv.writerow(self.header)
        self._file.flush()
        self._open_date = now.date()
        self.current_path = path
        self._row_counter = 0
        self._cleanup_old_files()
        print(f"[VOLT] 新日志文件：{path}")

    def _need_rotate_nolock(self) -> bool:
        # 跨天
        if datetime.now().date() != self._open_date:
            return True
        # 按大小轮换
        if self.rotate_mb and self.rotate_mb > 0 and self._file:
            try:
                if os.fstat(self._file.fileno()).st_size >= self.rotate_mb * 1024 * 1024:
                    return True
            except Exception:
                pass
        return False

    def _cleanup_old_files(self):
        if not self.keep_days or self.keep_days <= 0:
            return
        cutoff = datetime.now() - timedelta(days=self.keep_days)
        for fname in os.listdir(self.out_dir):
            if not (fname.startswith(self.prefix + "_") and fname.endswith(".csv")):
                continue
            fpath = os.path.join(self.out_dir, fname)
            try:
                if datetime.fromtimestamp(os.path.getmtime(fpath)) < cutoff:
                    os.remove(fpath)
                    print(f"[VOLT] 清理旧文件：{fname}")
            except Exception:
                pass

    def writerow(self, row: List[str]):
        with self._lock:
            if self._need_rotate_nolock():
                self._open_new_file()
            self._csv.writerow(row)
            self._row_counter += 1
            if self.autoflush_every and (self._row_counter % self.autoflush_every == 0):
                try:
                    self._file.flush()
                except Exception:
                    pass

    def flush(self):
        with self._lock:
            try:
                self._file.flush()
            except Exception:
                pass

    def close(self):
        with self._lock:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass


# ===================== ChannelBuffer =====================

class ChannelBuffer:
    """
    聚合 CH0..CH{N-1} 的一帧数据，齐则返回一行，否则返回 None。

    假定串口行格式类似：
        CH0:  1.234
        CH1: -0.321
        ...
    或其它“首 token 含 CHx:，末 token 为数值”的形式。
    """
    def __init__(self, n_channels: int):
        self.n = n_channels
        self._buf = {f"CH{i}": None for i in range(self.n)}
        self._lock = threading.Lock()

    @staticmethod
    def parse_line(line: str):
        # 兼容“多空格/Tab”分隔
        if not line:
            return None
        s = line.rstrip("\r\n").strip()
        if not s:
            return None
        toks = s.split()
        if len(toks) < 2:
            return None
        channel_part, value_part = toks[0], toks[-1]
        if ":" not in channel_part:
            return None
        ch_key, _ = channel_part.split(":", 1)
        ch_key = ch_key.strip().upper()
        if not ch_key.startswith("CH"):
            return None
        return ch_key, value_part

    def update(self, ch_key: str, value_str: str):
        if ch_key not in self._buf:
            return None
        with self._lock:
            self._buf[ch_key] = value_str
            # 所有通道齐全则返回一帧
            if all(self._buf[f"CH{i}"] is not None for i in range(self.n)):
                row = [self._buf[f"CH{i}"] for i in range(self.n)]
                for i in range(self.n):
                    self._buf[f"CH{i}"] = None
                return row
        return None


# ===================== util: 目录与参数 =====================

def repo_root() -> Path:
    here = Path(__file__).resolve()
    return here.parents[2]   # apps/acquire → apps → repo_root


def default_data_root() -> Path:
    return repo_root() / "data"


def make_day_volt_dir(data_root: Path) -> Path:
    day = time.strftime("%Y-%m-%d")
    d = data_root / day / "volt"
    d.mkdir(parents=True, exist_ok=True)
    return d


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Volt32 logger (multi-channel voltage/current board, timebase unified)"
    )
    ap.add_argument("--port", default="/dev/ttyUSB0",
                    help="串口设备，例如 /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200,
                    help="波特率，默认 115200")
    ap.add_argument("--channels", type=int, default=16,
                    help="通道数 N_CHANNELS，默认 16")
    ap.add_argument("--data-root", default=None,
                    help="data 根目录，默认=<repo_root>/data")
    ap.add_argument("--rotate-mb", type=int, default=50,
                    help="单文件轮换大小阈值（MB），默认 50MB")
    ap.add_argument("--keep-days", type=int, default=7,
                    help="保留天数，默认 7 天")
    ap.add_argument("--flush-sec", type=int, default=60,
                    help="周期性 flush 间隔（秒），0=不周期 flush")
    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARN", "ERROR"],
                    help="日志等级")
    ap.add_argument("--debug-raw-sniff", type=int, default=20,
                    help="仅打印前 N 行原始数据（0=关闭），默认 20")
    ap.add_argument("--debug-raw-echo", action="store_true",
                    help="打印所有原始行（高频会刷屏）")
    ap.add_argument("--autoflush-every", type=int, default=1,
                    help="每写入多少帧自动 flush（默认 1）")
    ap.add_argument("--stat-every", type=float, default=5.0,
                    help="统计打印间隔（秒），0=不打印")
    return ap.parse_args()


# ===================== 主流程 =====================

def main():
    args = parse_args()

    import logging
    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # data 目录
    data_root = Path(args.data_root).resolve() if args.data_root else default_data_root()
    data_root.mkdir(parents=True, exist_ok=True)
    day_volt_dir = make_day_volt_dir(data_root)
    logging.info(f"[VOLT] data_root = {data_root}")
    logging.info(f"[VOLT] 今日目录 = {day_volt_dir}")

    diag = SensorRunDiagnostics(day_volt_dir, "volt_capture", "VOLT32")
    diag.set_meta("port", args.port)
    diag.set_meta("baud", args.baud)
    diag.set_meta("channels", args.channels)
    diag.set_meta("data_root", str(data_root))

    if not Path(args.port).exists():
        diag.warn_once(
            "volt_serial_path_missing",
            "configured Volt32 serial path does not exist before open",
            port=args.port,
        )

    n_channels = int(args.channels)

    # === 关键修改 1：时间字段与 IMU/DVL 对齐 ===
    # 统一使用：MonoNS, EstNS, MonoS, EstS + CH0..CH{N-1}
    header = [
        "MonoNS", "EstNS", "MonoS", "EstS",
    ] + [f"CH{i}" for i in range(n_channels)]

    writer = RollingCSVWriter(
        out_dir=day_volt_dir,
        prefix="motor_data",
        header=header,
        rotate_mb=args.rotate_mb,
        keep_days=args.keep_days,
        autoflush_every=args.autoflush_every,
    )
    diag.note_file("motor_data_csv", writer.current_path)

    shutdown = threading.Event()
    frame_buf = ChannelFrameBuffer(n_channels=n_channels)
    frames_written = {"n": 0}
    sniff_count = {"n": 0}
    parse_error_count = {"n": 0}
    runtime_error = {"msg": ""}

    DEBUG_RAW_SNIFF = max(0, int(args.debug_raw_sniff))
    DEBUG_RAW_ECHO = bool(args.debug_raw_echo)
    FLUSH_SEC = max(0, int(args.flush_sec))
    STAT_EVERY = float(args.stat_every)
    allowed_units = {"", "V", "A", "mV", "mA"}

    # ---- 串口回调 ----
    def on_serial_line(payload: str):
        # 调试输出部分原始行
        if DEBUG_RAW_ECHO or (DEBUG_RAW_SNIFF and sniff_count["n"] < DEBUG_RAW_SNIFF):
            print(f"[RAW ] {repr(payload)}")
            sniff_count["n"] += 1

        parsed = parse_channel_line(payload)
        if parsed is None:
            diag.bump("malformed_lines")
            parse_error_count["n"] += 1
            if (parse_error_count["n"] == 1) or (parse_error_count["n"] % 20 == 0):
                diag.warn(
                    "volt_malformed_line",
                    "serial line did not match CHx:value format",
                    raw_line=payload,
                    malformed_count=parse_error_count["n"],
                )
            return

        if parsed.index >= n_channels:
            diag.bump("out_of_range_channels")
            diag.warn_once(
                "volt_channel_out_of_range",
                "received channel index beyond configured channel count",
                channel=parsed.channel,
                configured_channels=n_channels,
            )
            return

        diag.bump("channel_lines")
        if parsed.numeric_value is None:
            diag.bump("non_numeric_values")
            diag.warn(
                "volt_non_numeric_value",
                "channel line carried a non-numeric payload",
                channel=parsed.channel,
                raw_value=parsed.raw_value,
            )
        if parsed.unit not in allowed_units:
            diag.bump("unexpected_units")
            diag.warn_once(
                "volt_unexpected_unit",
                "channel line used an unexpected engineering unit",
                channel=parsed.channel,
                unit=parsed.unit,
            )
        if parsed.unit:
            diag.bump(f"unit_{parsed.unit}")

        row_values = frame_buf.update(parsed)
        if row_values is None:
            return

        # 一帧齐全 → 写入 CSV
        ts = stamp("volt0", SensorKind.OTHER)
        mono_ns = ts.host_time_ns
        est_ns = ts.corrected_time_ns
        mono_s = mono_ns / 1e9
        est_s = est_ns / 1e9

        writer.writerow([
            mono_ns,
            est_ns,
            mono_s,
            est_s,
            *row_values,
        ])
        frames_written["n"] += 1
        diag.bump("frames_written")
        diag.note_file("motor_data_csv", writer.current_path)

    # ---- 串口线程 ----
    serial_th = SerialReaderThread(
        port=args.port,
        baudrate=args.baud,
        shutdown_event=shutdown,
        data_callback=on_serial_line,
        line_mode=True,
        encoding="utf-8",
        eol=b"\n",
    )
    serial_th.start()
    logging.info(f"[VOLT] 串口已启动：{args.port} @ {args.baud}")

    time.sleep(0.2)
    if not serial_th.is_alive():
        writer.flush()
        writer.close()
        diag.finalize(
            "open_failed",
            "Volt32 serial reader exited immediately after start",
            frames_written=frames_written["n"],
        )
        logging.error("[VOLT] 串口线程启动后立即退出。")
        return

    # ---- 周期性 flush 线程（可选） ----
    def flush_loop():
        while not shutdown.wait(timeout=FLUSH_SEC if FLUSH_SEC > 0 else 1e9):
            try:
                writer.flush()
                logging.debug("[VOLT] 周期性刷新完成")
            except Exception as e:
                logging.warning(f"[VOLT] 周期性刷新失败：{e}")
                diag.warn("volt_periodic_flush_failed", "volt periodic flush failed", error=str(e))

    if FLUSH_SEC > 0:
        flush_th = threading.Thread(target=flush_loop, daemon=True)
        flush_th.start()
    else:
        flush_th = None

    # ---- 信号与主循环 ----
    def _signal_handler(sig, frame):
        logging.info(f"[VOLT] 收到信号 {sig}，准备退出…")
        shutdown.set()

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    start_t = time.time()
    last_stat_t = start_t

    try:
        while not shutdown.is_set():
            time.sleep(0.2)
            if STAT_EVERY > 0:
                now = time.time()
                if now - last_stat_t >= STAT_EVERY:
                    dt = now - start_t
                    rate = frames_written["n"] / dt if dt > 0 else 0.0
                    logging.info(f"[STAT] frames={frames_written['n']} ({rate:.1f} fps)")
                    last_stat_t = now
    except KeyboardInterrupt:
        shutdown.set()
    except Exception as e:
        runtime_error["msg"] = str(e)
        logging.error(f"[VOLT] 运行时异常：{e}", exc_info=True)

    # ---- 收尾 ----
    logging.info("[VOLT] 正在收尾…")
    serial_th.join(timeout=2.0)
    if flush_th is not None:
        flush_th.join(timeout=2.0)
    writer.flush()
    writer.close()
    diag.note_file("motor_data_csv", writer.current_path)
    logging.info(
        f"[VOLT] 总写入帧数：{frames_written['n']}  最后文件：{writer.current_path}"
    )

    if runtime_error["msg"]:
        status = "runtime_error"
        message = f"volt logger runtime error: {runtime_error['msg']}"
    elif frames_written["n"] == 0:
        status = "empty_capture"
        message = "volt capture finished without a complete frame"
        diag.warn("volt_empty_capture", message, port=args.port)
    else:
        status = "ok"
        message = "volt capture finished with data"

    diag.finalize(
        status,
        message,
        frames_written=frames_written["n"],
        malformed_lines=diag.bump("malformed_lines", 0),
        runtime_s=round(max(0.0, time.time() - start_t), 3),
    )
    logging.info("[VOLT] 已退出。")

if __name__ == "__main__":
    main()
