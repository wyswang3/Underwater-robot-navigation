#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
apps/tools/dvl_data_verifier.py  (timebase 版)

要点：
- 在 on_raw/on_parsed 回调中统一使用：
      ts = stamp("dvl0", SensorKind.DVL)
      mono_ns = ts.host_time_ns
      est_ns  = ts.corrected_time_ns
- 现有 DVLLogger 继续使用“单列时间戳”——此处喂入 est_s（不改库）
- 另起一个 MinimalSpeedWriterTB，专写带双时间戳的“精简速度表”，
  便于后续与 IMU/DVL 融合只看 MonoNS/EstNS
"""

from __future__ import annotations

import time
import signal
import logging
import argparse
from pathlib import Path
from typing import Optional
import csv
import os

from uwnav.io.timebase import stamp, SensorKind  # ★ 统一时间基（新 API）
from uwnav.drivers.dvl.hover_h1000.io import (
    DVLLogger,
    MinimalSpeedWriter,
    DVLSerialInterface,
)
from uwnav.io.data_paths import get_sensor_outdir


# --------- 带双时间戳的最小速度写手（不改库里的 MinimalSpeedWriter） ---------
class MinimalSpeedWriterTB:
    def __init__(self, out_dir: str, sensor_id: str = "DVL_H1000"):
        os.makedirs(out_dir, exist_ok=True)
        self.sensor_id = sensor_id
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = Path(out_dir) / f"dvl_speed_min_tb_{ts}.csv"
        self.f = open(path, "a", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow([
                "MonoNS", "EstNS", "MonoS", "EstS",
                "SensorID", "Src",
                "East_Vel(m_s)", "North_Vel(m_s)", "Up_Vel(m_s)",
            ])
        logging.info(f"[DVL-SPEED-TB] -> {path}")
        self._n = 0

    def write_if_valid(
        self,
        mono_ns: int,
        est_ns: int,
        src: str,
        ve: Optional[float],
        vn: Optional[float],
        vu: Optional[float],
    ):
        # 三轴都有值才写（融合时更省事）
        if ve is None or vn is None or vu is None:
            return
        mono_s = mono_ns / 1e9
        est_s  = est_ns / 1e9
        self.w.writerow([
            mono_ns, est_ns, mono_s, est_s,
            self.sensor_id, src,
            ve, vn, vu,
        ])
        self._n += 1
        if (self._n % 100) == 0:
            try:
                self.f.flush()
            except Exception:
                pass

    def close(self):
        try:
            self.f.flush()
        except Exception:
            pass
        self.f.close()


# ----------------- CLI -----------------
def parse_args():
    ap = argparse.ArgumentParser(
        description="DVL data verifier/logger (PD6/EPD6, timebase unified)"
    )
    # 串口
    ap.add_argument(
        "--port",
        default="/dev/ttyUSB2",
        help="Serial port (e.g., /dev/ttyUSB2 or COM6)",
    )
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate")
    ap.add_argument(
        "--parity",
        default="N",
        choices=["N", "E", "O"],
        help="Parity: N/E/O",
    )
    ap.add_argument(
        "--stopbits",
        type=float,
        default=1,
        choices=[1, 2],
        help="Stop bits",
    )
    ap.add_argument("--rtscts", action="store_true", help="Enable RTS/CTS")
    ap.add_argument("--dsrdtr", action="store_true", help="Enable DSR/DTR")
    ap.add_argument("--xonxoff", action="store_true", help="Enable XON/XOFF")
    ap.add_argument("--no-dtr", action="store_true", help="Force DTR low (False)")
    ap.add_argument("--no-rts", action="store_true", help="Force RTS low (False)")

    # 命令配置
    ap.add_argument("--df", type=int, default=None, help="Data Format (0=PD6, 6=EPD6)")
    ap.add_argument("--pr", type=int, default=None, help="Ping Rate (Hz)")
    ap.add_argument("--pm", type=int, default=None, help="Average Times")
    ap.add_argument("--st", action="store_true", help="Set UTC time (:ST)")

    # 启停
    ap.add_argument("--send-cs", action="store_true", help="Send CS (start)")
    ap.add_argument(
        "--send-cz",
        action="store_true",
        help="Send CZ (stop) on exit",
    )

    # 运行
    ap.add_argument(
        "--raw-only",
        action="store_true",
        help="Only log raw lines, skip parsing",
    )
    ap.add_argument(
        "--outdir",
        default=None,
        help="Output directory; default=<repo_root>/data",
    )
    ap.add_argument(
        "--no-min-speed",
        action="store_true",
        help="Do not write minimal speed CSV (timebase)",
    )
    ap.add_argument(
        "--log",
        default="INFO",
        choices=["DEBUG", "INFO", "WARN", "ERROR"],
    )
    return ap.parse_args()


def _default_outdir(args_outdir: Optional[str]) -> str:
    # ★ 使用统一路径函数：按日期 + sensor=dvl 分类
    return str(get_sensor_outdir("dvl", args_outdir))

# ----------------- 主程序 -----------------
def main():
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    out_dir = _default_outdir(args.outdir)
    logger = DVLLogger(out_dir)  # 仍旧单列时间戳（这里我们喂 est_s）
    speed_min_tb = None if args.no_min_speed else MinimalSpeedWriterTB(out_dir)

    # ---- 信号处理：抛 KeyboardInterrupt 进入 finally ----
    def _on_signal(sig, frame):
        logging.info("signal received, shutting down...")
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # ---- 回调：统一时间基 ----
    def _on_raw_with_timebase(ts_unused: float, line: str, note: str):
        # 使用 DVL 统一时间基：dvl0 + SensorKind.DVL
        ts = stamp("dvl0", SensorKind.DVL)
        est_s = ts.corrected_time_ns / 1e9
        # 用 est_s 代替库里原本的 time.time()，保持“可读时间线”
        logger.store_raw_line(est_s, line, note)

    def _on_parsed_with_timebase(d):
        # d 是 uwnav.drivers.dvl.hover_h1000.io.DVLData
        logger.store_parsed(d)

        # 另外输出一份带双时间戳的精简速度表（MonoNS/EstNS）
        if speed_min_tb:
            ts = stamp("dvl0", SensorKind.DVL)
            mono_ns = ts.host_time_ns
            est_ns  = ts.corrected_time_ns
            speed_min_tb.write_if_valid(mono_ns, est_ns, d.src, d.ve, d.vn, d.vu)

    dvl = DVLSerialInterface(
        port=args.port,
        baud=args.baud,
        parity=args.parity,
        stopbits=args.stopbits,
        rtscts=args.rtscts,
        dsrdtr=args.dsrdtr,
        xonxoff=args.xonxoff,
        no_dtr=args.no_dtr,
        no_rts=args.no_rts,
        on_raw=_on_raw_with_timebase,
        on_parsed=_on_parsed_with_timebase if not args.raw_only else None,
    )

    if not dvl.start_listening(raw_only=args.raw_only):
        logging.error("start_listening failed, exit.")
        logger.close()
        if speed_min_tb:
            speed_min_tb.close()
        return

    # ========= 安全启动序列：先 CZ 再 CS =========
    try:
        logging.info("[DVL-CMD] CZ (ensure stopped before start)")
        dvl.write_cmd_try("CZ", "")
        time.sleep(1.0)

        logging.info("[DVL-CMD] CS (start ping, ensure already in water)")
        dvl.write_cmd_try("CS", "")
        time.sleep(0.5)

        logging.info("[DVL] start sequence done, DVL should be running.")
    except Exception as e:
        logging.error(f"[DVL] start sequence failed: {e}")
        logging.error("[DVL] for safety, exiting now.")
        dvl.stop_listening()
        logger.close()
        if speed_min_tb:
            speed_min_tb.close()
        return
    # ==========================================

    try:
        # 参数同步（固定16字节优先，失败回退）
        if args.df is not None:
            logging.info(f"[DVL-CMD] DF={args.df}")
            dvl.write_cmd_try("DF", str(args.df))
        if args.pr is not None:
            logging.info(f"[DVL-CMD] PR={args.pr}")
            dvl.write_cmd_try("PR", str(args.pr))
        if args.pm is not None:
            logging.info(f"[DVL-CMD] PM={args.pm}")
            dvl.write_cmd_try("PM", str(args.pm))
        if args.st:
            logging.info("[DVL-CMD] ST (set time)")
            dvl.write_cmd_try("ST", "")

        # 主循环
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        logging.info("[DVL] KeyboardInterrupt, shutting down...")
    finally:
        # ========= 安全关停序列：CZ =========
        try:
            logging.info("[DVL-CMD] CZ (stop ping on exit)")
            dvl.write_cmd_try("CZ", "")
        except Exception as e:
            logging.warning(f"[DVL] CZ send failed: {e}")
        time.sleep(0.1)
        # =====================================

        dvl.stop_listening()
        logger.close()
        if speed_min_tb:
            speed_min_tb.close()
        logging.info(f"bye. stats: {dvl.stats()}")


if __name__ == "__main__":
    main()