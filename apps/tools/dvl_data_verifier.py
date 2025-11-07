#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
apps/tools/dvl_data_verifier.py

职责：
- 解析命令行参数（Windows/Unix 通用）
- 组合库：DVLSerialInterface + DVLLogger (+ MinimalSpeedWriter 可选)
- 发送 DF/PR/PM/ST/CS/CZ
- 同时写三份（可选第三份精简速度表）
"""

from __future__ import annotations

import time
import signal
import logging
import argparse
from pathlib import Path
from typing import Optional

from uwnav.drivers.dvl.hover_h1000.io import (
    DVLLogger, MinimalSpeedWriter, DVLSerialInterface
)

def parse_args():
    ap = argparse.ArgumentParser(description="DVL data verifier/logger (PD6/EPD6)")
    # 串口
    ap.add_argument("--port", default="/dev/ttyUSB1", help="Serial port (e.g., /dev/ttyUSB0 or COM6)")
    ap.add_argument("--baud", type=int, default=115200, help="Baudrate")
    ap.add_argument("--parity", default="N", choices=["N", "E", "O"], help="Parity: N/E/O")
    ap.add_argument("--stopbits", type=float, default=1, choices=[1, 2], help="Stop bits")
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
    ap.add_argument("--send-cz", action="store_true", help="Send CZ (stop) on exit")

    # 运行
    ap.add_argument("--raw-only", action="store_true", help="Only log raw lines, skip parsing")
    ap.add_argument("--outdir", default=None, help="Output directory; default=<repo_root>/data")
    ap.add_argument("--no-min-speed", action="store_true", help="Do not write minimal speed CSV")
    ap.add_argument("--log", default="INFO", choices=["DEBUG", "INFO", "WARN", "ERROR"])
    return ap.parse_args()


def _default_outdir(args_outdir: Optional[str]) -> str:
    if args_outdir:
        return args_outdir
    return str(Path(__file__).resolve().parents[2] / "data")


def main():
    args = parse_args()
    logging.basicConfig(
        level=getattr(logging, args.log),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    out_dir = _default_outdir(args.outdir)
    logger = DVLLogger(out_dir)

    # 可选：仅有效速度（m/s）精简表
    speed_min = None if args.no_min_speed else MinimalSpeedWriter(out_dir)

    stop_event = signal.signal

    def _on_signal(sig, frame):
        logging.info("signal received, shutting down...")
        # 仅置标志，由 DVLSerialInterface 的 stop 来打断读取
        # 这里用异常流（KeyboardInterrupt）也可，但保持对称更稳
        raise KeyboardInterrupt()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # 包装 on_parsed：既写完整 parsed，又写最小速度表
    def _on_parsed_and_filter(d):
        logger.store_parsed(d)
        if speed_min:
            speed_min.maybe_write(d)

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
        on_raw=logger.store_raw_line,
        on_parsed=_on_parsed_and_filter,
    )

    if not dvl.start_listening(raw_only=args.raw_only):
        logging.error("start_listening failed, exit.")
        logger.close()
        if speed_min: speed_min.close()
        return

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

        if args.send_cs:
            logging.info("[DVL-CMD] CS (start)")
            dvl.write_cmd_try("CS", "")

        # 主循环：轻量 sleep
        while True:
            time.sleep(0.1)

    except KeyboardInterrupt:
        pass
    finally:
        if args.send_cz:
            try:
                logging.info("[DVL-CMD] CZ (stop)")
                dvl.write_cmd_try("CZ", "")
            except Exception:
                pass
        time.sleep(0.1)
        dvl.stop_listening()
        logger.close()
        if speed_min: speed_min.close()
        logging.info(f"bye. stats: {dvl.stats()}")


if __name__ == "__main__":
    main()
