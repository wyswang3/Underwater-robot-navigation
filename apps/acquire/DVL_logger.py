#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/acquire/DVL_logger.py

面向“长期运行 / 实时导航数据采集”的 DVL 记录程序：
- 使用统一时间基 (MonoNS + EstNS)
- 自动写入 data/YYYY-MM-DD/dvl/
- 启动时先发送 CZ（确保停机），再发送 CS（启动 ping）
- 结束时再次发送 CZ，确保 DVL 停机，避免空气中 ping 损伤换能器

输出两类文件：
1) DVLLogger: 原始 + 解析后的完整日志（供排障与追溯）
2) MinimalSpeedWriterTB: dvl_speed_min_tb_*.csv（供融合/ESKF/LSTM 使用）
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import time
from pathlib import Path
from typing import Optional

import csv

from uwnav.io.timebase import SensorKind, stamp
from uwnav.drivers.dvl.hover_h1000.io import (
    DVLSerialInterface,
    DVLLogger,
)


# ==================== MinimalSpeedWriterTB ====================

class MinimalSpeedWriterTB:
    """
    带双时间戳( MonoNS + EstNS ) 的最小速度表写手。

    输出列：
    MonoNS, EstNS, MonoS, EstS,
    SensorID, Src,
    East_Vel(m_s), North_Vel(m_s), Up_Vel(m_s)
    """

    def __init__(self, out_dir: Path, sensor_id: str = "DVL_H1000"):
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = out_dir / f"dvl_speed_min_tb_{ts}.csv"
        self.sensor_id = sensor_id
        self._path = path
        self._f = open(path, "a", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        if self._f.tell() == 0:
            self._w.writerow([
                "MonoNS", "EstNS", "MonoS", "EstS",
                "SensorID", "Src",
                "East_Vel(m_s)", "North_Vel(m_s)", "Up_Vel(m_s)",
            ])
        logging.info(f"[DVL-SPEED-TB] 写入: {path}")
        self._n = 0

    def write_if_valid(self, mono_ns: int, est_ns: int, src: str,
                       ve: Optional[float], vn: Optional[float], vu: Optional[float]) -> None:
        # 三轴速度都有效才写入
        if ve is None or vn is None or vu is None:
            return
        self._w.writerow([
            mono_ns,
            est_ns,
            mono_ns / 1e9,
            est_ns / 1e9,
            self.sensor_id,
            src,
            ve,
            vn,
            vu,
        ])
        self._n += 1
        if self._n % 50 == 0:
            try:
                self._f.flush()
            except Exception:
                pass

    def close(self) -> None:
        try:
            self._f.flush()
        except Exception:
            pass
        self._f.close()
        logging.info(f"[DVL-SPEED-TB] 关闭文件: {self._path}")


# ==================== util: 目录与参数 ====================

def repo_root() -> Path:
    """推断仓库根目录：.../Underwater-robot-navigation"""
    here = Path(__file__).resolve()
    # apps/acquire/DVL_logger.py → parents[2] 为 repo_root
    return here.parents[2]


def default_data_root() -> Path:
    """默认 data 根目录：<repo_root>/data"""
    return repo_root() / "data"


def make_day_dvl_dir(data_root: Path) -> Path:
    """根据当前日期生成 data/YYYY-MM-DD/dvl/ 目录"""
    day = time.strftime("%Y-%m-%d")
    d = data_root / day / "dvl"
    d.mkdir(parents=True, exist_ok=True)
    return d


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="DVL logger for long-running navigation data acquisition (timebase unified)"
    )
    ap.add_argument("--port", default="/dev/ttyUSB2", help="DVL 串口 (e.g. /dev/ttyUSB2)")
    ap.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    ap.add_argument("--parity", default="N", choices=["N", "E", "O"], help="校验位")
    ap.add_argument("--stopbits", type=float, default=1.0, choices=[1.0, 2.0], help="停止位")
    ap.add_argument("--rtscts", action="store_true", help="启用 RTS/CTS")
    ap.add_argument("--dsrdtr", action="store_true", help="启用 DSR/DTR")
    ap.add_argument("--xonxoff", action="store_true", help="启用 XON/XOFF")
    ap.add_argument("--no-dtr", action="store_true", help="强制 DTR 低电平")
    ap.add_argument("--no-rts", action="store_true", help="强制 RTS 低电平")

    ap.add_argument("--data-root", default=None,
                    help="data 根目录，默认=<repo_root>/data")
    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARN", "ERROR"], help="日志等级")
    ap.add_argument("--raw-only", action="store_true",
                    help="只记录 raw 行，跳过 parsed")
    ap.add_argument("--stat-every", type=float, default=5.0,
                    help="统计打印间隔秒数（0=不打印）")
    return ap.parse_args()


# ==================== 主程序 ====================

def main():
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    # data 根目录
    if args.data_root:
        data_root = Path(args.data_root).resolve()
    else:
        data_root = default_data_root()
    if not data_root.exists():
        data_root.mkdir(parents=True, exist_ok=True)
    day_dvl_dir = make_day_dvl_dir(data_root)
    logging.info(f"[DVL] data_root={data_root}")
    logging.info(f"[DVL] 今日目录={day_dvl_dir}")

    # 日志器：完整 raw + parsed 日志
    dvl_logger = DVLLogger(str(day_dvl_dir))
    # 最小速度表：给融合/ESKF 使用
    speed_writer = MinimalSpeedWriterTB(day_dvl_dir)

    # 统计
    stats = {"raw": 0, "parsed": 0, "start_t": time.time()}

    # ---- 回调 ----
    def on_raw(_ts_unused: float, line: str, note: str):
        ts = stamp("dvl0", SensorKind.DVL)
        mono_ns = ts.host_time_ns
        est_ns  = ts.corrected_time_ns
        est_s   = est_ns / 1e9

        dvl_logger.store_raw_line(est_s, line, note)
        stats["raw"] += 1

    def on_parsed(d):
        # d 是 DVLData
        dvl_logger.store_parsed(d)
        
        ts = stamp("dvl0", SensorKind.DVL)
        mono_ns = ts.host_time_ns
        est_ns  = ts.corrected_time_ns
        speed_writer.write_if_valid(
        mono_ns, est_ns,
        d.src,
        d.ve, d.vn, d.vu
        )
        stats["parsed"] += 1

    # DVL 串口接口
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
        on_raw=on_raw,
        on_parsed=None if args.raw_only else on_parsed,
    )

    if not dvl.start_listening(raw_only=args.raw_only):
        logging.error("[DVL] start_listening 失败，退出。")
        dvl_logger.close()
        speed_writer.close()
        return

    # ========= 安全启动序列：先 CZ，再 CS =========
    try:
        logging.info("[DVL] 发送 CZ（确保停机，避免空气中 ping）")
        dvl.write_cmd_try("CZ", "")
        time.sleep(1.0)

        logging.info("[DVL] 发送 CS（启动 ping，确保此时 DVL 已在水中）")
        dvl.write_cmd_try("CS", "")
        time.sleep(0.5)

        logging.info("[DVL] 启动序列完成，DVL 应已开始正常工作。")
    except Exception as e:
        logging.error(f"[DVL] 启动序列失败：{e}")
        logging.error("[DVL] 为安全起见，立即退出程序。")
        dvl.stop_listening()
        dvl_logger.close()
        speed_writer.close()
        return
    # =============================================

    # 信号处理
    stop_flag = {"stop": False}

    def _on_sig(sig, frame):
        logging.info(f"[DVL] 收到信号 {sig}，准备退出…")
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _on_sig)
    signal.signal(signal.SIGTERM, _on_sig)

    # 主循环：低频打印统计信息
    last_stat_t = time.time()
    try:
        while not stop_flag["stop"]:
            time.sleep(0.2)
            if args.stat_every > 0:
                now = time.time()
                if now - last_stat_t >= args.stat_every:
                    dt = now - stats["start_t"]
                    raw_rate = stats["raw"] / dt if dt > 0 else 0.0
                    parsed_rate = stats["parsed"] / dt if dt > 0 else 0.0
                    logging.info(
                        f"[STAT] raw={stats['raw']} ({raw_rate:.1f}/s), "
                        f"parsed={stats['parsed']} ({parsed_rate:.1f}/s)"
                    )
                    last_stat_t = now
    except KeyboardInterrupt:
        logging.info("[DVL] KeyboardInterrupt，准备退出…")
    finally:
        # ========= 安全关停序列：CZ =========
        try:
            logging.info("[DVL] 发送 CZ（结束采集，关闭换能器）")
            dvl.write_cmd_try("CZ", "")
        except Exception as e:
            logging.warning(f"[DVL] CZ 发送失败：{e}")
        time.sleep(0.5)
        # ====================================

        dvl.stop_listening()
        dvl_logger.close()
        speed_writer.close()
        logging.info("[DVL] logger 完整退出。")


if __name__ == "__main__":
    main()
