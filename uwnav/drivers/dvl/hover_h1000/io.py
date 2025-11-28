#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
uwnav/drivers/dvl/hover_h1000/io.py

职责：
- 定义 DVLData 数据类（含 valid 标志）
- 定义两个写手：
    * DVLLogger：原始/解析双表
    * MinimalSpeedWriter：仅写“有效(A)速度(m/s)+时间戳”的精简表
- 定义 DVLSerialInterface（串口读/命令下发/解析回调）
- 依赖：同目录的 protocol.py 提供 parse_lines()

改进点：
- 快速停止：_stop_event + cancel_read() + close() + join()（容错保护）
- Windows/Unix 通用串口读：read_until('\r')，兼容 CR/CRLF
- 命令通道：固定 16 字节优先（"CC " + 12 参数右侧空格填充 + '\r'），失败回退 ":CC,<arg>\r"
- 有效位贯穿：protocol 返回的 valid (A/V/None) 写入 DVLData.valid，MinimalSpeedWriter 默认仅写 valid==True 的速度
"""

from __future__ import annotations

import csv
import time
import math
import logging
import threading
from dataclasses import dataclass
from typing import Optional, Tuple, List, Dict, Any
from pathlib import Path

import serial  # pip install pyserial

from .protocol import parse_lines  # -> List[dict]

__all__ = [
    "DVLData",
    "DVLLogger",
    "MinimalSpeedWriter",
    "DVLSerialInterface",
    "DEBUG_PRINT_EVERY",
    "FLUSH_EVERY",
    "READ_IDLE_SLEEP",
    "WARN_IF_IDLE_SEC",
    "SENSOR_ID",
    "RESP_WAIT_STEP",
    "RESP_WAIT_TOTAL",
]

# 运行参数（库级默认，可被外部覆盖）
DEBUG_PRINT_EVERY = 5
FLUSH_EVERY       = 50
READ_IDLE_SLEEP   = 0.005
WARN_IF_IDLE_SEC  = 3.0
SENSOR_ID         = "DVL_H1000"
RESP_WAIT_STEP    = 0.05
RESP_WAIT_TOTAL   = 1.20


def _to_float(x) -> Optional[float]:
    if x is None:
        return None
    try:
        return float(x)
    except Exception:
        return None


def _finite(x) -> bool:
    return (x is not None) and (not (isinstance(x, float) and math.isnan(x)))


@dataclass
class DVLData:
    """
    解析后的统一结构：
    - timestamp: 采集到该帧的本机 Unix 秒
    - src: 帧类型（BE/BS/BI/BD/SA/TS ...）
    - ve_mm, vn_mm, vu_mm: 东北天速度（mm/s）；None 表示无效/占位符
    - depth_m, e_m, n_m, u_m: 位移/高度（m）
    - valid: A/V/None（速度有效位；非速度帧通常为 None）

    便捷属性：
    - ve, vn, vu: 若 mm/s 有限，则转 m/s，否则为 None
    """
    timestamp: float
    src: str
    ve_mm: Optional[float]
    vn_mm: Optional[float]
    vu_mm: Optional[float]
    depth_m: Optional[float]
    e_m: Optional[float] = None
    n_m: Optional[float] = None
    u_m: Optional[float] = None
    valid: Optional[bool] = None

    @property
    def ve(self) -> Optional[float]:
        return None if not _finite(self.ve_mm) else self.ve_mm / 1000.0

    @property
    def vn(self) -> Optional[float]:
        return None if not _finite(self.vn_mm) else self.vn_mm / 1000.0

    @property
    def vu(self) -> Optional[float]:
        return None if not _finite(self.vu_mm) else self.vu_mm / 1000.0


class DVLLogger:
    """
    写两份 CSV：
      1) dvl_raw_lines_*.csv  — 原始行 + 标注（CMD/RESP/空）
      2) dvl_parsed_*.csv     — 解析后的统一字段（速度/位移/高度/valid）
    """
    def __init__(self, out_dir: str, sensor_id: str = SENSOR_ID):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.sensor_id = sensor_id
        self._count = 0
        self._ts_start = time.strftime("%Y%m%d_%H%M%S")

        self.raw_file, self.raw_writer = self._make_raw_writer()
        self.parsed_file, self.parsed_writer = self._make_parsed_writer()

        logging.info(f"[DVL-LOGGER] raw    -> {self.raw_file.name}")
        logging.info(f"[DVL-LOGGER] parsed -> {self.parsed_file.name}")

    def _make_raw_writer(self) -> Tuple[object, csv.writer]:
        raw_path = self.out_dir / f"dvl_raw_lines_{self._ts_start}.csv"
        f = open(raw_path, "a", newline="", encoding="utf-8")
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow(["Timestamp(s)", "SensorID", "RawLine", "CmdOrNote"])
            f.flush()
        return f, w

    def _make_parsed_writer(self) -> Tuple[object, csv.writer]:
        parsed_path = self.out_dir / f"dvl_parsed_{self._ts_start}.csv"
        f = open(parsed_path, "a", newline="", encoding="utf-8")
        w = csv.writer(f)
        if f.tell() == 0:
            w.writerow([
                "Timestamp(s)", "SensorID", "Src",
                "East_Vel(mm_s)", "North_Vel(mm_s)", "Up_Vel(mm_s)",
                "East_Vel(m_s)",  "North_Vel(m_s)",  "Up_Vel(m_s)",
                "Depth(m)", "E(m)", "N(m)", "U(m)", "Valid"
            ])
            f.flush()
        return f, w

    def store_raw_line(self, ts: float, line: str, note: str = ""):
        self.raw_writer.writerow([ts, self.sensor_id, line, note])

    def store_parsed(self, d: DVLData):
        self.parsed_writer.writerow([
            d.timestamp, self.sensor_id, d.src,
            d.ve_mm, d.vn_mm, d.vu_mm,
            d.ve,    d.vn,    d.vu,
            d.depth_m, d.e_m, d.n_m, d.u_m, d.valid
        ])
        self._count += 1

        if DEBUG_PRINT_EVERY and (self._count % DEBUG_PRINT_EVERY == 0):
            logging.info(
                f"[DVL] n={self._count} src={d.src} valid={d.valid} "
                f"ve(mm)={d.ve_mm} vn(mm)={d.vn_mm} vu(mm)={d.vu_mm} depth={d.depth_m}"
            )

        if FLUSH_EVERY and (self._count % FLUSH_EVERY == 0):
            try:
                self.raw_file.flush()
                self.parsed_file.flush()
            except Exception:
                pass

    def close(self):
        try:
            self.raw_file.flush()
            self.parsed_file.flush()
        except Exception:
            pass
        self.raw_file.close()
        self.parsed_file.close()


class MinimalSpeedWriter:
    """
    仅写“有效速度(m/s)+时间戳”的精简表，供快速融合/对齐。
    - only_valid=True：仅写 valid==True 的帧（A）
    - require_all_axes=True：要求 ve/vn/vu 三轴均为有限数才写入
    """
    def __init__(self, out_dir: str, sensor_id: str = SENSOR_ID,
                 only_valid: bool = True, require_all_axes: bool = True):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.sensor_id = sensor_id
        self.only_valid = only_valid
        self.require_all_axes = require_all_axes

        ts_start = time.strftime("%Y%m%d_%H%M%S")
        path = self.out_dir / f"dvl_speed_min_{ts_start}.csv"
        self.f = open(path, "a", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow(["Timestamp(s)", "SensorID", "Src", "East_Vel(m_s)", "North_Vel(m_s)", "Up_Vel(m_s)", "Valid"])
            self.f.flush()
        logging.info(f"[DVL-SPEED] -> {self.f.name}")
        self._n = 0

    def maybe_write(self, d: DVLData):
        if self.only_valid and d.valid is not True:
            return
        if self.require_all_axes:
            if not (_finite(d.ve) and _finite(d.vn) and _finite(d.vu)):
                return
        self.w.writerow([d.timestamp, self.sensor_id, d.src, d.ve, d.vn, d.vu, d.valid])
        self._n += 1
        if (self._n % 200) == 0:
            try: self.f.flush()
            except: pass

    def close(self):
        try: self.f.flush()
        except: pass
        self.f.close()


class DVLSerialInterface:
    """
    串口读/命令下发 & 解析回调：
      - read_until('\\r')，兼容 CR/CRLF
      - 停止：set -> cancel_read() -> close() -> join()
      - 命令：优先固定16字节（"CC " + 12参数右侧空格填充 + '\\r'），失败回退 ":CC,<arg>\\r"
    """
    def __init__(self, port: str, baud: int = 115200,
                 parity: str = "N", stopbits: float = 1,
                 rtscts: bool = False, dsrdtr: bool = False, xonxoff: bool = False,
                 no_dtr: bool = False, no_rts: bool = False,
                 on_raw=None, on_parsed=None):
        self.port = port
        self.baud = baud
        self.on_raw = on_raw
        self.on_parsed = on_parsed

        self.parity = {"N": serial.PARITY_NONE, "E": serial.PARITY_EVEN, "O": serial.PARITY_ODD}.get(parity.upper(), serial.PARITY_NONE)
        self.stopbits = serial.STOPBITS_ONE if stopbits == 1 else serial.STOPBITS_TWO
        self.rtscts = rtscts
        self.dsrdtr = dsrdtr
        self.xonxoff = xonxoff
        self.no_dtr = no_dtr
        self.no_rts = no_rts

        self.serial_conn: Optional[serial.Serial] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None

        self._last_rx_ts = 0.0
        self._n_rx = 0
        self._n_parsed_ok = 0
        self._n_parsed_fail = 0

    # ------ open/close ------
    def open(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                self.port, self.baud,
                bytesize=serial.EIGHTBITS,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=0.2, inter_byte_timeout=0.2,
                rtscts=self.rtscts, dsrdtr=self.dsrdtr, xonxoff=self.xonxoff
            )
            if self.no_dtr:
                try: self.serial_conn.dtr = False
                except: pass
            if self.no_rts:
                try: self.serial_conn.rts = False
                except: pass
            logging.info(f"[DVL-SERIAL] open {self.port} @ {self.baud} (8{'N' if self.parity==serial.PARITY_NONE else 'E' if self.parity==serial.PARITY_EVEN else 'O'}1)")
            return True
        except serial.SerialException as e:
            logging.error(f"[DVL-SERIAL] open failed: {e}")
            return False

    def close(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.close()
                logging.info("[DVL-SERIAL] closed")
            except Exception as e:
                logging.warning(f"[DVL-SERIAL] close error: {e}")

    # ------ read/parse ------
    def _read_one_frame(self) -> Optional[str]:
        try:
            raw = self.serial_conn.read_until(b'\r')
            if not raw:
                return None
            s = raw.replace(b"\r", b"").replace(b"\n", b"").decode("ascii", errors="ignore").strip()
            return s if s else None
        except Exception:
            return None

    @staticmethod
    def _pkt_to_dvldata(pkt: Dict[str, Any], ts: float) -> Optional[DVLData]:
        if not pkt:
            return None
        src = str(pkt.get("src", "")) if "src" in pkt else ""
        ve = _to_float(pkt.get("ve"))
        vn = _to_float(pkt.get("vn"))
        vu = _to_float(pkt.get("vu"))
        e  = _to_float(pkt.get("e"))
        n  = _to_float(pkt.get("n"))
        u  = _to_float(pkt.get("u"))
        depth = _to_float(pkt.get("depth"))
        valid = pkt.get("valid", None)
        if isinstance(valid, str):
            v = valid.strip().upper()
            valid = True if v == "A" else False if v == "V" else None

        return DVLData(
            timestamp=ts, src=src,
            ve_mm=ve, vn_mm=vn, vu_mm=vu,
            depth_m=depth, e_m=e, n_m=n, u_m=u, valid=valid
        )

    def _listen(self, raw_only: bool):
        last_warn = 0.0
        while not self._stop_event.is_set():
            line = self._read_one_frame()
            now = time.time()
            if not line:
                if self._last_rx_ts and (now - self._last_rx_ts > WARN_IF_IDLE_SEC) and (now - last_warn > 1.0):
                    logging.warning(f"[DVL] idle > {WARN_IF_IDLE_SEC:.1f}s without data. Check port/baud/cabling.")
                    last_warn = now
                time.sleep(READ_IDLE_SLEEP)
                continue

            ts = now
            self._last_rx_ts = ts

            if self.on_raw:
                try:
                    self.on_raw(ts, line, "")
                except Exception:
                    pass

            self._n_rx += 1

            if raw_only:
                continue

            try:
                frames = parse_lines(line)  # 支持一行多帧
                if not frames:
                    self._n_parsed_fail += 1
                    continue
                for pkt in frames:
                    try:
                        d = self._pkt_to_dvldata(pkt, ts)
                        if d and self.on_parsed:
                            self.on_parsed(d)
                            self._n_parsed_ok += 1
                        else:
                            self._n_parsed_fail += 1
                    except Exception:
                        self._n_parsed_fail += 1
            except Exception:
                self._n_parsed_fail += 1

        logging.info("[DVL-SERIAL] listen thread exit")

    def start_listening(self, raw_only: bool) -> bool:
        if not self.open():
            return False
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._listen, args=(raw_only,), daemon=True)
        self._thread.start()
        logging.info("[DVL-SERIAL] listen thread started")
        return True

    def stop_listening(self):
        self._stop_event.set()
        # 尽量打断阻塞 read
        try:
            if self.serial_conn:
                self.serial_conn.cancel_read()
        except Exception:
            pass
        try:
            self.close()
        except Exception:
            pass
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)

    # ------ command I/O ------
    def _send_fixed16(self, cmd2: str, arg: Optional[str]) -> None:
        """
        固定16字节： b'{CC} ' + 12字节参数(右侧空格填充) + b'\\r'
        例：DF,0 -> b'DF 0           \\r'
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        param = "" if arg is None else str(arg)
        body = (cmd2.upper() + " " + param)[:13].ljust(13, " ")
        payload = body.encode("ascii", errors="ignore") + b"\r"
        self.serial_conn.write(payload)
        if self.on_raw:
            self.on_raw(time.time(), body, "CMD(fixed16)")

    def _send_colon_comma(self, cmd2: str, arg: Optional[str]) -> None:
        """回退：':CC,<arg>\\r'"""
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        frame = f":{cmd2.upper()}\r" if arg is None else f":{cmd2.upper()},{arg}\r"
        self.serial_conn.write(frame.encode("ascii", errors="ignore"))
        if self.on_raw:
            self.on_raw(time.time(), frame.strip(), "CMD(colon,comma)")

    def write_cmd_try(self, cmd2: str, arg: Optional[str]) -> List[str]:
        """先 fixed16，再必要时回退；收集回显/应答文本（也会写入 raw csv）。"""
        self._send_fixed16(cmd2, arg)
        resp1 = self.read_resp_lines(RESP_WAIT_TOTAL)
        looks_ok = any(r.startswith(":") or r.upper().startswith(cmd2.upper()) for r in resp1)
        if not looks_ok:
            self._send_colon_comma(cmd2, arg)
            resp2 = self.read_resp_lines(RESP_WAIT_TOTAL)
            return resp1 + resp2
        return resp1

    def read_resp_lines(self, total_wait: float = RESP_WAIT_TOTAL) -> List[str]:
        lines: List[str] = []
        deadline = time.time() + total_wait
        while time.time() < deadline and not self._stop_event.is_set():
            try:
                raw = self.serial_conn.read_until(b"\r")
                if not raw:
                    time.sleep(RESP_WAIT_STEP)
                    continue
                s = raw.replace(b"\r", b"").replace(b"\n", b"").decode("ascii", errors="ignore").strip()
                if s:
                    lines.append(s)
                    if self.on_raw:
                        self.on_raw(time.time(), s, "RESP")
            except Exception:
                time.sleep(RESP_WAIT_STEP)
        return lines

    def stats(self) -> str:
        return (f"rx={self._n_rx}, parsed_ok={self._n_parsed_ok}, parsed_fail={self._n_parsed_fail}")
