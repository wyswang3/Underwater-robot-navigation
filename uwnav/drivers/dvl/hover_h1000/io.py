#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
uwnav/drivers/dvl/hover_h1000/io.py

职责：
- 定义 DVLData 数据类（含 valid 标志与原始 A/V）
- 定义两个写手：
    * DVLLogger：原始/解析双表（尽量保留全部关键信息）
    * MinimalSpeedWriter：仅写“速度(m/s)+时间戳”的精简表
- 定义 DVLSerialInterface（串口读/命令下发/解析回调）
- 依赖：同目录的 protocol.py 提供 parse_lines() -> List[dict]

设计原则：
- io 层只负责“忠实解析 + 设备相关清洗（如 8888 占位符）”，不做工程策略
- A/V 有效标志同时以 bool（valid）和原始字符（valid_raw）形式保留
- 命令采用严格 16 字符帧（CS/CZ/PR/PM 等），兼容你手册里的格式示例
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

# ===================== 运行参数（库级默认，可被外部覆盖） =====================

DEBUG_PRINT_EVERY = 5
FLUSH_EVERY       = 50
READ_IDLE_SLEEP   = 0.005
WARN_IF_IDLE_SEC  = 3.0
SENSOR_ID         = "DVL_H1000"
RESP_WAIT_STEP    = 0.05
RESP_WAIT_TOTAL   = 1.20


# ===================== 工具函数：数值解析 / 命令构造 =====================

# 显式无效占位符（可按需要继续扩展）
_INVALID_SENTINELS = {
    8888.0, 9999.0, -8888.0, -9999.0,
    88888.0, 99999.0, -88888.0, -99999.0,
}


def _to_float(x) -> Optional[float]:
    """
    通用安全 float 转换：
    - None / 空字符串 / 解析失败 / NaN -> None
    - 特殊占位符 (8888 / 88888 / 9999 / 99999 / 绝对值 >= 88888) -> None
    """
    if x is None:
        return None
    if isinstance(x, str):
        s = x.strip()
        if not s:
            return None
        try:
            v = float(s)
        except Exception:
            return None
    else:
        try:
            v = float(x)
        except Exception:
            return None

    if not math.isfinite(v):
        return None

    # 显式占位符过滤
    for s in _INVALID_SENTINELS:
        if abs(v - s) < 1e-6:
            return None

    # 保险：过大的值统一视作占位符
    if abs(v) >= 88888.0:
        return None

    return v


def _finite(x) -> bool:
    try:
        return x is not None and math.isfinite(float(x))
    except Exception:
        return False


def fmt_int_2d(v: int) -> str:
    """
    DVL 命令里的“2 位带前导空格”的整数格式：
    - 1  -> " 1"
    - 10 -> "10"
    """
    v = int(v)
    if not (0 <= v <= 99):
        raise ValueError(f"PR/PM 值必须在 0..99，当前={v}")
    return f"{v:2d}"


def build_cmd16(cmd2: str, arg: Optional[str]) -> bytes:
    """
    DH1000 风格 16 字符命令帧构造（依据说明书示例 + 需求）：

    约定：
    - cmd2：2 字符命令（CZ/CS/PR/PM 等）
    - 无参数命令：CMD + '\\r' + '0' 补齐到 16 字符
        例如：CS\\r0000000000000
    - 有参数命令：CMD + ' ' + arg + '\\r' + '0' 补齐到 16 字符
        例如：PR 10\\r000000000000

    返回：
    - bytes，长度必为 16
    """
    cmd2 = (cmd2 or "").strip().upper()
    if len(cmd2) != 2:
        raise ValueError(f"cmd2 must be 2 chars, got {cmd2!r}")

    if arg is None or str(arg).strip() == "":
        # 无参数命令：不加空格
        s = cmd2
    else:
        # 有参数命令：CMD + ' ' + 参数
        s = cmd2 + " " + str(arg)

    s += "\r"  # 帧内回车

    if len(s) > 16:
        raise ValueError(f"CMD16 too long before padding: {s!r} len={len(s)}")

    s = s.ljust(16, "0")
    if len(s) != 16:
        raise RuntimeError("internal: build_cmd16 length != 16")

    return s.encode("ascii", errors="strict")


# ===================== 数据结构：DVLData =====================

@dataclass
class DVLData:
    """
    解析后的统一结构（按帧类型拆解）：

    timestamp : 采集到该帧的本机 Unix 秒
    src       : 帧类型（"BI"/"BS"/"BE"/"BD"/"WI"/"WS"/"WE"/"WD"/...）

    速度（三套）：
    - vx_body_mm, vy_body_mm, vz_body_mm
        * BI：设备坐标系速度
        * BS：船体坐标系速度
        * WI/WS：水团速度（设备/船体参考）
    - ve_enu_mm, vn_enu_mm, vu_enu_mm
        * BE：大地 ENU 速度
        * WE：水团相对大地 ENU 速度

    距离（ENU，大地坐标系）：
    - de_enu_m, dn_enu_m, du_enu_m
        * BD：大地 ENU 距离
        * WD：水团对地距离

    depth_m : 深度（若协议提供）
    e_m, n_m, u_m : 若协议中有额外的位置/高度字段，可继续复用

    valid_raw : 原始有效标记（"A"/"V"/None）
    valid     : A -> True，V -> False，其它 -> None

    is_water_mass : 是否水团类帧（WI/WS/WE/WD）
    """

    timestamp: float
    src: str

    # 体坐标/船体坐标/水团在体系下的三轴速度 (mm/s)
    vx_body_mm: Optional[float] = None
    vy_body_mm: Optional[float] = None
    vz_body_mm: Optional[float] = None

    # 大地 ENU 速度 (mm/s)
    ve_enu_mm: Optional[float] = None
    vn_enu_mm: Optional[float] = None
    vu_enu_mm: Optional[float] = None

    # 大地 ENU 距离 (m)
    de_enu_m: Optional[float] = None
    dn_enu_m: Optional[float] = None
    du_enu_m: Optional[float] = None

    # 其它位置/深度字段（按你之前协议，可选）
    depth_m: Optional[float] = None
    e_m: Optional[float] = None
    n_m: Optional[float] = None
    u_m: Optional[float] = None

    valid_raw: Optional[str] = None
    valid: Optional[bool] = None

    # 是否水团类帧（WI/WS/WE/WD）
    is_water_mass: bool = False

    # --- 体坐标/船体坐标 m/s（X/Y/Z） ---
    @property
    def vx(self) -> Optional[float]:
        return float(self.vx_body_mm) / 1000.0 if _finite(self.vx_body_mm) else None

    @property
    def vy(self) -> Optional[float]:
        return float(self.vy_body_mm) / 1000.0 if _finite(self.vy_body_mm) else None

    @property
    def vz(self) -> Optional[float]:
        return float(self.vz_body_mm) / 1000.0 if _finite(self.vz_body_mm) else None

    # --- ENU 速度 m/s（East/North/Up） ---
    @property
    def ve_enu(self) -> Optional[float]:
        return float(self.ve_enu_mm) / 1000.0 if _finite(self.ve_enu_mm) else None

    @property
    def vn_enu(self) -> Optional[float]:
        return float(self.vn_enu_mm) / 1000.0 if _finite(self.vn_enu_mm) else None

    @property
    def vu_enu(self) -> Optional[float]:
        return float(self.vu_enu_mm) / 1000.0 if _finite(self.vu_enu_mm) else None



# ===================== DVLLogger：raw + parsed 双表 =====================

class DVLLogger:
    """
    写两份 CSV：
      1) dvl_raw_lines_*.csv  — 原始行 + 标注（CMD/RESP/空）
      2) dvl_parsed_*.csv     — 解析后的统一字段（速度/位移/高度/valid）

    说明：
    - parsed 表头使用体坐标命名：X/Y/Z，而不是 East/North/Up，
      以避免误导（你当前项目中把该速度当作体坐标使用）。
    - 同时保留 valid (bool) 与 valid_raw (A/V/None)。
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
                # 体/船体坐标三轴速度（BI/BS/WI/WS）
                "Vx_body(mm_s)", "Vy_body(mm_s)", "Vz_body(mm_s)",
                "Vx_body(m_s)",  "Vy_body(m_s)",  "Vz_body(m_s)",
                # 大地 ENU 速度（BE/WE）
                "Ve_enu(mm_s)", "Vn_enu(mm_s)", "Vu_enu(mm_s)",
                "Ve_enu(m_s)",  "Vn_enu(m_s)",  "Vu_enu(m_s)",
                # 大地 ENU 距离（BD/WD）
                "De_enu(m)", "Dn_enu(m)", "Du_enu(m)",
                # 其它位置/深度字段（按你之前的协议，可保留）
                "Depth(m)", "E(m)", "N(m)", "U(m)",
                # 有效标志 + 是否水团帧
                "Valid", "ValidFlag", "IsWaterMass",
            ])
            f.flush()
        return f, w


    def store_raw_line(self, ts: float, line: str, note: str = ""):
        self.raw_writer.writerow([ts, self.sensor_id, line, note])

    def store_parsed(self, d: DVLData):
        self.parsed_writer.writerow([
            d.timestamp, self.sensor_id, d.src,
            # 体/船体坐标速度
            d.vx_body_mm, d.vy_body_mm, d.vz_body_mm,
            d.vx,         d.vy,         d.vz,
            # 大地 ENU 速度
            d.ve_enu_mm,  d.vn_enu_mm,  d.vu_enu_mm,
            d.ve_enu,     d.vn_enu,     d.vu_enu,
            # ENU 距离
            d.de_enu_m,   d.dn_enu_m,   d.du_enu_m,
            # 其它位置/深度（如果你有用）
            d.depth_m,    d.e_m,        d.n_m,    d.u_m,
            # 有效标志
            d.valid, d.valid_raw, d.is_water_mass,
        ])
        self._count += 1

        if DEBUG_PRINT_EVERY and (self._count % DEBUG_PRINT_EVERY == 0):
            logging.info(
                f"[DVL] n={self._count} src={d.src} "
                f"valid={d.valid} water={d.is_water_mass} "
                f"vx_body(mm)={d.vx_body_mm} vy_body(mm)={d.vy_body_mm} "
                f"vz_body(mm)={d.vz_body_mm} "
                f"ve_enu(mm)={d.ve_enu_mm} vn_enu(mm)={d.vn_enu_mm} "
                f"vu_enu(mm)={d.vu_enu_mm} "
                f"de_enu(m)={d.de_enu_m} dn_enu(m)={d.dn_enu_m} du_enu(m)={d.du_enu_m}"
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


# ===================== MinimalSpeedWriter：精简速度表 =====================

class MinimalSpeedWriter:
    """
    仅写“速度(m/s)+时间戳”的精简表，供快速融合/对齐。

    参数：
    - only_valid       : True 时仅写 valid==True 的帧（A）
    - require_all_axes : True 时要求 vx/vy/vz 三轴均为有限数才写入
    - invalid_to_zero  : True 时无效轴写 0.0（覆盖 only_valid/require_all_axes 部分行为）

    注意：
    - 这个类只是便捷工具；后续我们会在 DVL_logger.py 中新建 TB 写手，
      按你的规则做“速度归 0 / 高度前值填充”等处理。
    """

    def __init__(self, out_dir: str, sensor_id: str = SENSOR_ID,
                 only_valid: bool = True,
                 require_all_axes: bool = True,
                 invalid_to_zero: bool = False):
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.sensor_id = sensor_id
        self.only_valid = only_valid
        self.require_all_axes = require_all_axes
        self.invalid_to_zero = invalid_to_zero

        ts_start = time.strftime("%Y%m%d_%H%M%S")
        path = self.out_dir / f"dvl_speed_min_{ts_start}.csv"
        self.f = open(path, "a", newline="", encoding="utf-8")
        self.w = csv.writer(self.f)
        if self.f.tell() == 0:
            self.w.writerow([
                "Timestamp(s)", "SensorID", "Src",
                "Vx_body(m_s)", "Vy_body(m_s)", "Vz_body(m_s)",
                "Valid", "ValidFlag",
            ])
            self.f.flush()
        logging.info(f"[DVL-SPEED] -> {self.f.name}")
        self._n = 0

    @staticmethod
    def _val_or_zero(x: Optional[float]) -> float:
        return float(x) if _finite(x) else 0.0

    def maybe_write(self, d: DVLData):
        vx = d.vx
        vy = d.vy
        vz = d.vz

        if self.invalid_to_zero:
            vx_out = self._val_or_zero(vx)
            vy_out = self._val_or_zero(vy)
            vz_out = self._val_or_zero(vz)
        else:
            if self.only_valid and d.valid is not True:
                return
            if self.require_all_axes:
                if not (_finite(vx) and _finite(vy) and _finite(vz)):
                    return
            vx_out = vx
            vy_out = vy
            vz_out = vz

        self.w.writerow([
            d.timestamp, self.sensor_id, d.src,
            vx_out, vy_out, vz_out,
            d.valid, d.valid_raw,
        ])
        self._n += 1
        if (self._n % 200) == 0:
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


# ===================== DVLSerialInterface：串口读 + 命令下发 =====================

class DVLSerialInterface:
    """
    串口读/命令下发 & 解析回调：

    - read_until('\\r')，兼容 CR/CRLF
    - 停止：set -> cancel_read() -> close() -> join()
    - 命令：
        * 严格 16 字符命令帧（build_cmd16）
        * 提供 set_ping_rate(PR) / set_average_count(PM) / safe_start(CZ+CS+PR+PM)
    """

    def __init__(self, port: str, baud: int = 115200,
                 parity: str = "N", stopbits: float = 1,
                 rtscts: bool = False, dsrdtr: bool = False, xonxoff: bool = False,
                 no_dtr: bool = False, no_rts: bool = False,
                 on_raw=None, on_parsed=None, on_event=None):
        self.port = port
        self.baud = baud
        self.on_raw = on_raw
        self.on_parsed = on_parsed
        self.on_event = on_event

        self.parity = {
            "N": serial.PARITY_NONE,
            "E": serial.PARITY_EVEN,
            "O": serial.PARITY_ODD
        }.get(parity.upper(), serial.PARITY_NONE)

        self.stopbits = serial.STOPBITS_ONE if float(stopbits) == 1.0 else serial.STOPBITS_TWO
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
        self._n_open_fail = 0
        self._n_idle_warn = 0
        self._n_raw_callback_error = 0
        self._n_parsed_callback_error = 0
        self._n_parse_empty = 0
        self._n_read_error = 0
    def _emit_event(self, level: str, code: str, message: str, **details: Any) -> None:
        if self.on_event is None:
            return
        try:
            self.on_event(level=level, code=code, message=message, **details)
        except Exception:
            pass

    # ------ open/close ------

    def open(self) -> bool:
        try:
            self.serial_conn = serial.Serial(
                self.port, self.baud,
                bytesize=serial.EIGHTBITS,
                parity=self.parity,
                stopbits=self.stopbits,
                timeout=0.2,
                inter_byte_timeout=0.2,
                rtscts=self.rtscts,
                dsrdtr=self.dsrdtr,
                xonxoff=self.xonxoff
            )
            if self.no_dtr:
                try:
                    self.serial_conn.dtr = False
                except Exception:
                    pass
            if self.no_rts:
                try:
                    self.serial_conn.rts = False
                except Exception:
                    pass
            logging.info(
                f"[DVL-SERIAL] open {self.port} @ {self.baud} "
                f"(8{'N' if self.parity == serial.PARITY_NONE else 'E' if self.parity == serial.PARITY_EVEN else 'O'}1)"
            )
            self._emit_event(
                "info",
                "dvl_serial_opened",
                "dvl serial opened",
                port=self.port,
                baud=self.baud,
            )
            return True
        except serial.SerialException as e:
            self._n_open_fail += 1
            logging.error(f"[DVL-SERIAL] open failed: {e}")
            self._emit_event(
                "error",
                "dvl_serial_open_failed",
                "dvl serial open failed",
                port=self.port,
                baud=self.baud,
                error=str(e),
                open_fail_count=self._n_open_fail,
            )
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
        """
        从串口读取一帧（以 '\\r' 结尾），去掉 CR/LF，返回去空白后的 ASCII 字符串。
        """
        try:
            raw = self.serial_conn.read_until(b"\\r")
            if not raw:
                return None
            s = raw.replace(b"\\r", b"").replace(b"\\n", b"").decode("ascii", errors="ignore").strip()
            return s if s else None
        except Exception as exc:
            self._n_read_error += 1
            if (self._n_read_error == 1) or (self._n_read_error % 20 == 0):
                self._emit_event(
                    "warn",
                    "dvl_serial_read_error",
                    "dvl serial read failed",
                    error=str(exc),
                    read_error_count=self._n_read_error,
                )
            return None
    @staticmethod
    def _pkt_to_dvldata(pkt: Dict[str, Any], ts: float) -> Optional[DVLData]:
        """
        将 protocol.parse_lines() 返回的单帧字典映射到 DVLData。

        这里只接 BI/BS/BE/BD/WI/WS/WE/WD 这类真正会进入 parsed/TB 的
        运动/距离帧；SA/TS 和噪声片段保留在 raw logger，不下沉到 DVLData。
        """
        if not pkt:
            return None

        src = str(pkt.get("src", "")).upper()
        allowed_src = {"BI", "BS", "BE", "BD", "WI", "WS", "WE", "WD"}
        if src not in allowed_src:
            return None

        # 通用数值解析（会把 8888/88888/9999 等占位符映射为 None）
        ve = _to_float(pkt.get("ve"))
        vn = _to_float(pkt.get("vn"))
        vu = _to_float(pkt.get("vu"))

        # 对 BD/WD，协议里一般是 E,N,U 距离（单位 m）
        # 如果你的 protocol.py 用的是其他键名（例如 de/dn/du），这里改一下即可
        de = _to_float(pkt.get("e"))
        dn = _to_float(pkt.get("n"))
        du = _to_float(pkt.get("u"))

        depth = _to_float(pkt.get("depth"))

        # 有效标志
        valid_raw = None
        valid = None
        v_field = pkt.get("valid", None)
        if isinstance(v_field, str):
            valid_raw = v_field.strip().upper() or None
            if valid_raw == "A":
                valid = True
            elif valid_raw == "V":
                valid = False

        # 是否水团帧
        is_water = src.startswith("W")  # WI/WS/WE/WD

        # 初始化所有字段为 None
        vx_body_mm = vy_body_mm = vz_body_mm = None
        ve_enu_mm = vn_enu_mm = vu_enu_mm = None
        de_enu_m = dn_enu_m = du_enu_m = None

        # 分情况填充
        if src in ("BI", "BS", "WI", "WS"):
            # 设备/船体坐标速度，或者水团相对设备/船体速度 → 体坐标 X/Y/Z
            vx_body_mm = ve
            vy_body_mm = vn
            vz_body_mm = vu

        elif src in ("BE", "WE"):
            # 大地 ENU 速度（设备/船体或水团对地）
            ve_enu_mm = ve
            vn_enu_mm = vn
            vu_enu_mm = vu

        if src in ("BD", "WD"):
            # 大地 ENU 距离（设备/水团）
            de_enu_m = de
            dn_enu_m = dn
            du_enu_m = du

        return DVLData(
            timestamp=ts,
            src=src,
            vx_body_mm=vx_body_mm,
            vy_body_mm=vy_body_mm,
            vz_body_mm=vz_body_mm,
            ve_enu_mm=ve_enu_mm,
            vn_enu_mm=vn_enu_mm,
            vu_enu_mm=vu_enu_mm,
            de_enu_m=de_enu_m,
            dn_enu_m=dn_enu_m,
            du_enu_m=du_enu_m,
            depth_m=depth,
            # 如果你原来还在 pkt 里解析 e/n/u 作为其他位置，可以在这里继续填 e_m/n_m/u_m
            e_m=None,
            n_m=None,
            u_m=None,
            valid_raw=valid_raw,
            valid=valid,
            is_water_mass=is_water,
        )

    def _listen(self, raw_only: bool):
        last_warn = 0.0
        while not self._stop_event.is_set():
            line = self._read_one_frame()
            now = time.time()
            if not line:
                if self._last_rx_ts and (now - self._last_rx_ts > WARN_IF_IDLE_SEC) and (now - last_warn > 1.0):
                    logging.warning(
                        f"[DVL] idle > {WARN_IF_IDLE_SEC:.1f}s without data. "
                        f"Check port/baud/cabling."
                    )
                    self._n_idle_warn += 1
                    self._emit_event(
                        "warn",
                        "dvl_idle_timeout",
                        "dvl idle beyond warning threshold",
                        idle_s=round(now - self._last_rx_ts, 3),
                        idle_warn_count=self._n_idle_warn,
                    )
                    last_warn = now
                time.sleep(READ_IDLE_SLEEP)
                continue

            ts = now
            self._last_rx_ts = ts

            # 原始回调（含 CmdOrNote 标注）
            if self.on_raw:
                try:
                    self.on_raw(ts, line, "")
                except Exception as exc:
                    self._n_raw_callback_error += 1
                    if (self._n_raw_callback_error == 1) or (self._n_raw_callback_error % 20 == 0):
                        self._emit_event(
                            "warn",
                            "dvl_raw_callback_error",
                            "dvl raw callback raised exception",
                            error=str(exc),
                            callback_error_count=self._n_raw_callback_error,
                        )

            self._n_rx += 1

            if raw_only:
                continue

            # 解析帧
            try:
                frames = parse_lines(line)  # 支持一行多帧
                if not frames:
                    self._n_parsed_fail += 1
                    self._n_parse_empty += 1
                    if (self._n_parse_empty == 1) or (self._n_parse_empty % 20 == 0):
                        self._emit_event(
                            "warn",
                            "dvl_parse_empty",
                            "dvl parser returned no frame for a raw line",
                            raw_line=line,
                            parse_empty_count=self._n_parse_empty,
                        )
                    continue
                for pkt in frames:
                    try:
                        d = self._pkt_to_dvldata(pkt, ts)
                        if d is None:
                            self._n_parsed_fail += 1
                            self._emit_event(
                                "warn",
                                "dvl_packet_mapped_none",
                                "dvl packet mapped to empty record",
                                packet=pkt,
                            )
                            continue
                        if self.on_parsed:
                            try:
                                self.on_parsed(d)
                            except Exception as exc:
                                self._n_parsed_callback_error += 1
                                self._n_parsed_fail += 1
                                if (self._n_parsed_callback_error == 1) or (self._n_parsed_callback_error % 20 == 0):
                                    self._emit_event(
                                        "warn",
                                        "dvl_parsed_callback_error",
                                        "dvl parsed callback raised exception",
                                        error=str(exc),
                                        callback_error_count=self._n_parsed_callback_error,
                                        src=d.src,
                                    )
                                continue
                        self._n_parsed_ok += 1
                    except Exception as exc:
                        self._n_parsed_fail += 1
                        self._emit_event(
                            "warn",
                            "dvl_packet_parse_error",
                            "dvl packet parse raised exception",
                            error=str(exc),
                            packet=pkt,
                        )
            except Exception as exc:
                self._n_parsed_fail += 1
                self._emit_event(
                    "warn",
                    "dvl_parse_lines_error",
                    "dvl parse_lines raised exception",
                    error=str(exc),
                    raw_line=line,
                )

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

    def _send_cmd16(self, cmd2: str, arg: Optional[str]) -> bytes:
        """
        严格 16 字符命令帧发送（按 DH1000 手册风格）。
        返回实际发送的 payload，便于调试 / echo 检查。
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            raise RuntimeError("serial not open")

        payload = build_cmd16(cmd2, arg)
        self.serial_conn.write(payload)

        if self.on_raw:
            # Raw 里显示时，把回车替换为可见转义，方便排障
            log_s = payload.decode("ascii", errors="ignore").replace("\r", "\\r")
            self.on_raw(time.time(), log_s, "CMD(CMD16)")

        return payload

    def _send_colon_comma(self, cmd2: str, arg: Optional[str]) -> None:
        """
        兼容老行为的“冒号协议”回退：
          ':CC,ARG\\r' 或 ':CC,\\r'（视设备支持情况而定）
        如你确认不再需要，可将调用删掉或改为仅调试使用。
        """
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        if arg is None or str(arg).strip() == "":
            frame = f":{cmd2.upper()},\r"
        else:
            frame = f":{cmd2.upper()},{arg}\r"
        self.serial_conn.write(frame.encode("ascii", errors="ignore"))
        if self.on_raw:
            self.on_raw(time.time(), frame.strip(), "CMD(colon,comma)")

    def read_resp_lines(self, total_wait: float = RESP_WAIT_TOTAL) -> List[str]:
        """
        在给定时间窗口内收集以 '\\r' 结尾的响应行（文本），用于简单 echo 检查。
        """
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

    def write_cmd_try(self,
                      cmd2: str,
                      arg: Optional[str],
                      timeout: float = RESP_WAIT_TOTAL,
                      raise_on_fail: bool = False,
                      use_colon_fallback: bool = True) -> List[str]:
        """
        发送一条命令并尝试接收响应行：

        - 首选：严格 16 字符命令 (CMD16)
        - 简单 echo 判定：响应行中包含 cmd2（不区分大小写）就算“看起来 OK”
        - 可选：如果 CMD16 没看到任何响应，可以再尝试一次冒号协议回退

        参数：
        - timeout          : 总等待时间（秒）
        - raise_on_fail    : echo 未通过时是否抛异常（默认只打 warning）
        - use_colon_fallback: CMD16 无响应时是否尝试 ':CC,ARG\\r'

        返回：
        - 收集到的响应行列表（文本）
        """
        try:
            self._send_cmd16(cmd2, arg)
        except Exception as e:
            logging.error(f"[DVL-CMD16] send failed: {e}")
            if raise_on_fail:
                raise
            return []

        resp1 = self.read_resp_lines(total_wait=timeout)
        looks_ok = any(cmd2.upper() in r.upper() for r in resp1)

        if looks_ok:
            return resp1

        logging.warning(f"[DVL-CMD16] no obvious echo for {cmd2!r}, resp={resp1!r}")

        if use_colon_fallback:
            # 回退一次冒号协议（兼容旧行为）
            try:
                self._send_colon_comma(cmd2, arg)
                resp2 = self.read_resp_lines(total_wait=timeout)
                return resp1 + resp2
            except Exception as e:
                logging.warning(f"[DVL] colon fallback send failed: {e}")

        if raise_on_fail:
            raise RuntimeError(f"DVL command {cmd2!r} did not get a recognizable echo")

        return resp1

    # 高层命令封装：PR / PM / 启动流程 -----------------------------------

    def set_ping_rate(self,
                      hz: int,
                      timeout: float = RESP_WAIT_TOTAL,
                      raise_on_fail: bool = False) -> List[str]:
        """
        设置发射呼率（PR 命令），单位/含义按说明书定义。
        这里将整数格式成 2 位宽（前导空格），例如：
            1  -> " 1"
            10 -> "10"
        """
        arg = fmt_int_2d(hz)
        logging.info(f"[DVL-CMD] PR={arg!r}")
        return self.write_cmd_try("PR", arg, timeout=timeout, raise_on_fail=raise_on_fail)

    def set_average_count(self,
                          n: int,
                          timeout: float = RESP_WAIT_TOTAL,
                          raise_on_fail: bool = False) -> List[str]:
        """
        设置平均次数（PM 命令），整数 0..99，2 位宽字符串。
        """
        arg = fmt_int_2d(n)
        logging.info(f"[DVL-CMD] PM={arg!r}")
        return self.write_cmd_try("PM", arg, timeout=timeout, raise_on_fail=raise_on_fail)

    def safe_start(self,
                   ping_rate: int = 10,
                   avg_count: int = 10,
                   sleep_after_cz: float = 1.0,
                   sleep_after_cs: float = 0.5) -> None:
        """
        建议在采集程序启动时调用的一条龙流程：

        1) CZ: 确保停机（避免空气中 ping）
        2) CS: 启动 ping（此时应确保 DVL 已在水中）
        3) PR: 设置发射呼率（例如 10）
        4) PM: 设置平均次数（例如 10）
        """
        logging.info("[DVL] 发送 CZ（确保停机，避免空气中 ping）")
        try:
            self.write_cmd_try("CZ", None, timeout=2.0, raise_on_fail=False)
        except Exception as e:
            logging.warning(f"[DVL] CZ failed: {e}")
        time.sleep(sleep_after_cz)

        logging.info("[DVL] 发送 CS（启动 ping）")
        try:
            self.write_cmd_try("CS", None, timeout=2.0, raise_on_fail=False)
        except Exception as e:
            logging.warning(f"[DVL] CS failed: {e}")
        time.sleep(sleep_after_cs)

        logging.info(f"[DVL] 设置发射呼率 PR={ping_rate}")
        try:
            self.set_ping_rate(ping_rate, timeout=2.0, raise_on_fail=False)
        except Exception as e:
            logging.warning(f"[DVL] PR failed: {e}")

        logging.info(f"[DVL] 设置平均次数 PM={avg_count}")
        try:
            self.set_average_count(avg_count, timeout=2.0, raise_on_fail=False)
        except Exception as e:
            logging.warning(f"[DVL] PM failed: {e}")

        logging.info("[DVL] safe_start 流程完成")

    # 统计信息 ------------------------------------------------------------

    def stats_dict(self) -> Dict[str, int]:
        return {
            "rx": self._n_rx,
            "parsed_ok": self._n_parsed_ok,
            "parsed_fail": self._n_parsed_fail,
            "open_fail": self._n_open_fail,
            "idle_warn": self._n_idle_warn,
            "raw_callback_error": self._n_raw_callback_error,
            "parsed_callback_error": self._n_parsed_callback_error,
            "parse_empty": self._n_parse_empty,
            "read_error": self._n_read_error,
        }

    def stats(self) -> str:
        st = self.stats_dict()
        return ", ".join(f"{key}={value}" for key, value in st.items())
