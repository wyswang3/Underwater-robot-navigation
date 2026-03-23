#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/acquire/DVL_logger.py

面向“长期运行 / 实时导航数据采集”的 DVL 记录程序：
- 使用统一时间基 (MonoNS + EstNS)
- 自动写入 data/YYYY-MM-DD/dvl/
- 启动时执行安全序列：
    1) CZ（确保停机）
    2) CS（启动 ping）
    3) PR（设置发射呼率）
    4) PM（设置平均次数）
- 结束时再次发送 CZ，确保 DVL 停机，避免空气中 ping 损伤换能器

输出三类文件：
1) DVLLogger:
   - dvl_raw_lines_*.csv  — 原始 + 命令/应答标注
   - dvl_parsed_*.csv     — 解析后的完整字段（BI/BS/BE/BD + WI/WS/WE/WD）
2) NavStateWriterTB:
   - dvl_nav_state_tb_*.csv
   - 仅用于“设备/船体/大地”导航状态（非水团帧），带统一时间基，适配 ESKF/MPC/LSTM 等
   - 速度无效记作 0.0；位置/距离无效做前值填充
3) WaterMassWriterTB:
   - dvl_watermass_tb_*.csv
   - 仅用于水团（WI/WS/WE/WD），无效数据直接按 0.0 处理，不影响主导航
"""

from __future__ import annotations

import argparse
import csv
import logging
import math
import os
import signal
import time
from pathlib import Path
from typing import Optional

from uwnav.io.timebase import SensorKind, stamp
from uwnav.io.acquisition_diagnostics import SensorRunDiagnostics
from uwnav.drivers.dvl.hover_h1000.io import (
    DVLSerialInterface,
    DVLLogger,
    DVLData,  # 需要在 io.py 的 __all__ 中导出 DVLData
)

SENSOR_ID = "DVL_H1000"


# ==================== 小工具函数 ====================

def _finite(x) -> bool:
    try:
        return x is not None and not (isinstance(x, float) and math.isnan(x))
    except Exception:
        return False


def _to_zero_if_none(x: Optional[float]) -> float:
    return float(x) if _finite(x) else 0.0


# ==================== NavStateWriterTB ====================

class NavStateWriterTB:
    """
    主导航状态 TB 表写手（非水团帧）。

    输出列：
    MonoNS, EstNS, MonoS, EstS,
    SensorID, Src,
    Vx_body(m_s), Vy_body(m_s), Vz_body(m_s),
    Ve_enu(m_s),  Vn_enu(m_s),  Vu_enu(m_s),
    De_enu(m), Dn_enu(m), Du_enu(m),
    Depth(m), E(m), N(m), U(m),
    Valid, ValidFlag, IsWaterMass

    策略：
    - 仅写 d.is_water_mass == False 的帧（BI/BS/BE/BD 等）
    - 速度：
        任何无效/缺失 → 0.0
    - 位置与距离（Depth/De/Dn/Du/E/N/U）：
        * 初始阶段（尚未出现有效位置）：
            - 若本帧全无效 → 输出 0.0
            - 一旦某轴出现有效值 → 该轴=当前值，其余无效轴=0.0，并记录为 last_pos
        * 之后（已有 last_pos）：
            - 某轴本帧有效 → 当前值，并更新 last_pos
            - 某轴无效     → 使用 last_pos 中的上一值（前值填充）
    """

    def __init__(
        self,
        out_dir: Path,
        sensor_id: str = SENSOR_ID,
        flush_every_n: int = 50,
        flush_every_s: float = 1.0,
    ):
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = out_dir / f"dvl_nav_state_tb_{ts}.csv"

        self.sensor_id = sensor_id
        self._path = path
        self._flush_every_n = max(1, int(flush_every_n))
        self._flush_every_s = max(0.0, float(flush_every_s))

        self._f = open(path, "a", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)

        if self._f.tell() == 0:
            self._w.writerow([
                "MonoNS", "EstNS", "MonoS", "EstS",
                "SensorID", "Src",
                "Vx_body(m_s)", "Vy_body(m_s)", "Vz_body(m_s)",
                "Ve_enu(m_s)",  "Vn_enu(m_s)",  "Vu_enu(m_s)",
                "De_enu(m)", "Dn_enu(m)", "Du_enu(m)",
                "Depth(m)", "E(m)", "N(m)", "U(m)",
                "Valid", "ValidFlag", "IsWaterMass",
            ])

        logging.info(f"[DVL-NAV-TB] 写入: {path}")
        self._n = 0
        self._last_flush_t = time.time()

        # 位置/距离前值填充状态
        self._has_pos_valid = False
        # (depth, de, dn, du, e, n, u)
        self._last_pos = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

    # ---- 内部：位置/距离处理 ----

    def _process_position(self, d: DVLData) -> tuple[float, float, float, float, float, float, float]:
        depth = d.depth_m
        de = d.de_enu_m
        dn = d.dn_enu_m
        du = d.du_enu_m
        e = d.e_m
        n = d.n_m
        u = d.u_m

        any_valid_now = any(_finite(x) for x in (depth, de, dn, du, e, n, u))

        if not self._has_pos_valid:
            if not any_valid_now:
                # 尚未出现有效位置 & 本帧依旧全无效 → 全 0
                return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

            # 第一次出现有效位置轴
            depth_out = float(depth) if _finite(depth) else 0.0
            de_out = float(de) if _finite(de) else 0.0
            dn_out = float(dn) if _finite(dn) else 0.0
            du_out = float(du) if _finite(du) else 0.0
            e_out = float(e) if _finite(e) else 0.0
            n_out = float(n) if _finite(n) else 0.0
            u_out = float(u) if _finite(u) else 0.0

            self._last_pos = (depth_out, de_out, dn_out, du_out, e_out, n_out, u_out)
            self._has_pos_valid = True
            return self._last_pos

        # 已有 last_pos，做前值填充
        last_depth, last_de, last_dn, last_du, last_e, last_n, last_u = self._last_pos

        depth_out = float(depth) if _finite(depth) else last_depth
        de_out = float(de) if _finite(de) else last_de
        dn_out = float(dn) if _finite(dn) else last_dn
        du_out = float(du) if _finite(du) else last_du
        e_out = float(e) if _finite(e) else last_e
        n_out = float(n) if _finite(n) else last_n
        u_out = float(u) if _finite(u) else last_u

        self._last_pos = (depth_out, de_out, dn_out, du_out, e_out, n_out, u_out)
        return self._last_pos

    # ---- 对外：写一条导航状态 ----

    def write_sample(self, mono_ns: int, est_ns: int, d: DVLData) -> None:
        # 只记录非水团帧
        if d.is_water_mass:
            return

        # 体/船体坐标速度（无效 → 0）
        vx = _to_zero_if_none(d.vx)
        vy = _to_zero_if_none(d.vy)
        vz = _to_zero_if_none(d.vz)

        # ENU 速度（无效 → 0）
        ve = _to_zero_if_none(d.ve_enu)
        vn = _to_zero_if_none(d.vn_enu)
        vu = _to_zero_if_none(d.vu_enu)

        # 位置与距离（含 Depth/De/Dn/Du/E/N/U）
        depth, de, dn, du, e, n, u = self._process_position(d)

        self._w.writerow([
            mono_ns,
            est_ns,
            mono_ns / 1e9,
            est_ns / 1e9,
            self.sensor_id,
            d.src,
            vx, vy, vz,
            ve, vn, vu,
            de, dn, du,
            depth, e, n, u,
            d.valid,
            d.valid_raw,
            d.is_water_mass,
        ])

        self._n += 1

        # 按条数 + 按时间双触发 flush
        need_flush = (self._n % self._flush_every_n == 0)
        if not need_flush and self._flush_every_s > 0:
            now = time.time()
            if now - self._last_flush_t >= self._flush_every_s:
                need_flush = True

        if need_flush:
            try:
                self._f.flush()
                self._last_flush_t = time.time()
            except Exception:
                pass

    def close(self) -> None:
        try:
            self._f.flush()
        except Exception:
            pass
        try:
            self._f.close()
        except Exception:
            pass
        logging.info(f"[DVL-NAV-TB] 关闭文件: {self._path}")

    def __enter__(self) -> "NavStateWriterTB":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# ==================== WaterMassWriterTB ====================

class WaterMassWriterTB:
    """
    水团（水体）状态 TB 表写手，仅记录 WI/WS/WE/WD 等水团帧。

    输出列：
    MonoNS, EstNS, MonoS, EstS,
    SensorID, Src,
    Vx_body_wm(m_s), Vy_body_wm(m_s), Vz_body_wm(m_s),
    Ve_enu_wm(m_s),  Vn_enu_wm(m_s),  Vu_enu_wm(m_s),
    De_enu_wm(m), Dn_enu_wm(m), Du_enu_wm(m),
    Valid, ValidFlag

    策略：
    - 仅写 d.is_water_mass == True 的帧
    - 速度/距离无效一律记为 0.0（不做前值填充，互不干扰）
    """

    def __init__(
        self,
        out_dir: Path,
        sensor_id: str = SENSOR_ID,
        flush_every_n: int = 50,
        flush_every_s: float = 1.0,
    ):
        os.makedirs(out_dir, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = out_dir / f"dvl_watermass_tb_{ts}.csv"

        self.sensor_id = sensor_id
        self._path = path
        self._flush_every_n = max(1, int(flush_every_n))
        self._flush_every_s = max(0.0, float(flush_every_s))

        self._f = open(path, "a", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)

        if self._f.tell() == 0:
            self._w.writerow([
                "MonoNS", "EstNS", "MonoS", "EstS",
                "SensorID", "Src",
                "Vx_body_wm(m_s)", "Vy_body_wm(m_s)", "Vz_body_wm(m_s)",
                "Ve_enu_wm(m_s)",  "Vn_enu_wm(m_s)",  "Vu_enu_wm(m_s)",
                "De_enu_wm(m)", "Dn_enu_wm(m)", "Du_enu_wm(m)",
                "Valid", "ValidFlag",
            ])

        logging.info(f"[DVL-WATER-TB] 写入: {path}")
        self._n = 0
        self._last_flush_t = time.time()

    def write_sample(self, mono_ns: int, est_ns: int, d: DVLData) -> None:
        if not d.is_water_mass:
            return

        vx = _to_zero_if_none(d.vx)
        vy = _to_zero_if_none(d.vy)
        vz = _to_zero_if_none(d.vz)

        ve = _to_zero_if_none(d.ve_enu)
        vn = _to_zero_if_none(d.vn_enu)
        vu = _to_zero_if_none(d.vu_enu)

        de = _to_zero_if_none(d.de_enu_m)
        dn = _to_zero_if_none(d.dn_enu_m)
        du = _to_zero_if_none(d.du_enu_m)

        self._w.writerow([
            mono_ns,
            est_ns,
            mono_ns / 1e9,
            est_ns / 1e9,
            self.sensor_id,
            d.src,
            vx, vy, vz,
            ve, vn, vu,
            de, dn, du,
            d.valid,
            d.valid_raw,
        ])

        self._n += 1

        need_flush = (self._n % self._flush_every_n == 0)
        if not need_flush and self._flush_every_s > 0:
            now = time.time()
            if now - self._last_flush_t >= self._flush_every_s:
                need_flush = True

        if need_flush:
            try:
                self._f.flush()
                self._last_flush_t = time.time()
            except Exception:
                pass

    def close(self) -> None:
        try:
            self._f.flush()
        except Exception:
            pass
        try:
            self._f.close()
        except Exception:
            pass
        logging.info(f"[DVL-WATER-TB] 关闭文件: {self._path}")

    def __enter__(self) -> "WaterMassWriterTB":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


# ==================== util: 目录与参数 ====================

def repo_root() -> Path:
    """
    尽量稳健地推断仓库根目录：
    - 从当前文件向上找：.git / pyproject.toml / setup.cfg / uwnav 目录
    - 若都找不到，回退到 parents[2]（兼容你原来的结构假设）
    """
    here = Path(__file__).resolve()
    markers = (".git", "pyproject.toml", "setup.cfg")

    for p in [here.parent, *here.parents]:
        for m in markers:
            if (p / m).exists():
                return p
        if (p / "uwnav").exists() and (p / "apps").exists():
            return p

    # fallback（与你原注释一致）
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
    ap.add_argument("--port", default="/dev/ttyACM0", help="DVL 串口 (e.g. /dev/ttyUSB2)")
    ap.add_argument("--baud", type=int, default=115200, help="波特率 (默认 115200)")
    ap.add_argument("--parity", default="N", choices=["N", "E", "O"], help="校验位")
    ap.add_argument("--stopbits", type=float, default=1.0, choices=[1.0, 2.0], help="停止位")
    ap.add_argument("--rtscts", action="store_true", help="启用 RTS/CTS")
    ap.add_argument("--dsrdtr", action="store_true", help="启用 DSR/DTR")
    ap.add_argument("--xonxoff", action="store_true", help="启用 XON/XOFF")
    ap.add_argument("--no-dtr", action="store_true", help="强制 DTR 低电平")
    ap.add_argument("--no-rts", action="store_true", help="强制 RTS 低电平")

    ap.add_argument("--data-root", default=None, help="data 根目录，默认=<repo_root>/data")
    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARN", "ERROR"], help="日志等级")
    ap.add_argument("--raw-only", action="store_true", help="只记录 raw 行，跳过 parsed/TB")
    ap.add_argument("--stat-every", type=float, default=5.0, help="统计打印间隔秒数（0=不打印）")

    # DVL 参数设置：发射呼率 & 平均次数
    ap.add_argument("--ping-rate", type=int, default=10, help="PR 发射呼率 (0..99)")
    ap.add_argument("--avg-count", type=int, default=10, help="PM 平均次数 (0..99)")

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
    data_root = Path(args.data_root).resolve() if args.data_root else default_data_root()
    data_root.mkdir(parents=True, exist_ok=True)
    day_dvl_dir = make_day_dvl_dir(data_root)

    logging.info(f"[DVL] data_root={data_root}")
    logging.info(f"[DVL] 今日目录={day_dvl_dir}")

    diag = SensorRunDiagnostics(day_dvl_dir, "dvl_capture", SENSOR_ID)
    diag.set_meta("port", args.port)
    diag.set_meta("baud", args.baud)
    diag.set_meta("parity", args.parity)
    diag.set_meta("stopbits", args.stopbits)
    diag.set_meta("raw_only", args.raw_only)
    diag.set_meta("ping_rate", args.ping_rate)
    diag.set_meta("avg_count", args.avg_count)
    diag.set_meta("data_root", str(data_root))

    port_path = Path(args.port)
    if not port_path.exists():
        diag.warn_once(
            "dvl_serial_path_missing",
            "configured DVL serial path does not exist before open",
            port=args.port,
        )

    # 日志器：完整 raw + parsed 日志
    dvl_logger = DVLLogger(str(day_dvl_dir))
    # TB：主导航 + 水团
    nav_writer = NavStateWriterTB(day_dvl_dir)
    water_writer = WaterMassWriterTB(day_dvl_dir)

    diag.note_file("raw_lines_csv", dvl_logger.raw_file.name)
    diag.note_file("parsed_csv", dvl_logger.parsed_file.name)
    diag.note_file("nav_state_tb_csv", nav_writer._path)
    diag.note_file("watermass_tb_csv", water_writer._path)

    # 统计
    stats = {"raw": 0, "parsed": 0, "watermass": 0, "nav": 0, "start_t": time.time()}
    last_stat_t = time.time()

    # 退出竞态保护：如果 stop_listening 不能保证 join，
    # 回调在 close 后仍被触发会导致写已关闭文件。
    closing = {"v": False}

    # TB 落盘开关：safe_start 完成之后才置 True
    started_tb = {"ok": False}
    runtime_error = {"msg": ""}

    def _safe_call(tag: str, fn, *a, **kw):
        """回调里用：任何异常都不能把监听线程打崩。"""
        try:
            return fn(*a, **kw)
        except Exception as e:
            logging.warning(f"[DVL][{tag}] callback error: {e}")
            diag.warn(
                "dvl_callback_write_error",
                "dvl callback write failed",
                tag=tag,
                error=str(e),
            )
            return None

    def _on_driver_event(level: str, code: str, message: str, **details):
        diag.bump(f"event_{level}")
        diag.event(level, code, message, **details)

    # ---- 回调 ----
    def on_raw(_ts_unused: float, line: str, note: str):
        if closing["v"]:
            return

        ts = stamp("dvl0", SensorKind.DVL)
        est_ns = ts.corrected_time_ns
        est_s = est_ns / 1e9

        _safe_call("RAW_STORE", dvl_logger.store_raw_line, est_s, line, note)
        stats["raw"] += 1
        diag.bump("raw_lines")
        if note.startswith("CMD"):
            diag.bump("cmd_lines")
        elif note == "RESP":
            diag.bump("resp_lines")

    def on_parsed(d: DVLData):
        if closing["v"]:
            return

        # 1) 完整解析日志
        _safe_call("PARSED_STORE", dvl_logger.store_parsed, d)

        # 2) 统一时间基
        ts = stamp("dvl0", SensorKind.DVL)
        mono_ns = ts.host_time_ns
        est_ns = ts.corrected_time_ns

        # 3) TB 表（仅在 safe_start 完成后写）
        if not args.raw_only and started_tb["ok"]:
            # 主导航
            _safe_call("NAV_TB", nav_writer.write_sample, mono_ns, est_ns, d)
            # 水团
            _safe_call("WATER_TB", water_writer.write_sample, mono_ns, est_ns, d)

        stats["parsed"] += 1
        diag.bump("parsed_frames")
        if d.is_water_mass:
            stats["watermass"] += 1
            diag.bump("watermass_frames")
        else:
            stats["nav"] += 1
            diag.bump("nav_frames")
        if d.valid is False:
            diag.bump("invalid_flag_frames")
        elif d.valid is None:
            diag.bump("unknown_valid_frames")

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
        on_event=_on_driver_event,
    )

    # 先启动监听，确保命令/应答也被 raw 记录
    if not dvl.start_listening(raw_only=args.raw_only):
        logging.error("[DVL] start_listening 失败，退出。")
        diag.finalize(
            "open_failed",
            "dvl start_listening failed before any capture",
            driver_stats=dvl.stats_dict(),
        )
        closing["v"] = True
        dvl_logger.close()
        nav_writer.close()
        water_writer.close()
        return

    # ========= 安全启动序列：CZ -> CS -> PR -> PM =========
    try:
        logging.info(
            f"[DVL] 安全启动序列：CZ -> CS -> PR({args.ping_rate}) -> PM({args.avg_count})"
        )
        # 依赖你在 io.py 中实现的 safe_start
        dvl.safe_start(
            ping_rate=args.ping_rate,
            avg_count=args.avg_count,
            sleep_after_cz=1.0,
            sleep_after_cs=0.5,
        )
        started_tb["ok"] = True
        diag.info(
            "dvl_safe_start_ok",
            "dvl safe_start finished",
            ping_rate=args.ping_rate,
            avg_count=args.avg_count,
        )
        logging.info("[DVL] safe_start 完成，开始记录 TB 状态数据。")
    except Exception as e:
        logging.error(f"[DVL] safe_start 失败：{e}")
        logging.error("[DVL] 为安全起见，立即退出程序。")
        diag.error("dvl_safe_start_failed", "dvl safe_start failed", error=str(e))
        closing["v"] = True
        try:
            dvl.stop_listening()
        except Exception:
            pass
        dvl_logger.close()
        nav_writer.close()
        water_writer.close()
        diag.finalize(
            "safe_start_failed",
            f"dvl safe_start failed: {e}",
            driver_stats=dvl.stats_dict(),
        )
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
    try:
        while not stop_flag["stop"]:
            time.sleep(0.2)
            if args.stat_every > 0:
                now = time.time()
                if now - last_stat_t >= args.stat_every:
                    dt = now - stats["start_t"]
                    raw_rate = stats["raw"] / dt if dt > 0 else 0.0

                    if args.raw_only:
                        logging.info(
                            f"[STAT] raw={stats['raw']} ({raw_rate:.1f}/s), parsed=disabled, driver=({dvl.stats()})"
                        )
                    else:
                        parsed_rate = stats["parsed"] / dt if dt > 0 else 0.0
                        logging.info(
                            f"[STAT] raw={stats['raw']} ({raw_rate:.1f}/s), "
                            f"parsed={stats['parsed']} ({parsed_rate:.1f}/s), driver=({dvl.stats()})"
                        )
                    last_stat_t = now
    except KeyboardInterrupt:
        logging.info("[DVL] KeyboardInterrupt，准备退出…")
    except Exception as e:
        runtime_error["msg"] = str(e)
        logging.error(f"[DVL] 运行时异常：{e}", exc_info=True)
    finally:
        # 先置位，避免 close 后回调仍写文件
        closing["v"] = True

        # ========= 安全关停序列：CZ =========
        try:
            logging.info("[DVL] 发送 CZ（结束采集，关闭换能器）")
            # 这里假定 write_cmd_try 的签名为 (cmd2, arg)
            dvl.write_cmd_try("CZ", None)
        except Exception as e:
            logging.warning(f"[DVL] CZ 发送失败：{e}")
            diag.warn("dvl_safe_stop_failed", "dvl CZ on stop failed", error=str(e))
        time.sleep(0.5)
        # ====================================

        try:
            dvl.stop_listening()
        except Exception as e:
            diag.warn("dvl_stop_listening_failed", "dvl stop_listening failed", error=str(e))

        dvl_logger.close()
        nav_writer.close()
        water_writer.close()

        driver_stats = dvl.stats_dict()
        diag.set_meta("driver_stats", driver_stats)

        if runtime_error["msg"]:
            status = "runtime_error"
            message = f"dvl logger runtime error: {runtime_error['msg']}"
        elif stats["raw"] == 0:
            status = "empty_capture"
            message = "dvl capture finished without any raw line"
            diag.warn("dvl_empty_capture", message, port=args.port)
        elif (not args.raw_only) and stats["parsed"] == 0:
            status = "no_parsed_frames"
            message = "dvl capture got raw lines but parsed zero frame"
            diag.warn("dvl_no_parsed_frames", message)
        else:
            status = "ok"
            message = "dvl capture finished with data"

        diag.finalize(
            status,
            message,
            raw_count=stats["raw"],
            parsed_count=stats["parsed"],
            nav_count=stats["nav"],
            watermass_count=stats["watermass"],
            runtime_s=round(max(0.0, time.time() - stats["start_t"]), 3),
        )
        logging.info("[DVL] logger 完整退出。")


if __name__ == "__main__":
    main()
