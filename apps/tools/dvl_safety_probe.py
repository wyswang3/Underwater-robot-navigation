#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/tools/dvl_safety_probe.py

DVL 安全启动/停机探针：
- 阶段 A：不发任何指令，仅监听，看是否有回传
- 阶段 B：发送 CS，观察是否开始回传
- 阶段 C：发送 CZ，观察是否停止回传

注意：为避免空气中误 ping 损伤换能器，默认在启动时先发一次 CZ（可用参数关闭）。
退出时无条件再发一次 CZ。
"""

from __future__ import annotations

import argparse
import logging
import signal
import time
from dataclasses import dataclass

from uwnav.drivers.dvl.hover_h1000.io import DVLSerialInterface


@dataclass
class PhaseStat:
    name: str
    t0: float
    raw: int = 0
    parsed: int = 0

    def rate_raw(self, now: float) -> float:
        dt = max(1e-6, now - self.t0)
        return self.raw / dt

    def rate_parsed(self, now: float) -> float:
        dt = max(1e-6, now - self.t0)
        return self.parsed / dt


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="DVL safety probe (no-CS / CS / CZ)")
    ap.add_argument("--port", default="/dev/ttyACM0", help="DVL 串口 (e.g. /dev/ttyUSB2)")
    ap.add_argument("--baud", type=int, default=115200, help="波特率")
    ap.add_argument("--parity", default="N", choices=["N", "E", "O"], help="校验位")
    ap.add_argument("--stopbits", type=float, default=1.0, choices=[1.0, 2.0], help="停止位")
    ap.add_argument("--rtscts", action="store_true", help="启用 RTS/CTS")
    ap.add_argument("--dsrdtr", action="store_true", help="启用 DSR/DTR")
    ap.add_argument("--xonxoff", action="store_true", help="启用 XON/XOFF")
    ap.add_argument("--no-dtr", action="store_true", help="强制 DTR 低电平")
    ap.add_argument("--no-rts", action="store_true", help="强制 RTS 低电平")

    ap.add_argument("--log-level", default="INFO",
                    choices=["DEBUG", "INFO", "WARN", "ERROR"], help="日志等级")

    ap.add_argument("--phase-a-sec", type=float, default=5.0,
                    help="阶段A(不发指令)监听时长")
    ap.add_argument("--phase-b-sec", type=float, default=5.0,
                    help="阶段B(CS后)监听时长")
    ap.add_argument("--phase-c-sec", type=float, default=5.0,
                    help="阶段C(CZ后)监听时长")

    ap.add_argument("--no-initial-cz", action="store_true",
                    help="禁用启动时先发 CZ（不推荐，除非你明确要验证“绝对不发任何指令”）")
    ap.add_argument("--raw-only", action="store_true",
                    help="只统计 raw，不走 parsed（若解析不稳定可开）")

    ap.add_argument("--print-every", type=float, default=1.0,
                    help="每隔多少秒打印一次阶段统计")
    return ap.parse_args()


def main():
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    stop = {"v": False}
    closing = {"v": False}
    current_phase = {"stat": None}  # type: ignore

    def on_sig(sig, frame):
        logging.info(f"[PROBE] 收到信号 {sig}，准备退出…")
        stop["v"] = True

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    def safe_print_stat():
        st = current_phase["stat"]
        if st is None:
            return
        now = time.time()
        if args.raw_only:
            logging.info(f"[{st.name}] raw={st.raw} ({st.rate_raw(now):.1f}/s)")
        else:
            logging.info(
                f"[{st.name}] raw={st.raw} ({st.rate_raw(now):.1f}/s), "
                f"parsed={st.parsed} ({st.rate_parsed(now):.1f}/s)"
            )

    # 回调：只计数（避免任何 I/O，确保线程安全与稳定）
    def on_raw(_ts_unused: float, line: str, note: str):
        if closing["v"]:
            return
        st = current_phase["stat"]
        if st is not None:
            st.raw += 1

    def on_parsed(d):
        if closing["v"]:
            return
        st = current_phase["stat"]
        if st is not None:
            st.parsed += 1

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
        logging.error("[PROBE] start_listening 失败，退出。")
        return

    try:
        # 默认安全：先发 CZ，避免设备上电默认 ping
        if not args.no_initial_cz:
            try:
                logging.info("[PROBE] (safe) 启动先发 CZ，确保换能器关闭")
                dvl.write_cmd_try("CZ", "")
                time.sleep(0.5)
            except Exception as e:
                logging.warning(f"[PROBE] 初始 CZ 发送失败：{e}")

        # ---------------- Phase A: no command ----------------
        logging.info("[PROBE] Phase A: 不下发任何指令，仅监听")
        current_phase["stat"] = PhaseStat("PHASE-A(NO-CMD)", time.time())
        t_end = time.time() + args.phase_a_sec
        next_print = time.time() + args.print_every
        while not stop["v"] and time.time() < t_end:
            time.sleep(0.05)
            if time.time() >= next_print:
                safe_print_stat()
                next_print = time.time() + args.print_every
        safe_print_stat()

        # ---------------- Phase B: send CS ----------------
        logging.info("[PROBE] Phase B: 下发 CS，观察是否开始回传")
        try:
            dvl.write_cmd_try("CS", "")
        except Exception as e:
            logging.error(f"[PROBE] CS 发送失败：{e}")
        time.sleep(0.2)

        current_phase["stat"] = PhaseStat("PHASE-B(AFTER-CS)", time.time())
        t_end = time.time() + args.phase_b_sec
        next_print = time.time() + args.print_every
        while not stop["v"] and time.time() < t_end:
            time.sleep(0.05)
            if time.time() >= next_print:
                safe_print_stat()
                next_print = time.time() + args.print_every
        safe_print_stat()

        # ---------------- Phase C: send CZ ----------------
        logging.info("[PROBE] Phase C: 下发 CZ，观察是否停机/回传下降")
        try:
            dvl.write_cmd_try("CZ", "")
        except Exception as e:
            logging.error(f"[PROBE] CZ 发送失败：{e}")
        time.sleep(0.2)

        current_phase["stat"] = PhaseStat("PHASE-C(AFTER-CZ)", time.time())
        t_end = time.time() + args.phase_c_sec
        next_print = time.time() + args.print_every
        while not stop["v"] and time.time() < t_end:
            time.sleep(0.05)
            if time.time() >= next_print:
                safe_print_stat()
                next_print = time.time() + args.print_every
        safe_print_stat()

        # ---------------- Summary ----------------
        logging.info("[PROBE] 完成。请对比三段 raw/parsed 速率：")
        logging.info("        A 段若有明显回传：说明无需 CS 也在输出/启动（风险更高，需默认 CZ 防护）。")
        logging.info("        B 段回传上升：说明 CS 确实启动。")
        logging.info("        C 段回传下降到接近 0：说明 CZ 能停机。")

    finally:
        closing["v"] = True
        # 退出无条件 CZ（安全兜底）
        try:
            logging.info("[PROBE] (safe) 退出前发送 CZ，确保换能器关闭")
            dvl.write_cmd_try("CZ", "")
            time.sleep(0.2)
        except Exception as e:
            logging.warning(f"[PROBE] 退出 CZ 发送失败：{e}")

        try:
            dvl.stop_listening()
        except Exception:
            pass

        logging.info("[PROBE] 退出完成。")


if __name__ == "__main__":
    main()
