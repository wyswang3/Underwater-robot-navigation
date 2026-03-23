#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
apps/acquire/sensor_capture_launcher.py

薄总开关：一次拉起 IMU / DVL / Volt32 三套采集脚本。

设计边界：
1. 不侵入各传感器采集脚本的内部逻辑，只做进程编排。
2. 每个子脚本仍然各自负责 CSV / events / session summary 落盘。
3. launcher 自己只补一份总览 manifest 与 session summary，便于操作员知道
   “这次一键启动到底拉起了谁、是谁先失败、剩余进程是否已被收掉”。
"""

from __future__ import annotations

import argparse
import json
import logging
import shlex
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from types import FrameType
from typing import Dict, List, Optional, Sequence

sys.path.append(str(Path(__file__).resolve().parents[2]))

from uwnav.io.acquisition_diagnostics import SensorRunDiagnostics


@dataclass
class ChildSpec:
    name: str
    command: List[str]


@dataclass
class RunningChild:
    name: str
    command: List[str]
    process: subprocess.Popen


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def default_data_root() -> Path:
    return repo_root() / "data"


def make_day_launcher_dir(data_root: Path) -> Path:
    day = time.strftime("%Y-%m-%d")
    out_dir = data_root / day / "launcher"
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir


def _parse_sensor_list(raw: str) -> List[str]:
    allowed = {"imu", "dvl", "volt"}
    items = [item.strip().lower() for item in raw.split(",") if item.strip()]
    if not items:
        raise argparse.ArgumentTypeError("至少要选择一个传感器")

    unique: List[str] = []
    for item in items:
        if item not in allowed:
            raise argparse.ArgumentTypeError(f"未知传感器：{item}")
        if item not in unique:
            unique.append(item)
    return unique


def _quoted(cmd: Sequence[str]) -> str:
    return " ".join(shlex.quote(part) for part in cmd)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="One-shot launcher for IMU / DVL / Volt32 acquisition scripts"
    )
    ap.add_argument(
        "--sensors",
        type=_parse_sensor_list,
        default=["imu", "dvl", "volt"],
        help="要启动的传感器，逗号分隔，默认 imu,dvl,volt",
    )
    ap.add_argument("--data-root", default=None, help="统一 data 根目录，默认 <repo_root>/data")
    ap.add_argument("--log-level", default="INFO", choices=["DEBUG", "INFO", "WARN", "ERROR"], help="launcher 日志等级")
    ap.add_argument("--child-log-level", default="INFO", choices=["DEBUG", "INFO", "WARN", "ERROR"], help="子脚本日志等级")
    ap.add_argument("--child-stat-every", type=float, default=5.0, help="透传给子脚本的统计打印周期，默认 5 秒")
    ap.add_argument("--launcher-stat-every", type=float, default=5.0, help="launcher 自身统计打印周期，默认 5 秒")
    ap.add_argument("--run-seconds", type=float, default=0.0, help="仅用于 smoke/test；>0 时到时自动发起停机")
    ap.add_argument("--stop-timeout-s", type=float, default=5.0, help="停机等待子进程退出的超时时间")

    ap.add_argument("--imu-port", default="/dev/ttyUSB0", help="IMU 串口")
    ap.add_argument("--imu-baud", type=int, default=230400, help="IMU 波特率")
    ap.add_argument("--imu-addr", type=lambda x: int(x, 0), default=0x50, help="IMU Modbus 地址")

    ap.add_argument("--dvl-port", default="/dev/ttyACM0", help="DVL 串口")
    ap.add_argument("--dvl-baud", type=int, default=115200, help="DVL 波特率")
    ap.add_argument("--dvl-ping-rate", type=int, default=10, help="DVL PR 参数")
    ap.add_argument("--dvl-avg-count", type=int, default=10, help="DVL PM 参数")
    ap.add_argument("--dvl-raw-only", action="store_true", help="只拉起 DVL raw 记录，不写 parsed/TB")

    ap.add_argument("--volt-port", default="/dev/ttyUSB1", help="Volt32 串口")
    ap.add_argument("--volt-baud", type=int, default=115200, help="Volt32 波特率")
    ap.add_argument("--volt-channels", type=int, default=16, help="Volt32 通道数")
    return ap.parse_args()


def build_child_specs(args: argparse.Namespace, data_root: Path) -> List[ChildSpec]:
    py = sys.executable
    base = repo_root() / "apps" / "acquire"
    child_specs: List[ChildSpec] = []

    if "imu" in args.sensors:
        child_specs.append(
            ChildSpec(
                name="imu",
                command=[
                    py,
                    str(base / "imu_logger.py"),
                    "--port", args.imu_port,
                    "--baud", str(args.imu_baud),
                    "--addr", hex(args.imu_addr),
                    "--data-root", str(data_root),
                    "--log-level", args.child_log_level,
                    "--stat-every", str(args.child_stat_every),
                ],
            )
        )

    if "dvl" in args.sensors:
        command = [
            py,
            str(base / "DVL_logger.py"),
            "--port", args.dvl_port,
            "--baud", str(args.dvl_baud),
            "--ping-rate", str(args.dvl_ping_rate),
            "--avg-count", str(args.dvl_avg_count),
            "--data-root", str(data_root),
            "--log-level", args.child_log_level,
            "--stat-every", str(args.child_stat_every),
        ]
        if args.dvl_raw_only:
            command.append("--raw-only")
        child_specs.append(ChildSpec(name="dvl", command=command))

    if "volt" in args.sensors:
        child_specs.append(
            ChildSpec(
                name="volt",
                command=[
                    py,
                    str(base / "Volt32_logger.py"),
                    "--port", args.volt_port,
                    "--baud", str(args.volt_baud),
                    "--channels", str(args.volt_channels),
                    "--data-root", str(data_root),
                    "--log-level", args.child_log_level,
                    "--stat-every", str(args.child_stat_every),
                    "--debug-raw-sniff", "0",
                ],
            )
        )

    return child_specs


def _write_manifest(out_dir: Path, child_specs: Sequence[ChildSpec], args: argparse.Namespace) -> Path:
    ts = time.strftime("%Y%m%d_%H%M%S")
    path = out_dir / f"sensor_launcher_manifest_{ts}.json"
    payload = {
        "created_wall_time": time.strftime("%Y-%m-%d %H:%M:%S"),
        "repo_root": str(repo_root()),
        "data_root": str(Path(args.data_root).resolve()) if args.data_root else str(default_data_root()),
        "selected_sensors": list(args.sensors),
        "children": [
            {
                "name": child.name,
                "command": child.command,
                "command_shell": _quoted(child.command),
            }
            for child in child_specs
        ],
    }
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    return path


def _launch_children(child_specs: Sequence[ChildSpec], diag: SensorRunDiagnostics) -> List[RunningChild]:
    running: List[RunningChild] = []
    for child in child_specs:
        diag.info(
            "launcher_child_starting",
            "launcher is starting child capture script",
            sensor=child.name,
            command=child.command,
        )
        process = subprocess.Popen(child.command, cwd=str(repo_root()))
        running.append(RunningChild(name=child.name, command=child.command, process=process))
        diag.info(
            "launcher_child_started",
            "launcher started child capture script",
            sensor=child.name,
            pid=process.pid,
        )
    return running


def _stop_children(children: Sequence[RunningChild], diag: SensorRunDiagnostics, timeout_s: float) -> Dict[str, Optional[int]]:
    exit_codes: Dict[str, Optional[int]] = {}

    for child in children:
        rc = child.process.poll()
        if rc is None:
            diag.info("launcher_child_stopping", "launcher is sending SIGTERM to child", sensor=child.name, pid=child.process.pid)
            try:
                child.process.terminate()
            except Exception as exc:
                diag.warn("launcher_child_terminate_failed", "failed to terminate child", sensor=child.name, error=str(exc))
        else:
            exit_codes[child.name] = rc

    deadline = time.time() + max(0.1, float(timeout_s))
    for child in children:
        if child.name in exit_codes:
            continue
        remaining = max(0.1, deadline - time.time())
        try:
            exit_codes[child.name] = child.process.wait(timeout=remaining)
        except subprocess.TimeoutExpired:
            diag.warn("launcher_child_kill", "child did not exit after SIGTERM; sending SIGKILL", sensor=child.name, pid=child.process.pid)
            child.process.kill()
            exit_codes[child.name] = child.process.wait(timeout=1.0)

    return exit_codes


def main() -> int:
    args = parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s.%(msecs)03d %(levelname)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    data_root = Path(args.data_root).resolve() if args.data_root else default_data_root()
    data_root.mkdir(parents=True, exist_ok=True)
    launcher_dir = make_day_launcher_dir(data_root)

    diag = SensorRunDiagnostics(launcher_dir, "sensor_launcher", "SENSOR_LAUNCHER")
    diag.set_meta("selected_sensors", list(args.sensors))
    diag.set_meta("data_root", str(data_root))
    diag.set_meta("child_log_level", args.child_log_level)
    diag.set_meta("child_stat_every", args.child_stat_every)
    diag.set_meta("stop_timeout_s", args.stop_timeout_s)
    diag.set_meta("run_seconds", args.run_seconds)

    child_specs = build_child_specs(args, data_root)
    if not child_specs:
        diag.finalize("invalid_args", "no child sensor selected")
        logging.error("[LAUNCHER] 没有可启动的传感器。")
        return 2

    manifest_path = _write_manifest(launcher_dir, child_specs, args)
    diag.note_file("manifest_json", manifest_path)

    running_children = _launch_children(child_specs, diag)
    stop_flag = {"requested": False, "reason": "signal"}
    start_t = time.time()
    last_stat_t = start_t

    def _signal_handler(sig: int, frame: Optional[FrameType]) -> None:
        logging.info(f"[LAUNCHER] 收到信号 {sig}，准备停止全部采集…")
        stop_flag["requested"] = True
        stop_flag["reason"] = f"signal_{sig}"

    signal.signal(signal.SIGINT, _signal_handler)
    signal.signal(signal.SIGTERM, _signal_handler)

    final_status = "stopped"
    final_message = "launcher stopped all child capture scripts"
    failing_child: Optional[str] = None

    try:
        while True:
            time.sleep(0.2)

            if args.run_seconds > 0 and (time.time() - start_t) >= args.run_seconds:
                stop_flag["requested"] = True
                stop_flag["reason"] = "run_seconds_elapsed"

            for child in running_children:
                rc = child.process.poll()
                if rc is None:
                    continue
                if stop_flag["requested"]:
                    continue
                failing_child = child.name
                final_status = "child_failed"
                final_message = f"child capture script exited unexpectedly: {child.name} rc={rc}"
                diag.error(
                    "launcher_child_exited_early",
                    "child capture script exited before launcher stop was requested",
                    sensor=child.name,
                    exit_code=rc,
                )
                stop_flag["requested"] = True
                stop_flag["reason"] = f"child_failed_{child.name}"
                break

            if stop_flag["requested"]:
                break

            if args.launcher_stat_every > 0:
                now = time.time()
                if now - last_stat_t >= args.launcher_stat_every:
                    status_parts = []
                    for child in running_children:
                        rc = child.process.poll()
                        status_parts.append(f"{child.name}={'running' if rc is None else rc}")
                    logging.info(f"[LAUNCHER-STAT] {' '.join(status_parts)}")
                    last_stat_t = now
    except KeyboardInterrupt:
        stop_flag["requested"] = True
        stop_flag["reason"] = "keyboard_interrupt"
    finally:
        exit_codes = _stop_children(running_children, diag, timeout_s=args.stop_timeout_s)
        diag.set_meta("child_exit_codes", exit_codes)

    if failing_child is None and stop_flag["reason"] == "run_seconds_elapsed":
        final_status = "stopped"
        final_message = "launcher stopped children after run_seconds elapsed"
    elif failing_child is None and stop_flag["reason"].startswith("signal"):
        final_status = "stopped"
        final_message = "launcher stopped children on signal"
    elif failing_child is None and stop_flag["reason"] == "keyboard_interrupt":
        final_status = "stopped"
        final_message = "launcher stopped children on keyboard interrupt"

    diag.finalize(
        final_status,
        final_message,
        runtime_s=round(max(0.0, time.time() - start_t), 3),
        stop_reason=stop_flag["reason"],
        child_count=len(running_children),
    )

    if final_status == "child_failed":
        logging.error(f"[LAUNCHER] {final_message}")
        return 1

    logging.info("[LAUNCHER] 全部子采集脚本已停止。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
