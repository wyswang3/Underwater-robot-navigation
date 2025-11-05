#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
启动两个采集脚本于同一 tmux 会话，并允许命令行灵活指定串口与波特率。

示例（在 apps 目录内执行）：
  python3 tmux_telemetry_manager.py \
    --imu-script imu_realtime_pipeline.py --imu-port /dev/ttyUSB1 --imu-baud 230400 \
    --volt-script volt32_logger.py        --volt-port /dev/ttyUSB0 --volt-baud 115200

查看可用串口：
  python3 tmux_telemetry_manager.py --list-ports
"""

import os
import sys
import shutil
import subprocess
import threading
import time
import glob
import argparse
from typing import List

SESSION_DEFAULT = "data_session"

# ---------- tmux 基础 ----------
def run_tmux(cmd_list, err_msg, check=True, capture=False):
    try:
        return subprocess.run(
            cmd_list, check=check,
            stdout=subprocess.PIPE if capture else None,
            stderr=subprocess.PIPE if capture else None,
            text=True
        )
    except subprocess.CalledProcessError as e:
        print(f"{err_msg}: {e}")
        if e.stdout: print(e.stdout)
        if e.stderr: print(e.stderr)
        sys.exit(1)

def tmux_exists() -> bool:
    return shutil.which("tmux") is not None

def session_exists(name: str) -> bool:
    res = subprocess.run(["tmux", "has-session", "-t", name],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return res.returncode == 0

def send_enter(target: str):
    run_tmux(["tmux", "send-keys", "-t", target, "C-m"], "发送回车失败", check=True)

def graceful_stop_all(session: str, try_soft_exit_key: str = "s"):
    """向每个 pane 发送 Ctrl+C；再尝试 's'+回车；最后 kill 会话。"""
    try:
        res = run_tmux(["tmux", "list-panes", "-t", session, "-F", "#{pane_id}"],
                       "列出 panes 失败", capture=True)
        panes = [line.strip() for line in res.stdout.splitlines() if line.strip()]
    except Exception:
        panes = []
    if not panes:
        run_tmux(["tmux", "kill-session", "-t", session], "终止 tmux 会话失败", check=False)
        print("tmux 会话已终止（空）。")
        return

    print("向各 pane 发送 Ctrl+C …")
    for pid in panes:
        run_tmux(["tmux", "send-keys", "-t", pid, "C-c"], f"向 {pid} 发送 Ctrl+C 失败", check=False)

    if try_soft_exit_key:
        for pid in panes:
            run_tmux(["tmux", "send-keys", "-t", pid, try_soft_exit_key], "发送退出键失败", check=False)
            send_enter(pid)

    time.sleep(1.5)
    run_tmux(["tmux", "kill-session", "-t", session], "终止 tmux 会话失败", check=False)
    print("tmux 会话已终止。")

# ---------- 串口与路径工具 ----------
def list_serial_ports() -> List[str]:
    cands = sorted(set(
        glob.glob("/dev/ttyUSB*") +
        glob.glob("/dev/ttyACM*") +
        glob.glob("/dev/ttyAMA*")
    ))
    by_ids = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_ids:
        print("可用串口(by-id)：")
        for p in by_ids: print("  ", p, "->", os.path.realpath(p))
    print("可用串口：")
    for p in cands: print("  ", p)
    if not cands and not by_ids:
        print("(未发现串口设备)")
    return cands

def port_exists(p: str) -> bool:
    return bool(p) and os.path.exists(p)

def here_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))

def repo_root_dir() -> str:
    # 若当前在 apps/ 下，仓库根为父目录，否则为当前位置
    here = here_dir()
    return os.path.dirname(here) if os.path.basename(here) == "apps" else here

def resolve_script_abs(script: str) -> str:
    """将脚本路径解析为绝对路径，避免 apps/apps 重复。"""
    here = here_dir()
    if os.path.isabs(script):
        return script
    # 如果当前就在 apps 下，且传入以 apps/ 开头，去掉一次前缀
    if os.path.basename(here) == "apps" and script.startswith("apps/"):
        script = script.split("apps/", 1)[1]
    cand = os.path.normpath(os.path.join(here, script))
    if os.path.exists(cand):
        return cand
    cand2 = os.path.normpath(os.path.join(here, "..", script))
    if os.path.exists(cand2):
        return cand2
    return cand  # 让报错更直观

def build_tmux_command(env: dict, python_cmd: str, script: str, cli_args: List[str]) -> str:
    """
    在 tmux pane 中执行的完整命令：
      注入环境变量（含 PYTHONPATH），直接运行“绝对路径脚本” + 参数（不再 cd）
    """
    script_abs = resolve_script_abs(script)
    exports = env.copy()
    # 注入 PYTHONPATH=仓库根
    rr = repo_root_dir()
    prev_pp = os.environ.get("PYTHONPATH", "")
    exports["PYTHONPATH"] = f"{rr}:{prev_pp}" if prev_pp else rr
    export_cmd = " ".join([f'{k}="{v}"' for k, v in exports.items() if v is not None])
    cli = " ".join(cli_args)
    prefix = f"{export_cmd} " if export_cmd else ""
    return f'{prefix}{python_cmd} -u "{script_abs}" {cli}'.strip()

# ---------- 主程序 ----------
def main():
    # 根据自己是否在 apps/ 下决定默认脚本路径
    in_apps = (os.path.basename(here_dir()) == "apps")
    default_imu  = "imu_realtime_pipeline.py" if in_apps else "apps/imu_realtime_pipeline.py"
    default_volt = "volt32_logger.py"         if in_apps else "apps/volt32_logger.py"

    ap = argparse.ArgumentParser(description="tmux telemetry manager (IMU + Volt32)")
    ap.add_argument("--session", default=SESSION_DEFAULT, help=f"tmux 会话名（默认 {SESSION_DEFAULT}）")
    ap.add_argument("--imu-script",  default=default_imu,  help="IMU 采集脚本路径（相对/绝对均可）")
    ap.add_argument("--volt-script", default=default_volt, help="电压采集脚本路径（相对/绝对均可）")
    ap.add_argument("--imu-port",  default="/dev/ttyUSB1", help="IMU 串口（默认 /dev/ttyUSB1）")
    ap.add_argument("--imu-baud",  default="230400",       help="IMU 波特率（默认 230400）")
    ap.add_argument("--volt-port", default="/dev/ttyUSB0", help="电压卡串口（默认 /dev/ttyUSB0）")
    ap.add_argument("--volt-baud", default="115200",       help="电压卡波特率（默认 115200）")
    ap.add_argument("--list-ports", action="store_true",   help="仅列出串口后退出")
    ap.add_argument("--attach", action="store_true",       help="启动后自动 attach 到 tmux")
    ap.add_argument("--python", default=sys.executable,    help="Python 解释器（默认当前解释器）")
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    if not tmux_exists():
        print("未找到 tmux，请先安装（apt install tmux）。")
        sys.exit(1)

    # 若已有旧会话，先清理，确保重复运行不叠加 pane
    if session_exists(args.session):
        print(f"检测到已存在会话 '{args.session}'，先清理…")
        run_tmux(["tmux", "kill-session", "-t", args.session], "清理旧会话失败", check=False)

    # 创建会话并水平分割为两窗格
    run_tmux(["tmux", "new-session", "-d", "-s", args.session], "创建 tmux 会话失败")
    run_tmux(["tmux", "split-window", "-h", "-t", f"{args.session}:0"], "水平分割窗格失败")
    run_tmux(["tmux", "select-layout", "-t", args.session, "even-horizontal"], "调整窗格布局失败")
    run_tmux(["tmux", "set-option", "-t", args.session, "remain-on-exit", "on"],
             "设置 remain-on-exit 失败", check=False)
    print(f"tmux 会话 '{args.session}' 创建成功。")

    # pane 0.0：电压卡；pane 0.1：IMU
    volt_env = {"VOLT_PORT": args.volt_port, "VOLT_BAUD": args.volt_baud}
    imu_env  = {"IMU_PORT":  args.imu_port,  "IMU_BAUD":  args.imu_baud}

    volt_cli = [f"--port {args.volt_port}", f"--baud {args.volt_baud}"]
    imu_cli  = [f"--port {args.imu_port}",  f"--baud {args.imu_baud}"]

    cmd_volt = build_tmux_command(volt_env, args.python, args.volt_script, volt_cli)
    cmd_imu  = build_tmux_command(imu_env,  args.python, args.imu_script,  imu_cli)

    # 端口存在才启动；先 IMU（需要轮询），再电压卡（主动发）
    start_imu  = port_exists(args.imu_port)
    start_volt = port_exists(args.volt_port)
    if not start_imu:
        print(f"[WARN] IMU 端口不存在：{args.imu_port}，跳过 IMU 启动。")
    if not start_volt:
        print(f"[WARN] 电压卡端口不存在：{args.volt_port}，跳过电压卡启动。")

    if start_imu:
        run_tmux(["tmux", "send-keys", "-t", f"{args.session}:0.1", cmd_imu], "在 pane 0.1 发送命令失败")
        send_enter(f"{args.session}:0.1")
        print(f"[pane 0.1] 启动：{cmd_imu}")
        time.sleep(0.5)

    if start_volt:
        run_tmux(["tmux", "send-keys", "-t", f"{args.session}:0.0", cmd_volt], "在 pane 0.0 发送命令失败")
        send_enter(f"{args.session}:0.0")
        print(f"[pane 0.0] 启动：{cmd_volt}")

    if not (start_imu or start_volt):
        print("[ERR] 没有可用端口可启动，结束会话。")
        graceful_stop_all(args.session)
        sys.exit(1)

    print("\n=== 已启动 ===")
    print(f"- 查看窗口： tmux attach -t {args.session}")
    print("- 在此终端输入 's' 优雅停止并关闭会话；或使用 Ctrl+C。\n")

    # 本终端监听 's' 触发停止
    def input_listener():
        try:
            while True:
                user_input = input("输入 's' 停止所有采集并关闭 tmux 会话: ").strip().lower()
                if user_input == 's':
                    print("收到退出指令，开始优雅停止…")
                    graceful_stop_all(args.session, try_soft_exit_key="s")
                    break
                else:
                    print("无效输入，请输入 's'。")
        except (KeyboardInterrupt, EOFError):
            print("\n收到中断，尝试优雅停止…")
            graceful_stop_all(args.session, try_soft_exit_key="s")

    t = threading.Thread(target=input_listener, daemon=True)
    t.start()

    if args.attach:
        os.execvp("tmux", ["tmux", "attach-session", "-t", args.session])
    else:
        while t.is_alive():
            time.sleep(1)
        print("管理脚本退出。")

if __name__ == "__main__":
    main()

