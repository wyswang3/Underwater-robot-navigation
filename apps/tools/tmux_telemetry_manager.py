#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tmux telemetry manager (IMU + DVL + Volt)

支持三路传感器同步启动：
- 窗口 imu  : IMU 采集脚本
- 窗口 dvl  : DVL 采集脚本
- 窗口 volt : 电压采集脚本

示例：
  python3 tmux_telemetry_manager.py \
    --imu-port /dev/ttyUSB1 --imu-baud 230400 \
    --dvl-port /dev/ttyUSB2 --dvl-baud 115200 \
    --volt-port /dev/ttyUSB0 --volt-baud 115200 \
    --attach

查看串口：
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


# ==================== tmux 基础 ====================
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
        if e.stdout:
            print(e.stdout)
        if e.stderr:
            print(e.stderr)
        sys.exit(1)


def tmux_exists() -> bool:
    return shutil.which("tmux") is not None


def session_exists(name: str) -> bool:
    res = subprocess.run(
        ["tmux", "has-session", "-t", name],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    return res.returncode == 0


def send_enter(target: str):
    run_tmux(["tmux", "send-keys", "-t", target, "C-m"], "发送回车失败")


def graceful_stop_all(session: str, try_soft_exit_key: str = "s"):
    """向每个 pane 发送 Ctrl+C，再尝试 soft-exit 键，最后 kill 会话。"""
    try:
        res = run_tmux(
            ["tmux", "list-panes", "-t", session, "-F", "#{pane_id}"],
            "列出 panes 失败",
            capture=True,
        )
        panes = [line.strip() for line in res.stdout.splitlines() if line.strip()]
    except Exception:
        panes = []

    if not panes:
        run_tmux(["tmux", "kill-session", "-t", session], "终止 tmux 会话失败", check=False)
        print("tmux 会话已终止（空会话）。")
        return

    print("向各 pane 发送 Ctrl+C …")
    for pid in panes:
        run_tmux(
            ["tmux", "send-keys", "-t", pid, "C-c"],
            f"向 {pid} 发送 Ctrl+C 失败",
            check=False,
        )

    if try_soft_exit_key:
        for pid in panes:
            run_tmux(
                ["tmux", "send-keys", "-t", pid, try_soft_exit_key],
                "发送退出键失败",
                check=False,
            )
            send_enter(pid)

    time.sleep(1.5)
    run_tmux(["tmux", "kill-session", "-t", session], "终止 tmux 会话失败", check=False)
    print("tmux 会话已终止。")


# ==================== 串口 / 路径工具 ====================
def list_serial_ports() -> List[str]:
    cands = sorted(
        set(
            glob.glob("/dev/ttyUSB*")
            + glob.glob("/dev/ttyACM*")
            + glob.glob("/dev/ttyAMA*")
        )
    )
    by_ids = sorted(glob.glob("/dev/serial/by-id/*"))
    if by_ids:
        print("可用串口(by-id)：")
        for p in by_ids:
            print("  ", p, "->", os.path.realpath(p))
    print("可用串口：")
    for p in cands:
        print("  ", p)
    if not cands and not by_ids:
        print("(未发现串口设备)")
    return cands


def port_exists(p: str) -> bool:
    return bool(p) and os.path.exists(p)


def here_dir() -> str:
    return os.path.dirname(os.path.abspath(__file__))


def repo_root_dir() -> str:
    here = here_dir()
    return os.path.dirname(here) if os.path.basename(here) == "apps" else here


def resolve_script_abs(script: str) -> str:
    """将脚本路径解析为绝对路径，避免 apps/apps 重复。"""
    here = here_dir()
    if os.path.isabs(script):
        return script
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
    rr = repo_root_dir()
    prev_pp = os.environ.get("PYTHONPATH", "")
    exports["PYTHONPATH"] = f"{rr}:{prev_pp}" if prev_pp else rr
    export_cmd = " ".join(
        [f'{k}="{v}"' for k, v in exports.items() if v is not None]
    )
    cli = " ".join(cli_args)
    prefix = f"{export_cmd} " if export_cmd else ""
    return f'{prefix}{python_cmd} -u "{script_abs}" {cli}'.strip()


# ==================== 主程序 ====================
def main():
    # 根据是否在 apps/ 下决定默认脚本相对路径
    in_apps = os.path.basename(here_dir()) == "apps"
    default_imu = "imu_data_verifier.py" if in_apps else "apps/tools/imu_data_verifier.py"
    default_dvl = "dvl_data_verifier.py"     if in_apps else "apps/tools/dvl_data_verifier.py"
    default_volt = "volt32_data_verifier.py"        if in_apps else "apps/tools/volt32_data_verifier.py"

    ap = argparse.ArgumentParser(description="tmux telemetry manager (IMU + DVL + Volt)")
    ap.add_argument("--session", default=SESSION_DEFAULT, help="tmux 会话名")
    ap.add_argument("--imu-script",  default=default_imu,  help="IMU 采集脚本路径")
    ap.add_argument("--dvl-script",  default=default_dvl,  help="DVL 采集脚本路径")
    ap.add_argument("--volt-script", default=default_volt, help="电压采集脚本路径")

    ap.add_argument("--imu-port",  default="/dev/ttyUSB1", help="IMU 串口")
    ap.add_argument("--imu-baud",  default="230400",       help="IMU 波特率")
    ap.add_argument("--dvl-port",  default="/dev/ttyUSB2", help="DVL 串口")
    ap.add_argument("--dvl-baud",  default="115200",       help="DVL 波特率")
    ap.add_argument("--volt-port", default="/dev/ttyUSB0", help="电压卡串口")
    ap.add_argument("--volt-baud", default="115200",       help="电压卡波特率")

    ap.add_argument("--list-ports", action="store_true", help="仅列出串口后退出")
    ap.add_argument("--attach", action="store_true", help="启动后自动 attach 到 tmux")
    ap.add_argument("--python", default=sys.executable, help="Python 解释器（默认当前解释器）")
    args = ap.parse_args()

    if args.list_ports:
        list_serial_ports()
        return

    if not tmux_exists():
        print("未找到 tmux，请先安装（例如：sudo apt install tmux）。")
        sys.exit(1)

    # 若已有同名会话，先清理，避免 pane/window 叠加
    if session_exists(args.session):
        print(f"检测到已存在会话 '{args.session}'，先清理…")
        run_tmux(["tmux", "kill-session", "-t", args.session], "清理旧会话失败", check=False)

    # ========== 三窗口布局 ==========
    # window 0: imu
    run_tmux(
        ["tmux", "new-session", "-d", "-s", args.session, "-n", "imu"],
        "创建 tmux 会话失败"
    )
    # window 1: dvl
    run_tmux(
        ["tmux", "new-window", "-t", args.session, "-n", "dvl"],
        "创建 DVL 窗口失败"
    )
    # window 2: volt
    run_tmux(
        ["tmux", "new-window", "-t", args.session, "-n", "volt"],
        "创建 Volt 窗口失败"
    )

    run_tmux(
        ["tmux", "set-option", "-t", args.session, "remain-on-exit", "on"],
        "设置 remain-on-exit 失败",
        check=False,
    )
    print(f"tmux 会话 '{args.session}' 已创建（三窗口：imu / dvl / volt）。")

    # pane 路径：session:window_name.pane_index
    pane_imu  = f"{args.session}:imu.0"
    pane_dvl  = f"{args.session}:dvl.0"
    pane_volt = f"{args.session}:volt.0"

    # 环境变量（可用于脚本内部读取）
    imu_env =  {"IMU_PORT": args.imu_port,  "IMU_BAUD": args.imu_baud}
    dvl_env =  {"DVL_PORT": args.dvl_port,  "DVL_BAUD": args.dvl_baud}
    volt_env = {"VOLT_PORT": args.volt_port, "VOLT_BAUD": args.volt_baud}

    # CLI 参数传给各脚本
    imu_cli  = [f"--port {args.imu_port}",  f"--baud {args.imu_baud}"]
    dvl_cli  = [f"--port {args.dvl_port}",  f"--baud {args.dvl_baud}"]
    volt_cli = [f"--port {args.volt_port}", f"--baud {args.volt_baud}"]

    # 构造 tmux 命令
    cmd_imu  = build_tmux_command(imu_env,  args.python, args.imu_script,  imu_cli)
    cmd_dvl  = build_tmux_command(dvl_env,  args.python, args.dvl_script,  dvl_cli)
    cmd_volt = build_tmux_command(volt_env, args.python, args.volt_script, volt_cli)

    # 检查端口存在性
    start_imu  = port_exists(args.imu_port)
    start_dvl  = port_exists(args.dvl_port)
    start_volt = port_exists(args.volt_port)

    if not start_imu:
        print(f"[WARN] IMU 端口不存在：{args.imu_port}，跳过 IMU 启动。")
    if not start_dvl:
        print(f"[WARN] DVL 端口不存在：{args.dvl_port}，跳过 DVL 启动。")
    if not start_volt:
        print(f"[WARN] Volt 端口不存在：{args.volt_port}，跳过 Volt 启动。")

    # 各窗口内启动命令
    if start_imu:
        run_tmux(["tmux", "send-keys", "-t", pane_imu, cmd_imu], "在 imu 窗口发送命令失败")
        send_enter(pane_imu)
        print(f"[imu]  启动：{cmd_imu}")

    if start_dvl:
        run_tmux(["tmux", "send-keys", "-t", pane_dvl, cmd_dvl], "在 dvl 窗口发送命令失败")
        send_enter(pane_dvl)
        print(f"[dvl]  启动：{cmd_dvl}")

    if start_volt:
        run_tmux(["tmux", "send-keys", "-t", pane_volt, cmd_volt], "在 volt 窗口发送命令失败")
        send_enter(pane_volt)
        print(f"[volt] 启动：{cmd_volt}")

    if not (start_imu or start_dvl or start_volt):
        print("[ERR] 无任何传感器成功启动，关闭会话。")
        graceful_stop_all(args.session)
        sys.exit(1)

    print("\n=== 已启动 ===")
    print(f"- 查看会话： tmux attach -t {args.session}")
    print("- 在此终端输入 's' 可停止所有采集并关闭会话；或 Ctrl+C 退出管理脚本。\n")

    # 本终端监听 's' 触发停止
    def input_listener():
        try:
            while True:
                user_input = input("输入 's' 停止所有采集并关闭 tmux 会话: ").strip().lower()
                if user_input == "s":
                    print("收到退出指令，开始优雅停止…")
                    graceful_stop_all(args.session, try_soft_exit_key="s")
                    break
                else:
                    print("无效输入，请输入 's'。")
        except (KeyboardInterrupt, EOFError):
            print("\n收到中断，尝试优雅停止会话…")
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
