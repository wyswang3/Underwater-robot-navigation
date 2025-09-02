# tmux_telemetry_manager.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件名: tmux_telemetry_manager.py

功能:
- 在一个 tmux 会话中并排启动两路采集：
  1) imu_realtime_pipeline.py  (IMU 实时采集/滤波/落盘)
  2) volt32_logger.py          (16通道电压采集/轮换落盘)
- 在当前终端监听输入: 输入 's' → 先向每个 pane 发送 Ctrl+C 优雅退出，再 kill 会话
- 可重复运行；若会话已存在会先杀掉旧会话，确保 pane 干净

用法:
  python tmux_telemetry_manager.py
  # 另一个终端里:
  tmux attach -t data_session   # 查看运行中的窗格
"""

import os
import sys
import shutil
import subprocess
import threading
import time

SESSION = "data_session"
SCRIPTS = [
    # (启动命令, pane 目标)
    (f"{sys.executable} -u imu_realtime_pipeline.py", "0.0"),  # 左 pane
    (f"{sys.executable} -u volt32_logger.py",        "0.1"),  # 右 pane
]

# ========== 基础封装 ==========
def run_tmux(cmd_list, err_msg, check=True, capture=False):
    """执行 tmux 命令; 失败时报错退出。"""
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

def tmux_exists():
    """检查 tmux 是否可用。"""
    return shutil.which("tmux") is not None

def session_exists(name: str) -> bool:
    """判断会话是否存在。"""
    res = subprocess.run(["tmux", "has-session", "-t", name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return res.returncode == 0

def send_keys(target: str, keys: str):
    """向 pane 发送键入（不自动回车）。"""
    run_tmux(["tmux", "send-keys", "-t", target, keys], "send-keys 失败", check=True)

def send_enter(target: str):
    run_tmux(["tmux", "send-keys", "-t", target, "C-m"], "发送回车失败", check=True)

def graceful_stop_all(session: str):
    """向每个 pane 发送 Ctrl+C，等待片刻，再 kill 会话。"""
    # 列出 pane id
    res = run_tmux(["tmux", "list-panes", "-t", session, "-F", "#{pane_id}"], "列出 panes 失败", capture=True)
    panes = [line.strip() for line in res.stdout.splitlines() if line.strip()]
    if not panes:
        # 没 pane 也直接尝试杀会话
        run_tmux(["tmux", "kill-session", "-t", session], "终止 tmux 会话失败")
        return

    print("向各 pane 发送 Ctrl+C …")
    for pid in panes:
        run_tmux(["tmux", "send-keys", "-t", pid, "C-c"], f"向 {pid} 发送 Ctrl+C 失败", check=False)

    # 可选：再发一次 's' 回车（你的采集程序也支持 's' 退出）
    for pid in panes:
        send_keys(pid, "s")
        send_enter(pid)

    # 等待脚本优雅退出
    time.sleep(1.5)

    # 兜底杀会话
    run_tmux(["tmux", "kill-session", "-t", session], "终止 tmux 会话失败")
    print("tmux 会话已终止。")

# ========== 主流程 ==========
def input_listener():
    """在本终端监听 's'，触发统一收尾。"""
    while True:
        user_input = input("输入 's' 停止所有采集并关闭 tmux 会话: ").strip().lower()
        if user_input == 's':
            print("收到退出指令，开始优雅停止…")
            graceful_stop_all(SESSION)
            break
        else:
            print("无效输入，请输入 's' 退出。")

def main():
    if not tmux_exists():
        print("未找到 tmux，请先安装（Linux/macOS 可用）。")
        sys.exit(1)

    # 若已有旧会话，先清理，确保重复运行不叠加 pane
    if session_exists(SESSION):
        print(f"检测到已存在会话 '{SESSION}'，先清理…")
        run_tmux(["tmux", "kill-session", "-t", SESSION], "清理旧会话失败")

    # 创建新会话（后台），先起一个空 shell
    run_tmux(["tmux", "new-session", "-d", "-s", SESSION], "创建 tmux 会话失败")
    print(f"tmux 会话 '{SESSION}' 创建成功。")

    # 在窗口 0 水平分割成两个 pane
    run_tmux(["tmux", "split-window", "-h", "-t", f"{SESSION}:0"], "水平分割窗格失败")
    run_tmux(["tmux", "select-layout", "-t", SESSION, "even-horizontal"], "调整窗格布局失败")
    # 可选：退出后保留输出
    run_tmux(["tmux", "set-option", "-t", SESSION, "remain-on-exit", "on"], "设置 remain-on-exit 失败", check=False)

    # 在每个 pane 里 cd 到脚本目录并启动脚本（-u 关闭缓冲）
    scripts_dir = os.path.dirname(os.path.abspath(__file__))
    for cmd, target in SCRIPTS:
        full_cmd = f"cd {scripts_dir} && {cmd}"
        run_tmux(["tmux", "send-keys", "-t", f"{SESSION}:{target}", full_cmd], f"在窗格 {target} 发送命令失败")
        send_enter(f"{SESSION}:{target}")
        print(f"已在窗格 {target} 启动：{cmd}")

    # 打印使用提示
    print("\n=== 已启动 ===")
    print(f"- 在另一个终端查看: tmux attach -t {SESSION}")
    print("- 在此终端输入 's' 可优雅停止并关闭会话。\n")

    # 监听本地 's'
    t = threading.Thread(target=input_listener, daemon=True)
    t.start()
    while t.is_alive():
        time.sleep(1)
    print("管理脚本退出。")

if __name__ == "__main__":
    main()