# manager.py
# coding: UTF-8

import subprocess
import sys
import threading
import time
#在另一个终端运行 tmux attach-session -t data_session 来显示窗格

def run_tmux_command(cmd_list, err_msg):
    """
    封装 tmux 命令调用，如果命令执行失败则输出错误信息并退出
    """
    try:
        subprocess.run(cmd_list, check=True)
    except subprocess.CalledProcessError as e:
        print(f"{err_msg}: {e}")
        sys.exit(1)

def input_listener(session_name):
    """
    在管理脚本的终端中监听用户输入，当输入 's' 时，终止整个 tmux 会话
    """
    while True:
        user_input = input("请输入 's' 停止所有采集进程并关闭 tmux 会话: ").strip().lower()
        if user_input == 's':
            print("收到退出指令，正在终止 tmux 会话...")
            run_tmux_command(
                ["tmux", "kill-session", "-t", session_name],
                "终止 tmux 会话失败"
            )
            break
        else:
            print("无效输入，请输入 's' 退出。")

def main():
    session_name = "data_session"

    # 创建 tmux 会话（后台模式）
    run_tmux_command(["tmux", "new-session", "-d", "-s", session_name],
                     "创建 tmux 会话失败")
    print(f"tmux 会话 '{session_name}' 创建成功。")

    # 在窗格 0.0 中启动 sensor1_realtime.py（IMU实时采集与滤波保存）
    run_tmux_command(
        ["tmux", "send-keys", "-t", f"{session_name}:0.0", f"{sys.executable} sensor1_realtime.py", "C-m"],
        "在窗格 0 启动 sensor1_realtime.py 失败"
    )
    print("sensor1_realtime.py 已在窗格 0 启动。")

    # 分割窗格，水平分割创建窗格 0.1，并启动 sensor2.py（STM32采集）
    run_tmux_command(["tmux", "split-window", "-h", "-t", f"{session_name}:0"],
                     "水平分割窗格失败")
    run_tmux_command(
        ["tmux", "send-keys", "-t", f"{session_name}:0.1", f"{sys.executable} sensor2.py", "C-m"],
        "在窗格 1 启动 sensor2.py 失败"
    )
    print("sensor2.py 已在窗格 1 启动。")

    # 调整窗格布局，使两个窗格均匀排列
    run_tmux_command(["tmux", "select-layout", "-t", session_name, "even-horizontal"],
                     "调整窗格布局失败")
    print("窗格布局已调整为两窗格模式。")

    # 启动一个线程在管理终端监听 's' 指令
    listener_thread = threading.Thread(target=input_listener, args=(session_name,), daemon=True)
    listener_thread.start()

    print("管理脚本正在运行。请在此终端输入 's' 停止采集并关闭 tmux 会话。")
    while listener_thread.is_alive():
        time.sleep(1)
    print("管理脚本退出。")

if __name__ == "__main__":
    main()
