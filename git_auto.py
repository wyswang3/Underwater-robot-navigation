#!/usr/bin/env python3
# git_auto.py - 跨平台 Git 自动提交 + 推送脚本
#
# 用法示例：
#   python git_auto.py
#   python git_auto.py -m "更新 README 与测试文档"
#   python git_auto.py -b feature/IMU-DVL-serial -m "修复导航 C++ 工程"
#   python git_auto.py -p merge
#   python git_auto.py --no-push

import argparse
import subprocess
import sys
from pathlib import Path
from datetime import datetime


def run_git(args, cwd, check=True):
    """运行 git 命令并打印"""
    cmd = ["git"] + args
    print(f"[CMD] {' '.join(cmd)}")
    result = subprocess.run(cmd, cwd=cwd, text=True)
    if check and result.returncode != 0:
        print(f"[ERROR] git 命令执行失败：{' '.join(cmd)}", file=sys.stderr)
        sys.exit(result.returncode)
    return result.returncode


def git_output(args, cwd):
    """运行 git 命令并返回 stdout"""
    cmd = ["git"] + args
    result = subprocess.run(cmd, cwd=cwd, text=True, capture_output=True)
    if result.returncode != 0:
        print(f"[ERROR] git 命令执行失败：{' '.join(cmd)}", file=sys.stderr)
        print(result.stderr, file=sys.stderr)
        sys.exit(result.returncode)
    return result.stdout.strip()


def main():
    parser = argparse.ArgumentParser(
        description="跨平台 Git 自动提交 / 推送脚本（适用于 VS Code 终端）"
    )
    parser.add_argument(
        "-b", "--branch",
        default="feature/IMU-DVL-serial",
        help="目标分支名称（默认：feature/IMU-DVL-serial）"
    )
    parser.add_argument(
        "-m", "--message",
        default="",
        help="提交信息（默认：Auto commit + 时间戳）"
    )
    parser.add_argument(
        "-p", "--pull-mode",
        choices=["rebase", "merge", "none"],
        default="rebase",
        help="拉取模式：rebase / merge / none（默认：rebase）"
    )
    parser.add_argument(
        "-r", "--remote",
        default="origin",
        help="远程名称（默认：origin）"
    )
    parser.add_argument(
        "--no-push",
        action="store_true",
        help="只提交到本地，不执行 git push"
    )

    args = parser.parse_args()

    # 1) 确保在 Git 仓库内
    try:
        inside = git_output(["rev-parse", "--is-inside-work-tree"], cwd=Path("."))
    except SystemExit:
        print("[ERROR] 当前目录不是 Git 仓库，请先 cd 到仓库根目录。", file=sys.stderr)
        sys.exit(1)

    if inside.strip() != "true":
        print("[ERROR] 当前目录不是 Git 仓库。", file=sys.stderr)
        sys.exit(1)

    # 2) 仓库根目录
    repo_root = git_output(["rev-parse", "--show-toplevel"], cwd=Path("."))
    repo_root = Path(repo_root)
    print(f"[INFO] 仓库根目录: {repo_root}")
    print(f"[INFO] 目标分支 : {args.branch}")
    print(f"[INFO] 远程     : {args.remote}")
    print(f"[INFO] 拉取模式 : {args.pull_mode}")
    print(f"[INFO] 是否推送 : {not args.no_push}")

    # 3) 显示 remote 信息
    print("[INFO] 远程列表：")
    run_git(["remote", "-v"], cwd=repo_root, check=False)

    # 4) 切换 / 创建 分支
    # 4.1 检查本地是否已有该分支
    ret = run_git(["show-ref", "--verify", f"refs/heads/{args.branch}"],
                  cwd=repo_root, check=False)
    if ret == 0:
        print(f"[INFO] 本地存在分支 {args.branch}，切换过去...")
        run_git(["checkout", args.branch], cwd=repo_root)
    else:
        # 4.2 检查远程是否存在
        ret_remote = run_git(
            ["ls-remote", "--exit-code", "--heads", args.remote, args.branch],
            cwd=repo_root,
            check=False
        )
        if ret_remote == 0:
            print(f"[INFO] 远程存在分支 {args.branch}，本地创建跟踪分支...")
            run_git(["checkout", "-b", args.branch, f"{args.remote}/{args.branch}"],
                    cwd=repo_root)
        else:
            print(f"[WARN] 本地和远程都不存在分支 {args.branch}，本地新建该分支...")
            run_git(["checkout", "-b", args.branch], cwd=repo_root)

    # 5) 拉取最新代码
    if args.pull_mode != "none":
        print(f"[INFO] 拉取远程更新：git pull {args.remote} {args.branch} --{args.pull_mode}")
        run_git(["pull", args.remote, args.branch, f"--{args.pull_mode}"], cwd=repo_root)
    else:
        print("[INFO] 跳过 git pull（pull-mode = none）")

    # 6) 检查改动
    status_output = git_output(["status", "--porcelain"], cwd=repo_root)
    if not status_output.strip():
        print("[INFO] 当前工作区没有任何改动（nothing to commit）。")
        if not args.no_push:
            print(f"[INFO] 仍尝试同步远程：git push {args.remote} {args.branch}")
            run_git(["push", args.remote, args.branch], cwd=repo_root, check=False)
        print("[INFO] 操作完成（无新的提交）。")
        return

    print("[INFO] 检测到以下改动：")
    print(status_output)

    # 7) git add .
    print("[INFO] 执行 git add .")
    run_git(["add", "."], cwd=repo_root)

    # 8) git commit
    if not args.message:
        args.message = "Auto commit: " + datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    print(f'[INFO] 执行 git commit -m "{args.message}"')
    run_git(["commit", "-m", args.message], cwd=repo_root)

    # 9) git push
    if not args.no_push:
        print(f"[INFO] 推送到远程：git push -u {args.remote} {args.branch}")
        run_git(["push", "-u", args.remote, args.branch], cwd=repo_root)
    else:
        print("[INFO] 已根据参数选择跳过推送（--no-push）")

    print("[INFO] 全部操作完成。")


if __name__ == "__main__":
    main()
