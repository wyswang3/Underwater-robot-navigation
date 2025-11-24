#!/usr/bin/env python3
# git_auto.py - 全自动 Git 提交 + 推送脚本
#
# 用法：
#   python git_auto.py -m "更新内容"
#
# 功能：
#   - 自动检测未暂存更改
#   - 自动 git add .
#   - 自动 git commit
#   - 自动 pull（rebase 或 merge）
#   - 自动 push
#
# 目标：让你只需一条命令即可上传代码

import argparse
import subprocess
import sys
from pathlib import Path
from datetime import datetime


def run(args, cwd, check=True):
    print(f"[CMD] git {' '.join(args)}")
    result = subprocess.run(["git"] + args, cwd=cwd)
    if check and result.returncode != 0:
        print(f"[ERROR] 命令失败：git {' '.join(args)}")
        sys.exit(result.returncode)
    return result.returncode


def output(args, cwd):
    result = subprocess.run(["git"] + args, cwd=cwd, text=True, capture_output=True)
    if result.returncode != 0:
        print(f"[ERROR] 命令失败：git {' '.join(args)}")
        print(result.stderr)
        sys.exit(result.returncode)
    return result.stdout.strip()


def main():
    parser = argparse.ArgumentParser(description="全自动 Git 提交与推送脚本")
    parser.add_argument("-m", "--message", default="", help="提交说明（必填）")
    parser.add_argument("-b", "--branch", default="feature/IMU-DVL-serial",
                        help="分支名称（默认：feature/IMU-DVL-serial）")
    parser.add_argument("-p", "--pull-mode", default="none",
                        choices=["rebase", "merge", "none"], help="pull 模式")
    args = parser.parse_args()

    # 0) message 必填
    if args.message.strip() == "":
        print("[ERROR] 提交说明不能为空，请使用 -m <message>")
        sys.exit(1)

    # 1) 仓库根目录
    inside = output(["rev-parse", "--is-inside-work-tree"], Path("."))
    if inside != "true":
        print("[ERROR] 当前目录不是 Git 仓库。")
        sys.exit(1)

    root = Path(output(["rev-parse", "--show-toplevel"], Path(".")))
    print(f"[INFO] 仓库根目录: {root}")

    # 2) 确保在目标分支
    run(["checkout", args.branch], root, check=False)

    # 3) 检查未提交改动
    changes = output(["status", "--porcelain"], root)
    if changes:
        print("[INFO] 检测到未提交改动，自动执行 add+commit")
        run(["add", "."], root)
        run(["commit", "-m", args.message], root)
    else:
        print("[INFO] 工作区干净，无需 commit")

    # 4) pull（如果设置）
    if args.pull_mode != "none":
        print(f"[INFO] 执行 git pull --{args.pull_mode}")
        run(["pull", "origin", args.branch, f"--{args.pull_mode}"], root)

    # 5) push
    print("[INFO] 推送到远程仓库...")
    run(["push", "-u", "origin", args.branch], root)

    print("[INFO] 上传完成！")


if __name__ == "__main__":
    main()
