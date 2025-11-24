#!/usr/bin/env bash
# git-auto.sh - 统一的 Git 自动提交流程脚本（跨平台版本）
#
# 特性：
#   - 支持 Linux / macOS / WSL / Windows(Git Bash)
#   - 自动切换/创建分支
#   - 自动 git pull（rebase/merge/none）
#   - 自动 git add .
#   - 自动 commit（若无改动则跳过提交）
#   - 自动 push（可用 -n 跳过）
#   - 支持从子目录调用（会自动跳到仓库根目录）
#
# 使用示例：
#   ./git-auto.sh
#   ./git-auto.sh -b feature/IMU-DVL-serial -m "更新 README"
#   ./git-auto.sh -p merge
#   ./git-auto.sh -n                   # 仅提交，不推送
#
# 参数说明：
#   -b <branch>  : 目标分支（默认：feature/IMU-DVL-serial）
#   -m <message> : 提交说明（默认自动生成时间戳）
#   -p <mode>    : 拉取模式：rebase / merge / none（默认：rebase）
#   -r <remote>  : 远程名称（默认：origin）
#   -n           : 不推送到远程（只 commit）
#
# 注意：
#   - 建议在仓库内任何子目录下直接运行即可
#   - Windows 请使用 Git Bash 或 `bash git-auto.sh` 运行

set -euo pipefail

# ========= 默认参数 =========
BRANCH="feature/IMU-DVL-serial"
REMOTE="origin"
PULL_MODE="rebase"   # rebase / merge / none
PUSH=1
COMMIT_MSG=""
ADD_ALL=1

usage() {
    cat <<EOF
Usage: $0 [-b branch] [-m message] [-p pull_mode] [-r remote] [-n]

Options:
  -b branch     目标分支名称（默认：feature/IMU-DVL-serial）
  -m message    提交信息（默认：Auto commit + 时间戳）
  -p pull_mode  拉取模式：rebase | merge | none（默认：rebase）
  -r remote     远程名称（默认：origin）
  -n            只提交到本地，不执行 git push

Examples:
  $0
  $0 -b feature/IMU-DVL-serial -m "更新 README"
  $0 -p merge
  $0 -n
EOF
    exit 1
}

# ========= 解析参数 =========
while getopts "b:m:p:r:n" opt; do
    case "$opt" in
        b) BRANCH=$OPTARG ;;
        m) COMMIT_MSG=$OPTARG ;;
        p) PULL_MODE=$OPTARG ;;
        r) REMOTE=$OPTARG ;;
        n) PUSH=0 ;;
        *) usage ;;
    esac
done

echo "[INFO] 目标分支 : $BRANCH"
echo "[INFO] 远程     : $REMOTE"
echo "[INFO] 拉取模式 : $PULL_MODE"
echo "[INFO] 是否推送 : $PUSH"

# ========= 确保在 Git 仓库内 =========
if ! git rev-parse --is-inside-work-tree >/dev/null 2>&1; then
    echo "[ERROR] 当前目录不是 Git 仓库，请切换到仓库目录后再运行。"
    exit 1
fi

# 自动跳到仓库根目录（跨平台通用）
REPO_ROOT=$(git rev-parse --show-toplevel)
echo "[INFO] 仓库根目录: $REPO_ROOT"
cd "$REPO_ROOT"

# ========= 显示远程信息（调试用） =========
git remote -v || {
    echo "[ERROR] 无法获取远程信息，检查 Git 配置。"
    exit 1
}

# ========= 切换 / 创建 分支 =========
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    echo "[INFO] 本地存在分支 $BRANCH，尝试切换过去..."
    git checkout "$BRANCH"
else
    # 本地没有该分支，检查远程是否有
    if git ls-remote --exit-code --heads "$REMOTE" "$BRANCH" >/dev/null 2>&1; then
        echo "[INFO] 远程存在分支 $BRANCH，本地创建跟踪分支..."
        git checkout -b "$BRANCH" "$REMOTE/$BRANCH"
    else
        echo "[WARN] 本地和远程都不存在分支 $BRANCH，将在本地新建该分支。"
        git checkout -b "$BRANCH"
    fi
fi

# ========= 拉取最新远程代码（可配置 rebase / merge / none） =========
if [ "$PULL_MODE" != "none" ]; then
    echo "[INFO] 拉取远程更新：git pull $REMOTE $BRANCH --$PULL_MODE"
    if ! git pull "$REMOTE" "$BRANCH" --"$PULL_MODE"; then
        echo "[ERROR] git pull 失败，请解决冲突后重试。"
        exit 1
    fi
else
    echo "[INFO] 跳过 git pull（PULL_MODE=none）"
fi

# ========= 检查工作区是否有改动 =========
CHANGES=$(git status --porcelain)

if [ -z "$CHANGES" ]; then
    echo "[INFO] 当前工作区没有任何改动（nothing to commit）。"
    if [ "$PUSH" -eq 1 ]; then
        echo "[INFO] 尝试同步远程：git push $REMOTE $BRANCH"
        if ! git push "$REMOTE" "$BRANCH"; then
            echo "[ERROR] git push 失败，请检查网络或远程权限。"
            exit 1
        fi
    fi
    echo "[INFO] 操作完成（无新的提交）。"
    exit 0
fi

echo "[INFO] 检测到以下改动："
echo "$CHANGES"

# ========= 添加所有改动（可根据需要未来扩展为按文件添加） =========
if [ "$ADD_ALL" -eq 1 ]; then
    echo "[INFO] 执行 git add ."
    git add .
fi

# ========= 提交 =========
if [ -z "$COMMIT_MSG" ]; then
    COMMIT_MSG="Auto commit: $(date +'%Y-%m-%d %H:%M:%S')"
fi

echo "[INFO] 执行 git commit -m \"$COMMIT_MSG\""
if ! git commit -m "$COMMIT_MSG"; then
    echo "[ERROR] git commit 失败（可能是 hook 拦截、冲突或无改动）。"
    exit 1
fi

# ========= 推送到远程 =========
if [ "$PUSH" -eq 1 ]; then
    echo "[INFO] 推送到远程：git push -u $REMOTE $BRANCH"
    if ! git push -u "$REMOTE" "$BRANCH"; then
        echo "[ERROR] git push 失败，请检查远程权限或网络。"
        exit 1
    fi
else
    echo "[INFO] 已根据参数选择跳过推送（-n）。"
fi

echo "[INFO] 全部操作完成。"
