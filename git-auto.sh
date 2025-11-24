#!/bin/bash
# git-auto.sh - 自动完成分支创建、同步、提交与推送

# 默认分支名称
BRANCH="feature/IMU-DVL-serial"
REMOTE="origin"
PULL_MODE="rebase"
CREATE_IF_MISSING=1
PUSH=1
COMMIT_MSG=""
ADD_ALL=1
NO_VERIFY=0

usage() {
    echo "Usage: $0 [-b branch] [-m commit_message] [-p pull_mode] [-n]"
    echo "-b branch         : 分支名称 (default: feature/IMU-DVL-serial)"
    echo "-m commit_message : 提交信息"
    echo "-p pull_mode      : 拉取模式（rebase、merge、none），默认 rebase"
    echo "-n                : 跳过推送"
    exit 1
}

# 解析参数
while getopts "b:m:p:n" OPTION; do
    case $OPTION in
        b) BRANCH=$OPTARG ;;
        m) COMMIT_MSG=$OPTARG ;;
        p) PULL_MODE=$OPTARG ;;
        n) PUSH=0 ;;
        *) usage ;;
    esac
done

# 确保当前目录为 Git 仓库
if ! git rev-parse --is-inside-work-tree > /dev/null 2>&1; then
    echo "[ERROR] 当前目录不是 Git 仓库"
    exit 1
fi

# 获取远程信息
git remote -v

# 确认是否存在目标分支
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    echo "[INFO] 切换到分支 $BRANCH"
    git checkout "$BRANCH"
else
    if git ls-remote --exit-code --heads "$REMOTE" "$BRANCH" > /dev/null 2>&1; then
        echo "[INFO] 远程已存在分支 $BRANCH，检出远程分支"
        git checkout -b "$BRANCH" "$REMOTE/$BRANCH"
    else
        if [ $CREATE_IF_MISSING -eq 1 ]; then
            echo "[INFO] 分支 $BRANCH 不存在，创建新分支"
            git checkout -b "$BRANCH"
        else
            echo "[ERROR] 远程无该分支，且禁止创建"
            exit 1
        fi
    fi
fi

# 拉取远程最新代码
if [ "$PULL_MODE" != "none" ]; then
    echo "[INFO] 拉取远程代码: git pull $REMOTE $BRANCH --$PULL_MODE"
    git pull "$REMOTE" "$BRANCH" --$PULL_MODE
fi

# 添加所有变更
if [ $ADD_ALL -eq 1 ]; then
    echo "[INFO] 添加所有文件：git add ."
    git add .
fi

# 提交改动
if [ -z "$COMMIT_MSG" ]; then
    COMMIT_MSG="Auto commit: $(date +'%Y-%m-%d %H:%M:%S')"
fi
echo "[INFO] 提交更改：git commit -m '$COMMIT_MSG'"
git commit -m "$COMMIT_MSG"

# 推送到远程仓库
if [ $PUSH -eq 1 ]; then
    echo "[INFO] 推送到远程分支：git push -u $REMOTE $BRANCH"
    git push -u "$REMOTE" "$BRANCH"
else
    echo "[INFO] 跳过推送（--no-push）"
fi

echo "[INFO] 操作完成"
