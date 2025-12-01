#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
project_size_report.py

用途：
  - 统计一个 C/C++/Python 项目的代码规模和“复杂度指标”
  - 输出一份可读的摘要报告，方便做工程级代码体检

核心指标：
  - 总行数 / 有效代码行数（非空行）
  - 每种语言的文件数 / 行数
  - 按目录聚合的行数（一级目录）
  - Top N 最大的源文件（行数 + 简单“分支复杂度”计数）
  - 规模评级：小型 / 中等 / 大型

使用示例：
  python project_size_report.py .
  python project_size_report.py ../UnderwaterRobotSystem --top 15
"""

import argparse
import os
from dataclasses import dataclass, field
from typing import Dict, List, Tuple


# 默认统计的扩展名：可以按需要增减
DEFAULT_EXTS = [
    ".c", ".cpp", ".cc", ".cxx",
    ".h", ".hpp", ".hh",
    ".py",
]


BRANCH_TOKENS = [
    "if",
    "for",
    "while",
    "switch",
    "case",
    "?:",   # 三目运算符（只是提示）
]


@dataclass
class FileStat:
    rel_path: str
    ext: str
    total_lines: int = 0       # 文件总行数
    code_lines: int = 0        # 非空行
    branch_tokens: int = 0     # 简单“分支复杂度”计数


@dataclass
class ProjectStat:
    root: str
    files: List[FileStat] = field(default_factory=list)

    def add_file(self, stat: FileStat):
        self.files.append(stat)

    @property
    def total_files(self) -> int:
        return len(self.files)

    @property
    def total_lines(self) -> int:
        return sum(f.total_lines for f in self.files)

    @property
    def total_code_lines(self) -> int:
        return sum(f.code_lines for f in self.files)

    def group_by_ext(self) -> Dict[str, Tuple[int, int]]:
        """
        返回 {ext: (file_count, code_lines)}
        """
        result: Dict[str, Tuple[int, int]] = {}
        for f in self.files:
            cnt, loc = result.get(f.ext, (0, 0))
            result[f.ext] = (cnt + 1, loc + f.code_lines)
        return result

    def group_by_top_dir(self) -> Dict[str, int]:
        """
        以仓库根目录下的 一级目录 为单位汇总 code_lines：
        - pwm_control_program/src/... -> pwm_control_program
        - nav_core/src/...            -> nav_core
        - 根目录下的文件              -> "(root)"
        """
        result: Dict[str, int] = {}
        for f in self.files:
            parts = f.rel_path.split(os.sep)
            if len(parts) == 1:
                key = "(root)"
            else:
                key = parts[0]
            result[key] = result.get(key, 0) + f.code_lines
        return result

    def top_n_files(self, n: int = 10) -> List[FileStat]:
        return sorted(self.files,
                      key=lambda f: f.code_lines,
                      reverse=True)[:n]


def is_source_file(path: str, exts: List[str]) -> bool:
    _, ext = os.path.splitext(path)
    return ext.lower() in exts


def analyze_file(root: str, path: str) -> FileStat:
    rel_path = os.path.relpath(path, root)
    _, ext = os.path.splitext(path)
    stat = FileStat(rel_path=rel_path, ext=ext.lower())

    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            for line in f:
                stat.total_lines += 1
                stripped = line.strip()
                if stripped:
                    stat.code_lines += 1

                    # 简单“分支复杂度”：出现一次 if/for/while/switch/case 记 1
                    # 这里只是一个粗略的 branchiness 指标，不是严格的圈复杂度
                    lowered = stripped.lower()
                    for tok in BRANCH_TOKENS:
                        # 用空格/括号/括号组合避免误伤（例如 "diff"）
                        # 简化处理：只做 in 检查，足够用于相对比较
                        if tok in lowered:
                            stat.branch_tokens += 1
    except Exception as e:
        print(f"[WARN] 读取文件失败: {rel_path} ({e})")

    return stat


def analyze_project(root: str, exts: List[str]) -> ProjectStat:
    proj = ProjectStat(root=root)

    for dirpath, dirnames, filenames in os.walk(root):
        # 跳过一些典型目录（按需修改）
        base = os.path.basename(dirpath)
        if base in {".git", ".idea", ".vscode", "__pycache__", "build"}:
            continue

        for fname in filenames:
            full_path = os.path.join(dirpath, fname)
            if not is_source_file(full_path, exts):
                continue
            fs = analyze_file(root, full_path)
            proj.add_file(fs)
    return proj


def rate_project_size(total_loc: int) -> str:
    """
    根据总代码行数给出一个简单的规模评级。
    你可以按自己的项目习惯调整阈值。
    """
    if total_loc < 5000:
        return "小型（< 5k LOC）：整体代码量较小，适合快速迭代。"
    elif total_loc < 20000:
        return "中等（5k–20k LOC）：典型工程项目规模，注意模块划分和文档。"
    elif total_loc < 50000:
        return "大型（20k–50k LOC）：系统性工程，需要严格的模块边界和测试。"
    else:
        return "超大型（> 50k LOC）：建议引入更严格的架构治理和代码审查流程。"


def rate_avg_file_size(avg_loc: float) -> str:
    if avg_loc < 150:
        return "单文件平均行数较小，模块拆分比较细。"
    elif avg_loc < 400:
        return "单文件平均行数适中，可读性通常良好。"
    else:
        return "单文件平均行数偏大，建议关注大文件（>800 行）是否可以拆分。"


def main():
    parser = argparse.ArgumentParser(
        description="分析项目代码规模，生成简要评审报告。"
    )
    parser.add_argument(
        "root",
        nargs="?",
        default=".",
        help="项目根目录（默认：当前目录）"
    )
    parser.add_argument(
        "--ext",
        nargs="*",
        default=DEFAULT_EXTS,
        help="需要统计的文件扩展名（默认：常见 C/C++/Python 扩展）"
    )
    parser.add_argument(
        "--top",
        type=int,
        default=10,
        help="输出 Top N 最大文件（按代码行数，默认 10）"
    )

    args = parser.parse_args()
    root = os.path.abspath(args.root)
    exts = [e.lower() if e.startswith(".") else "." + e.lower() for e in args.ext]

    print(f"[INFO] 项目根目录: {root}")
    print(f"[INFO] 扩展名过滤: {', '.join(exts)}\n")

    proj = analyze_project(root, exts)

    if proj.total_files == 0:
        print("[WARN] 未找到匹配的源文件，请检查 root 路径或扩展名过滤参数。")
        return

    total_loc = proj.total_code_lines
    total_files = proj.total_files
    avg_file_loc = total_loc / total_files if total_files > 0 else 0.0

    # ========== 1. 总体指标 ==========
    print("========== 1. 总体规模指标 ==========")
    print(f"  源文件总数      : {total_files}")
    print(f"  有效代码行数 LOC: {total_loc}")
    print(f"  单文件平均 LOC  : {avg_file_loc:.1f}")
    print(f"  规模评级        : {rate_project_size(total_loc)}")
    print(f"  平均文件大小评语: {rate_avg_file_size(avg_file_loc)}")
    print()

    # ========== 2. 按语言/扩展名统计 ==========
    print("========== 2. 按扩展名统计 ==========")
    by_ext = proj.group_by_ext()
    for ext, (cnt, loc) in sorted(by_ext.items(), key=lambda kv: kv[1][1], reverse=True):
        print(f"  {ext:6s}  文件数: {cnt:4d}   LOC: {loc:6d}")
    print()

    # ========== 3. 按一级目录聚合 ==========
    print("========== 3. 按一级目录聚合 ==========")
    by_dir = proj.group_by_top_dir()
    for d, loc in sorted(by_dir.items(), key=lambda kv: kv[1], reverse=True):
        print(f"  {d:20s}  LOC: {loc:6d}")
    print()

    # ========== 4. Top N 最大文件 ==========
    print(f"========== 4. Top {args.top} 最大源文件（按 LOC） ==========")
    top_files = proj.top_n_files(args.top)
    for idx, f in enumerate(top_files, start=1):
        print(f"  [{idx:2d}] {f.rel_path}")
        print(f"       LOC(有效行): {f.code_lines}")
        print(f"       总行数     : {f.total_lines}")
        print(f"       分支 token : {f.branch_tokens}")
    print()

    # ========== 5. 简短评审建议 ==========
    print("========== 5. 评审建议（可以据此做人工 Review） ==========")
    print("  - 优先人工检查 Top N 最大文件：")
    print("      * 是否职责单一？是否可以拆分为多个模块？")
    print("      * 分支 token 很多的文件，是否逻辑过于集中？")
    print("  - 查看按目录聚合的 LOC：")
    print("      * 某个子系统（如 pwm_control_program 或 nav_core）是否代码量异常庞大？")
    print("      * 是否需要拆出子模块或单独仓库？")
    print("  - 对于 > 800 LOC 的单文件，建议：")
    print("      * 考虑按功能拆分（例如：协议解析 / 控制逻辑 / 日志）")
    print("      * 或者提炼公共工具函数，降低主流程函数长度。")
    print()
    print("[INFO] 分析完成。你可以将本输出重定向保存，如：")
    print("       python project_size_report.py . > project_size_report.txt")


if __name__ == "__main__":
    main()
