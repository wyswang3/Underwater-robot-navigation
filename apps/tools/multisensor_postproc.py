#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
multisensor_postproc.py

离线三传感器数据后处理与可视化工具：
- 读取给定日期的 IMU / DVL / Volt 数据
- 按统一时间轴对齐（以 DVL EstS 为参考）
- 基础可视化
- 可选导出对齐后的融合 CSV

依赖：
  pip install pandas matplotlib
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
from typing import Optional, Tuple

import pandas as pd
import matplotlib.pyplot as plt


# ===================== 路径与文件发现 =====================

def default_data_root() -> Path:
    """
    默认 data 根目录：
      仓库根/ data
    本文件位于: <repo_root>/apps/tools/multisensor_postproc.py
    """
    here = Path(__file__).resolve()
    repo_root = here.parents[2]  # .../Underwater-robot-navigation
    return repo_root / "data"


def resolve_day_dirs(data_root: Path, date_str: str) -> Tuple[Path, Path, Path]:
    """
    返回某一天的 imu/dvl/volt 子目录（不存在也返回路径，由调用方判断）。
    """
    day_root = data_root / date_str
    imu_dir = day_root / "imu"
    dvl_dir = day_root / "dvl"
    volt_dir = day_root / "volt"
    return imu_dir, dvl_dir, volt_dir


def find_latest_file(directory: Path, pattern: str) -> Optional[Path]:
    """
    在目录下按通配符 pattern 找到“最新”的一个文件（按文件名排序）。
    若无匹配，返回 None。
    """
    candidates = sorted(directory.glob(pattern))
    if not candidates:
        return None
    return candidates[-1]


# ===================== 数据加载函数 =====================

def load_imu_filtered(path: Path) -> pd.DataFrame:
    """
    读取 IMU 滤波后的 CSV：
      列: Timestamp, AccX_filt,AccY_filt,AccZ_filt, AsX_filt,AsY_filt,AsZ_filt, Yaw_filt(deg)
    增加一列:
      t_s_imu : 浮点秒，用于时间对齐
    """
    df = pd.read_csv(path)
    if "Timestamp" not in df.columns:
        raise ValueError(f"IMU 文件不含 'Timestamp' 列: {path}")
    # 解析为 datetime
    df["Timestamp"] = pd.to_datetime(df["Timestamp"])
    # 转为 unix 秒（注意 pandas datetime64[ns]）
    df["t_s_imu"] = df["Timestamp"].astype("int64") / 1e9
    return df


def load_dvl_speed_tb(path: Path) -> pd.DataFrame:
    """
    读取 DVL 最小速度表（MinimalSpeedWriterTB 输出）：
      列: MonoNS, EstNS, MonoS, EstS, SensorID, Src, East_Vel(m_s), North_Vel(m_s), Up_Vel(m_s)
    增加:
      t_s_dvl = EstS（直接复制）
    """
    df = pd.read_csv(path)
    needed = ["EstS", "East_Vel(m_s)", "North_Vel(m_s)", "Up_Vel(m_s)"]
    for col in needed:
        if col not in df.columns:
            raise ValueError(f"DVL 文件缺少列 '{col}': {path}")
    df = df.copy()
    df["t_s_dvl"] = df["EstS"].astype(float)
    return df


def load_volt(path: Path) -> pd.DataFrame:
    """
    读取 Volt32 logger 输出：
      第一列 Timestamp (字符串)，其余为 CH0, CH1, ...
    增加:
      t_s_volt : 浮点秒，用于时间对齐
    """
    df = pd.read_csv(path)
    if "Timestamp" not in df.columns:
        raise ValueError(f"Volt 文件不含 'Timestamp' 列: {path}")
    df["Timestamp"] = pd.to_datetime(df["Timestamp"])
    df["t_s_volt"] = df["Timestamp"].astype("int64") / 1e9
    return df


# ===================== 对齐与融合 =====================

def align_multisensor(
    df_dvl: pd.DataFrame,
    df_imu: Optional[pd.DataFrame],
    df_volt: Optional[pd.DataFrame],
    tolerance: float = 0.2,
) -> pd.DataFrame:
    """
    以 DVL 的 t_s_dvl 为参考时间轴，使用 merge_asof 将 IMU 和 Volt 对齐到 DVL 采样点。
    tolerance: 最大时间差（秒），超出则对应列为 NaN。
    返回：包含 dvl + imu + volt 的融合表。
    """

    # 只保留需要的列，避免列名冲突 / 冗余
    dvl_cols = ["t_s_dvl", "EstS", "East_Vel(m_s)", "North_Vel(m_s)", "Up_Vel(m_s)", "SensorID", "Src"]
    df_dvl_use = df_dvl[dvl_cols].sort_values("t_s_dvl")

    merged = df_dvl_use.rename(columns={"t_s_dvl": "t_s"})

    if df_imu is not None and not df_imu.empty:
        imu_cols = [
            "t_s_imu",
            "AccX_filt", "AccY_filt", "AccZ_filt",
            "AsX_filt", "AsY_filt", "AsZ_filt",
            "Yaw_filt(deg)",
        ]
        for col in imu_cols:
            if col not in df_imu.columns:
                raise ValueError(f"IMU DataFrame 缺少列 '{col}'")
        df_imu_use = df_imu[imu_cols].sort_values("t_s_imu")
        merged = pd.merge_asof(
            merged.sort_values("t_s"),
            df_imu_use,
            left_on="t_s",
            right_on="t_s_imu",
            direction="nearest",
            tolerance=tolerance,
        )
        # 合并后可以根据需要删除 t_s_imu
        # 这里保留，便于调试
    else:
        print("[WARN] 未对齐 IMU 数据（None 或空表）")

    if df_volt is not None and not df_volt.empty:
        # 取 t_s_volt + 所有 CH* 列
        volt_cols = ["t_s_volt"] + [c for c in df_volt.columns if c.startswith("CH")]
        df_volt_use = df_volt[volt_cols].sort_values("t_s_volt")
        merged = pd.merge_asof(
            merged.sort_values("t_s"),
            df_volt_use,
            left_on="t_s",
            right_on="t_s_volt",
            direction="nearest",
            tolerance=tolerance,
        )
    else:
        print("[WARN] 未对齐 Volt 数据（None 或空表）")

    return merged


# ===================== 可视化 =====================

def plot_basic(merged: pd.DataFrame, show: bool = True, save_prefix: Optional[Path] = None):
    """
    基础可视化：
      图1：IMU 滤波加速度 + DVL 速度
      图2：Volt 通道（示意：CH0, CH1）
    """
    if merged.empty:
        print("[WARN] 融合数据为空，跳过绘图。")
        return

    # 使用 t_s 作为统一时间轴（秒）
    t = merged["t_s"]

    # ---- 图1：IMU + DVL ----
    fig1, axes = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    ax1, ax2 = axes

    # IMU 加速度（若存在）
    if {"AccX_filt", "AccY_filt", "AccZ_filt"}.issubset(merged.columns):
        ax1.plot(t, merged["AccX_filt"], label="AccX_filt")
        ax1.plot(t, merged["AccY_filt"], label="AccY_filt")
        ax1.plot(t, merged["AccZ_filt"], label="AccZ_filt")
        ax1.set_ylabel("Acc_filt (m/s^2 or g)")
        ax1.legend()
        ax1.grid(True)
    else:
        ax1.text(0.5, 0.5, "No IMU filtered acc", transform=ax1.transAxes, ha="center")

    # DVL 速度
    ax2.plot(t, merged["East_Vel(m_s)"], label="Ve")
    ax2.plot(t, merged["North_Vel(m_s)"], label="Vn")
    ax2.plot(t, merged["Up_Vel(m_s)"], label="Vu")
    ax2.set_xlabel("Time (s, EstS)")
    ax2.set_ylabel("DVL Vel (m/s)")
    ax2.legend()
    ax2.grid(True)

    fig1.tight_layout()
    if save_prefix is not None:
        fig1.savefig(str(save_prefix) + "_imu_dvl.png", dpi=150)

    # ---- 图2：Volt 通道（示意 CH0/CH1）----
    ch_cols = [c for c in merged.columns if c.startswith("CH")]
    if ch_cols:
        fig2, axv = plt.subplots(1, 1, figsize=(10, 4))
        # 只画前两路以免太乱
        for cname in ch_cols[:2]:
            axv.plot(t, merged[cname], label=cname)
        axv.set_xlabel("Time (s, EstS)")
        axv.set_ylabel("Volt/Current (raw units)")
        axv.legend()
        axv.grid(True)
        fig2.tight_layout()
        if save_prefix is not None:
            fig2.savefig(str(save_prefix) + "_volt.png", dpi=150)
    else:
        print("[INFO] 融合表中未发现 CH* 列，跳过 Volt 绘图。")

    if show:
        plt.show()
    else:
        plt.close("all")


# ===================== CLI =====================

def make_argparser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Multi-sensor (IMU+DVL+Volt) post-processing & visualization")
    ap.add_argument("--date", default=None,
                    help="数据日期，格式 YYYY-MM-DD；默认=今天（系统时间）")
    ap.add_argument("--data-root", default=None,
                    help="data 根目录；默认=<repo_root>/data")
    ap.add_argument("--tolerance", type=float, default=0.2,
                    help="时间对齐 merge_asof 容差（秒），默认 0.2s")
    ap.add_argument("--no-plot", action="store_true",
                    help="不弹出图像窗口（仅导出融合数据）")
    ap.add_argument("--save-merged", action="store_true",
                    help="保存对齐后的融合 CSV 到 <data_root>/<date>/aligned/ 目录")
    return ap


def main():
    ap = make_argparser()
    args = ap.parse_args()

    # 解析 data_root
    if args.data_root:
        data_root = Path(args.data_root).resolve()
    else:
        data_root = default_data_root()
    if not data_root.exists():
        print(f"[ERR] data_root 不存在: {data_root}")
        return

    # 解析日期
    if args.date is None:
        # 如果没指定日期，就取今天（按本机时间）
        from datetime import datetime
        date_str = datetime.now().strftime("%Y-%m-%d")
    else:
        date_str = args.date

    imu_dir, dvl_dir, volt_dir = resolve_day_dirs(data_root, date_str)
    print(f"[INFO] 使用日期 {date_str}")
    print(f"[INFO] IMU  目录: {imu_dir}")
    print(f"[INFO] DVL  目录: {dvl_dir}")
    print(f"[INFO] Volt 目录: {volt_dir}")

    # 选取各自最新文件
    imu_file  = find_latest_file(imu_dir,  "imu_filtered_data_*.csv")
    dvl_file  = find_latest_file(dvl_dir,  "dvl_speed_min_tb_*.csv")
    volt_file = find_latest_file(volt_dir, "motor_data_*.csv")

    if imu_file is None:
        print(f"[WARN] 未找到 IMU 滤波文件 (imu_filtered_data_*.csv) 于 {imu_dir}")
    else:
        print(f"[INFO] 使用 IMU 文件:  {imu_file}")

    if dvl_file is None:
        print(f"[WARN] 未找到 DVL 最小速度文件 (dvl_speed_min_tb_*.csv) 于 {dvl_dir}")
    else:
        print(f"[INFO] 使用 DVL 文件:  {dvl_file}")

    if volt_file is None:
        print(f"[WARN] 未找到 Volt 文件 (motor_data_*.csv) 于 {volt_dir}")
    else:
        print(f"[INFO] 使用 Volt 文件: {volt_file}")

    if dvl_file is None:
        print("[ERR] 没有 DVL 参考时间轴，无法进行三传感器对齐。直接退出。")
        return

    # 加载
    df_dvl = load_dvl_speed_tb(dvl_file)
    df_imu = load_imu_filtered(imu_file) if imu_file is not None else None
    df_volt = load_volt(volt_file) if volt_file is not None else None

    # 对齐
    merged = align_multisensor(df_dvl, df_imu, df_volt, tolerance=args.tolerance)
    print(f"[INFO] 融合表行数: {len(merged)}")

    # 保存融合表
    save_prefix = None
    if args.save_merged:
        aligned_dir = (data_root / date_str / "aligned")
        aligned_dir.mkdir(parents=True, exist_ok=True)
        out_path = aligned_dir / f"aligned_imu_dvl_volt_{date_str}.csv"
        merged.to_csv(out_path, index=False)
        print(f"[INFO] 融合数据已保存: {out_path}")
        save_prefix = aligned_dir / f"aligned_imu_dvl_volt_{date_str}"

    # 可视化
    if not args.no_plot:
        plot_basic(merged, show=True, save_prefix=save_prefix)
    else:
        print("[INFO] no-plot 模式，不生成图像。")


if __name__ == "__main__":
    main()
