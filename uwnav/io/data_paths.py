# uwnav/io/data_paths.py
from __future__ import annotations
from pathlib import Path
from datetime import datetime
from typing import Optional


def get_sensor_outdir(sensor: str, user_outdir: Optional[str] = None) -> Path:
    """
    生成统一格式的数据目录:
        <data_root>/<YYYY-MM-DD>/<sensor>/

    参数：
    - sensor: "dvl", "imu", "volt", "usbl", "camera", ...
    - user_outdir: 
        None => 默认使用 <repo_root>/data
        非 None => 作为 data 根目录（例如 /mnt/ssd/data）

    返回 Path 对象，并确保目录已创建。
    """

    # 1) 确定 data 根目录
    if user_outdir:
        base = Path(user_outdir)
    else:
        # uwnav/io/data_paths.py 所在位置：repo_root/uwnav/io
        base = Path(__file__).resolve().parents[2] / "data"

    # 2) 日期目录
    today = datetime.now().strftime("%Y-%m-%d")

    # 3) 传感器子目录
    out_dir = base / today / sensor
    out_dir.mkdir(parents=True, exist_ok=True)
    return out_dir
