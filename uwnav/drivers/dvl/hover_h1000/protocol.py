# uwnav/drivers/dvl/hover_h1000/protocol.py
# -*- coding: utf-8 -*-

"""
Hover H1000 PD6/EPD6 协议解析器（定宽优先，兼容粘连/残缺）。
返回统一字段：
  - 速度(毫米/秒)：ve, vn, vu
  - 位移/高度(米)：e, n, u, depth
  - 其他：pitch, roll, heading, ts(设备时间字符串或计数), salinity, temperature, sound_speed, status
  - 有效位：valid(True/False/None)，src(帧类型, 如 BE/BS/BI/BD/SA/TS)

解析原则：
  * 先按手册固定宽度切片（PD6：速度 6 宽；位移 8 宽；深度 6 宽；误差 4 宽；标志 1 宽）
  * 若定宽失败，回退逗号分隔+正则提取数字
  * 占位符 "88888", "+88888", "-88888" 以及 "nan" 视为无效→返回 None（不当作 0）
  * 一行可能含多帧（如 ":SA...:TS...:BE..."），请用 parse_lines(raw) 解析
"""

from __future__ import annotations
import re
from typing import List, Dict, Optional

# ====================== 工具函数 ======================

_PLACEHOLDERS = {"88888", "+88888", "-88888"}
_NAN_TOKENS = {"nan", "+nan", "-nan", "NaN", "NAN", "Nan"}

def _strip_quotes(s: str) -> str:
    s = s.strip()
    if len(s) >= 2 and ((s[0] == s[-1] == '"') or (s[0] == s[-1] == "'")):
        return s[1:-1].strip()
    return s

def _is_placeholder(s: str) -> bool:
    return s.strip() in _PLACEHOLDERS

def _is_nan_token(s: str) -> bool:
    return s.strip() in _NAN_TOKENS

def _clean_spaces(s: str) -> str:
    # 去除字符串中多余的内部空格（不用于定宽切片，仅用于回退解析）
    return re.sub(r"\s+", "", s.strip())

def _take_numeric_int(token: str) -> Optional[float]:
    """
    速度字段用，整数（mm/s）。占位/NaN→None；尽量从混乱字符串抓一个 +/-digits。
    """
    if not token:
        return None
    t = token.strip()
    if _is_nan_token(t) or _is_placeholder(t):
        return None
    # 纯整型
    try:
        return float(int(t))
    except Exception:
        pass
    m = re.search(r"([+\-]?\d{1,6})", t)
    if not m:
        return None
    v = m.group(1)
    if _is_placeholder(v) or _is_nan_token(v):
        return None
    try:
        return float(int(v))
    except Exception:
        return None

def _take_numeric_float(token: str) -> Optional[float]:
    """
    位移/深度用（m）。占位/NaN→None；尽量从混乱字符串抓一个 +/-digits(.digits)。
    """
    if not token:
        return None
    t = token.strip()
    if _is_nan_token(t):
        return None
    if "88888" in t:
        return None
    # 常规浮点
    try:
        return float(_clean_spaces(t))
    except Exception:
        pass
    m = re.search(r"([+\-]?\d+(?:\.\d+)?)", t)
    if not m:
        return None
    v = m.group(1)
    if "88888" in v or _is_nan_token(v):
        return None
    try:
        return float(v)
    except Exception:
        return None

def _valid_flag(token: str) -> Optional[bool]:
    if not token:
        return None
    t = token.strip().upper()
    if t == "A": return True
    if t == "V": return False
    return None

def _split_fixed(payload: str, widths: List[int]) -> Optional[List[str]]:
    """
    定宽切片：从去掉前导逗号/空格的 payload 开始，按 widths 逐段切片。
    注意：PD6 每段间常见有逗号分隔；此处策略：
      - 先移除所有逗号，仅按宽度切片
      - 切片后 strip()
    """
    if not payload:
        return None
    s = payload.lstrip(" ,")
    # 去掉逗号（手册给的是定宽+逗号，直接去逗号更稳）
    s = s.replace(",", " ")
    s = re.sub(r"\s+", " ", s).strip()
    # 为避免合并过度，先把空格去掉，仅保留正负号和数字与字母（A/V）
    # 但位移/深度有小数点，需要保留 '.'
    # 暂不删除空格，定宽直接从原串连续取片段（空格算占位）
    pos = 0
    out = []
    try:
        for w in widths:
            seg = s[pos:pos+w]
            out.append(seg.strip())
            pos += w
        return out
    except Exception:
        return None

def _split_csv(payload: str, expect: int) -> List[str]:
    parts = [p.strip() for p in payload.lstrip(" ,").split(",")]
    while len(parts) < expect:
        parts.append("")
    return parts[:expect]

def _split_fixed_or_csv(payload: str, widths: List[int], expect: int) -> List[str]:
    fixed = _split_fixed(payload, widths)
    if fixed is not None and len(fixed) >= expect:
        return fixed[:expect]
    return _split_csv(payload, expect)

# ====================== 各帧解析 ======================

def _parse_SA(payload: str) -> Optional[Dict]:
    # :SA, pitch, roll, heading
    parts = _split_csv(payload, 3)
    # 回退：用正则抓 3 个浮点
    if not all(parts):
        nums = re.findall(r"[+\-]?\d+(?:\.\d+)?", payload)
        if len(nums) >= 3:
            parts = nums[:3]
    try:
        return {
            "src": "SA",
            "pitch": float(parts[0]),
            "roll": float(parts[1]),
            "heading": float(parts[2]),
        }
    except Exception:
        return None

def _parse_TS(payload: str) -> Optional[Dict]:
    # :TS, ts, sal, temp, depth, sound_speed, status
    parts = _split_csv(payload, 6)
    # 回退正则
    if not parts[0] or not parts[1] or not parts[2]:
        nums = re.findall(r"[+\-]?\d+(?:\.\d+)?", payload)
        if len(nums) >= 6:
            parts = nums[:6]
    try:
        return {
            "src": "TS",
            "ts": parts[0],  # 原始设备计数/时间字符串（不强转）
            "salinity": float(parts[1]),
            "temperature": float(parts[2]),
            "depth": _take_numeric_float(parts[3]),
            "sound_speed": float(parts[4]),
            "status": parts[5],
        }
    except Exception:
        return None

def _parse_BI(payload: str) -> Optional[Dict]:
    # :BI, vx, vy, vz, err, A/V   (体坐标；mm/s)
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,4,1], expect=5)
    vx = _take_numeric_int(fields[0])
    vy = _take_numeric_int(fields[1])
    vz = _take_numeric_int(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src": "BI", "ve": vx, "vn": vy, "vu": vz, "valid": valid}

def _parse_BS(payload: str) -> Optional[Dict]:
    # :BS, vx, vy, vz, A/V       (体坐标；mm/s)
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    vx = _take_numeric_int(fields[0])
    vy = _take_numeric_int(fields[1])
    vz = _take_numeric_int(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src": "BS", "ve": vx, "vn": vy, "vu": vz, "valid": valid}

def _parse_BE(payload: str) -> Optional[Dict]:
    # :BE, ve, vn, vu, A/V       (东北天；mm/s)
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    ve = _take_numeric_int(fields[0])
    vn = _take_numeric_int(fields[1])
    vu = _take_numeric_int(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src": "BE", "ve": ve, "vn": vn, "vu": vu, "valid": valid}

def _parse_WI(payload: str) -> Optional[Dict]:
    # :WI, vx, vy, vz, err, A/V  (某些固件的体坐标速度；与 BI 类似)
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,4,1], expect=5)
    vx = _take_numeric_int(fields[0])
    vy = _take_numeric_int(fields[1])
    vz = _take_numeric_int(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src": "WI", "ve": vx, "vn": vy, "vu": vz, "valid": valid}

def _parse_WS(payload: str) -> Optional[Dict]:
    # :WS, vx, vy, vz, A/V       (某些固件的体坐标速度简版)
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    vx = _take_numeric_int(fields[0])
    vy = _take_numeric_int(fields[1])
    vz = _take_numeric_int(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src": "WS", "ve": vx, "vn": vy, "vu": vz, "valid": valid}

def _parse_WE(payload: str) -> Optional[Dict]:
    # :WE, ve, vn, vu, A/V       (某些固件的东北天速度；与 BE 类似)
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    ve = _take_numeric_int(fields[0])
    vn = _take_numeric_int(fields[1])
    vu = _take_numeric_int(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src": "WE", "ve": ve, "vn": vn, "vu": vu, "valid": valid}

def _parse_BD(payload: str) -> Optional[Dict]:
    # :BD, e, n, u, depth, q     (东北天位移/高度；米)
    fields = _split_fixed_or_csv(payload, widths=[8,8,8,6,4], expect=5)
    e = _take_numeric_float(fields[0])
    n = _take_numeric_float(fields[1])
    u = _take_numeric_float(fields[2])
    depth = _take_numeric_float(fields[3])
    return {"src": "BD", "e": e, "n": n, "u": u, "depth": depth}

def _parse_WD(payload: str) -> Optional[Dict]:
    # :WD, dx, dy, dz, depth, q  (体坐标位移/高度；米)
    fields = _split_fixed_or_csv(payload, widths=[8,8,8,6,4], expect=5)
    e = _take_numeric_float(fields[0])
    n = _take_numeric_float(fields[1])
    u = _take_numeric_float(fields[2])
    depth = _take_numeric_float(fields[3])
    return {"src": "WD", "e": e, "n": n, "u": u, "depth": depth}

# 帧类型到解析函数的映射
_PARSERS = {
    "SA": _parse_SA,
    "TS": _parse_TS,
    "BI": _parse_BI,
    "BS": _parse_BS,
    "BE": _parse_BE,
    "WI": _parse_WI,
    "WS": _parse_WS,
    "WE": _parse_WE,
    "BD": _parse_BD,
    "WD": _parse_WD,
}

# 只把真正的数据帧起点当作 frame boundary，避免把命令回显/噪声切成伪帧。
_FRAME_TAGS = tuple(_PARSERS.keys())
_FRAME_START_RE = re.compile(
    r":" + r"(?=(?:" + "|".join(_FRAME_TAGS) + r")(?:,|\s|$))"
)
# ====================== 主入口 ======================

def parse_line(line: str) -> Optional[Dict]:
    """
    解析单帧（行首需有 ":"），返回 dict 或 None。

    这里只接受已知数据帧类型；命令回显和噪声片段直接返回 None，
    避免上层把它们当作有效 DVLData 落入 parsed/TB 文件。
    """
    if not line:
        return None
    s = _strip_quotes(line)

    # 清理多余冒号（一些设备/回显会出现 '::::CS ...'）
    while s.startswith("::"):
        s = s[1:]
    if not s.startswith(":") or len(s) < 3:
        return None

    msg = s[1:3].upper()
    payload = s[3:]

    fn = _PARSERS.get(msg)
    if fn is None:
        return None

    try:
        pkt = fn(payload)
        if pkt:
            return pkt
    except Exception:
        pass
    return None


def parse_lines(raw: str) -> List[Dict]:
    """
    一行可能含多帧。

    旧实现按所有 ':' 生切，真实样本中会把 `CZ` 回显、乱码和半截 payload
    误切成伪帧；这里改为只从已知数据帧起点切块。
    """
    if not raw:
        return []
    s = _strip_quotes(raw)
    if ":" not in s:
        return []

    starts = [m.start() for m in _FRAME_START_RE.finditer(s)]
    if not starts:
        return []

    out: List[Dict] = []
    for idx, start in enumerate(starts):
        end = starts[idx + 1] if (idx + 1) < len(starts) else len(s)
        chunk = s[start:end].strip()
        pkt = parse_line(chunk)
        if pkt:
            out.append(pkt)
    return out
