# uwnav/drivers/dvl/hover_h1000/protocol.py
# -*- coding: utf-8 -*-

"""
Hover H1000 PD6/EPD6 解析器（定宽优先，A/V 有效位）
返回统一字段：
  - 速度(毫米/秒)：ve, vn, vu
  - 位移/高度(米)：e, n, u, depth
  - 其他：pitch, roll, heading, ts(原始计数/时间字串), valid(True/False), src(帧类型)
占位符 88888 / +88888 / -88888 视为 0。
"""

from __future__ import annotations
import re
from typing import List, Dict, Optional

# -------------------- 基础工具 --------------------

_PLACEHOLDER_SET = {"88888", "+88888", "-88888"}

def _strip_quotes(s: str) -> str:
    s = s.strip()
    if len(s) >= 2 and ((s[0] == s[-1] == '"') or (s[0] == s[-1] == "'")):
        return s[1:-1].strip()
    return s

def _is_placeholder(s: str) -> bool:
    return s.strip() in _PLACEHOLDER_SET

def _as_int_mm(s: str) -> Optional[float]:
    """速度字段（mm/s）。占位符→0；异常→None；尽量从混乱字符串摘数字"""
    s0 = s.strip()
    if not s0:
        return None
    if _is_placeholder(s0):
        return 0.0
    # 常规：纯整型
    try:
        return float(int(s0))
    except Exception:
        # 兜底：抓 +/-digits
        m = re.search(r'([+\-]?\d{1,6})', s0)
        if not m:
            return None
        v = m.group(1)
        if _is_placeholder(v):
            return 0.0
        try:
            return float(int(v))
        except Exception:
            return None

def _as_float_m(s: str) -> Optional[float]:
    """位移/深度（m）。占位符→0；异常→None"""
    s0 = s.strip().replace(' ', '')
    if not s0:
        return None
    if "88888" in s0:
        return 0.0
    try:
        return float(s0)
    except Exception:
        m = re.search(r'([+\-]?\d+(?:\.\d+)?)', s0)
        if not m:
            return None
        v = m.group(1)
        if "88888" in v:
            return 0.0
        try:
            return float(v)
        except Exception:
            return None

def _valid_flag(token: str) -> Optional[bool]:
    t = token.strip().upper()
    if t == "A": return True
    if t == "V": return False
    return None

def _split_fixed_or_csv(payload: str, widths: List[int], expect: int) -> List[str]:
    """
    先尝试定宽切片（去掉起始逗号/空格），失败再按逗号分割，不足填空。
    """
    s = payload.lstrip(' ,')
    # 定宽
    pos = 0
    out = []
    try:
        for w in widths:
            out.append(s[pos:pos+w])
            pos += w + 1  # 宽度之间通常还有一个逗号或空格，+1 允许轻微错位
        if len(out) >= expect:
            return [x.strip() for x in out[:expect]]
    except Exception:
        pass
    # 逗号
    parts = [p.strip() for p in payload.split(',')]
    while len(parts) < expect:
        parts.append("")
    return parts[:expect]

# -------------------- 各帧解析 --------------------

def _parse_SA(payload: str) -> Optional[Dict]:
    # :SA, pitch, roll, heading
    # 定宽意义小，优先逗号/正则
    parts = [p for p in payload.lstrip(' ,').split(',') if p != ""]
    if len(parts) < 3:
        nums = re.findall(r'[+\-]?\d+(?:\.\d+)?', payload)
        if len(nums) < 3:
            return None
        parts = nums[:3]
    try:
        return {
            "src":"SA",
            "pitch": float(parts[0]),
            "roll": float(parts[1]),
            "heading": float(parts[2]),
        }
    except Exception:
        return None

def _parse_TS(payload: str) -> Optional[Dict]:
    # :TS, ts, sal, temp, depth, sound_speed, status
    parts = [p.strip() for p in payload.lstrip(' ,').split(',')]
    if len(parts) < 6:
        nums = re.findall(r'[+\-]?\d+(?:\.\d+)?', payload)
        if len(nums) < 6:
            return None
        parts = [nums[0], nums[1], nums[2], nums[3], nums[4], nums[5]]
    try:
        return {
            "src":"TS",
            "ts": parts[0],
            "salinity": float(parts[1]),
            "temperature": float(parts[2]),
            "depth": _as_float_m(parts[3]),
            "sound_speed": float(parts[4]),
            "status": parts[5],
        }
    except Exception:
        return None

def _parse_WI(payload: str) -> Optional[Dict]:
    # :WI, vx, vy, vz, err, A/V   （体坐标速度＋误差）
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,4,1], expect=5)
    vx = _as_int_mm(fields[0]); vy = _as_int_mm(fields[1]); vz = _as_int_mm(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src":"WI", "ve":vx, "vn":vy, "vu":vz, "valid":valid}

def _parse_BI(payload: str) -> Optional[Dict]:
    # :BI, vx, vy, vz, err, A/V
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,4,1], expect=5)
    vx = _as_int_mm(fields[0]); vy = _as_int_mm(fields[1]); vz = _as_int_mm(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src":"BI", "ve":vx, "vn":vy, "vu":vz, "valid":valid}

def _parse_WS(payload: str) -> Optional[Dict]:
    # :WS, vx, vy, vz, A/V  （体坐标简版）
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    vx = _as_int_mm(fields[0]); vy = _as_int_mm(fields[1]); vz = _as_int_mm(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src":"WS", "ve":vx, "vn":vy, "vu":vz, "valid":valid}

def _parse_BS(payload: str) -> Optional[Dict]:
    # :BS, vx, vy, vz, A/V
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    vx = _as_int_mm(fields[0]); vy = _as_int_mm(fields[1]); vz = _as_int_mm(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src":"BS", "ve":vx, "vn":vy, "vu":vz, "valid":valid}

def _parse_WE(payload: str) -> Optional[Dict]:
    # :WE, ve, vn, vu, A/V  （东北天）
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    ve = _as_int_mm(fields[0]); vn = _as_int_mm(fields[1]); vu = _as_int_mm(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src":"WE", "ve":ve, "vn":vn, "vu":vu, "valid":valid}

def _parse_BE(payload: str) -> Optional[Dict]:
    # :BE, ve, vn, vu, A/V
    fields = _split_fixed_or_csv(payload, widths=[6,6,6,1], expect=4)
    ve = _as_int_mm(fields[0]); vn = _as_int_mm(fields[1]); vu = _as_int_mm(fields[2])
    valid = _valid_flag(fields[-1])
    return {"src":"BE", "ve":ve, "vn":vn, "vu":vu, "valid":valid}

def _parse_WD(payload: str) -> Optional[Dict]:
    # :WD, dx, dy, dz, depth, q （体坐标位移/高度）
    fields = _split_fixed_or_csv(payload, widths=[8,8,8,6,4], expect=5)
    e = _as_float_m(fields[0]); n = _as_float_m(fields[1]); u = _as_float_m(fields[2])
    depth = _as_float_m(fields[3])
    return {"src":"WD", "e":e, "n":n, "u":u, "depth":depth}

def _parse_BD(payload: str) -> Optional[Dict]:
    # :BD, e, n, u, depth, q  （东北天位移/高度）
    fields = _split_fixed_or_csv(payload, widths=[8,8,8,6,4], expect=5)
    e = _as_float_m(fields[0]); n = _as_float_m(fields[1]); u = _as_float_m(fields[2])
    depth = _as_float_m(fields[3])
    return {"src":"BD", "e":e, "n":n, "u":u, "depth":depth}

# -------------------- 主入口 --------------------

_PARSERS = {
    "SA": _parse_SA,
    "TS": _parse_TS,
    "WI": _parse_WI,
    "BI": _parse_BI,
    "WS": _parse_WS,
    "BS": _parse_BS,
    "WE": _parse_WE,
    "BE": _parse_BE,
    "WD": _parse_WD,
    "BD": _parse_BD,
}

def parse_line(line: str) -> Optional[Dict]:
    """解析单帧（行首需有 :XX）。"""
    if not line:
        return None
    s = _strip_quotes(line)

    # 清理多余前缀，如 '::::CS ...' → '::CS ...' → ':CS ...'
    while s.startswith('::'):
        s = s[1:]
    if not s.startswith(':') or len(s) < 3:
        return None

    msg = s[1:3].upper()
    payload = s[3:]
    fn = _PARSERS.get(msg)
    if fn:
        try:
            return fn(payload)
        except Exception:
            return None
    # 未知类型，返回 src 以便上层记录
    return {"src": msg}

def parse_lines(raw: str) -> List[Dict]:
    """一行可能含多帧：用 ':' 分割再逐帧解析。"""
    if not raw:
        return []
    s = _strip_quotes(raw)
    # 切块时保留 ':' 让 parse_line 识别
    chunks = [f":{p}" for p in s.split(':') if p]
    out: List[Dict] = []
    for ck in chunks:
        pkt = parse_line(ck)
        if pkt:
            out.append(pkt)
    return out
