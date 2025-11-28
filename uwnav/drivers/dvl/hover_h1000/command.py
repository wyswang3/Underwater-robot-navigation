# uwnav/drivers/dvl/hover_h1000/command.py
from typing import Optional
from datetime import datetime

# 命令参数格式定义：format_spec, kind
# kind: "int" 用于左填充空格的整数宽度；"float" 用于固定宽度小数；None 表示无参；"special" 表示特殊组帧
CMD_FORMATS = {
    "PR":  ("{:>2d}",   "int"),   # 1..20, 2d
    "PM":  ("{:>2d}",   "int"),   # 1..20, 2d
    "DF":  ("{:>1d}",   "int"),   # 0/1/3, 1d (0:PD6,1:PLT,3:EPD6)
    "BX":  ("{:>5.2f}", "float"), # 0.50..80.00, 5.2f
    "WF":  ("{:>5.2f}", "float"), # 0.80..70.00, 5.2f
    "PV":  ("{:>5.2f}", "float"), # eg. 10.00, 5.2f
    "PC":  ("{:>7.2f}", "float"), # 1400.00..1600.00, 7.2f
    "PS":  ("{:>5.2f}", "float"), # 10.00..50.00, 5.2f
    "CS":  (None,       None),    # start measure
    "CZ":  (None,       None),    # stop measure
    "ST":  ("ST_BCD",   "special"),
}

def _pad_to_16(s: str) -> str:
    # s 已含回车 \r；末尾用 '0' 补齐至 16 字符
    if len(s) > 16:
        raise ValueError(f"Command too long (>16): {repr(s)}")
    return s + ("0" * (16 - len(s)))

def build_simple(cmd: str, val: Optional[float|int]=None) -> bytes:
    """
    构造除 ST 以外的固定 16 字符命令帧：'CMD⎵PARAM\\r' + '0' 补齐到 16 字符
    - int/float 参数会按手册要求左填充空格到规定宽度
    """
    cmd = cmd.upper().strip()
    if cmd not in CMD_FORMATS:
        raise ValueError(f"Unsupported cmd {cmd}")

    fmt, kind = CMD_FORMATS[cmd]
    if fmt is None:  # 无参命令 CS/CZ
        frame = f"{cmd} \r"
        return _pad_to_16(frame).encode("ascii")

    if kind == "int":
        if not isinstance(val, int):
            raise ValueError(f"{cmd} requires int")
        param = fmt.format(val)
    elif kind == "float":
        if not isinstance(val, (float,int)):
            raise ValueError(f"{cmd} requires float")
        param = fmt.format(float(val))
    elif kind == "special":
        raise ValueError("Use build_st() for ST")
    else:
        raise ValueError("Bad kind")

    frame = f"{cmd} {param}\r"
    return _pad_to_16(frame).encode("ascii")

def build_st(dt: Optional[datetime]=None) -> bytes:
    """
    时间同步 ST（BCD 的“对应 ASCII 字符”拼接）
    帧格式仍为固定 16 字符：'ST⎵<8字节BCD-ascii>\\r' + '0' 补齐
    """
    if dt is None:
        dt = datetime.utcnow()

    def _bcd_byte(n: int) -> int:
        return ((n // 10) << 4) | (n % 10)  # 0xYY

    yy = _bcd_byte(dt.year % 100)  # 00..99
    mm = _bcd_byte(dt.month)       # 01..12
    dd = _bcd_byte(dt.day)         # 01..31
    hh = _bcd_byte(dt.hour)        # 00..23
    mi = _bcd_byte(dt.minute)      # 00..59
    ss = _bcd_byte(dt.second)      # 00..59

    ms = dt.microsecond // 1000    # 0..999
    ms_hi = _bcd_byte(ms // 10)    # 百十位（两位BCD）
    ms_lo = _bcd_byte(ms % 10)     # 个位（单个BCD）

    payload = ''.join(chr(x) for x in (yy, mm, dd, hh, mi, ss, ms_hi, ms_lo))
    frame = f"ST {payload}\r"
    return _pad_to_16(frame).encode("latin1")  # 非可打印字节，latin1 原样发送
