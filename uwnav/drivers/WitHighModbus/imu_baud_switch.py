#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import serial

DEVICE    = '/dev/ttyUSB0'   # Windows 请改为 'COMx'
OLD_BAUD  = 9600
NEW_BAUD  = 115200           # 默认目标：115200

# —— 固定指令（已含 CRC） ——
unlock_cmd          = bytes([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1])
set_baud_115200_cmd = bytes([0x50, 0x06, 0x00, 0x04, 0x00, 0x06, 0x45, 0x88])
set_baud_230400_cmd = bytes([0x50, 0x06, 0x00, 0x04, 0x00, 0x07, 0x84, 0x48])
save_cmd            = bytes([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])

def _change_baud(target_baud: int, set_cmd: bytes, device: str = DEVICE,
                 old_baud: int = OLD_BAUD, pause: float = 0.1) -> None:
    """在不改变原有流程的前提下，切换波特率并保存"""
    print(f"[{device}] Baud change: {old_baud}  →  {target_baud}")

    # 1) 旧速率打开
    ser = serial.Serial(device, baudrate=old_baud, timeout=0.5)
    if not ser.is_open:
        print(f"[ERR] open {device} @ {old_baud} failed")
        return
    print(f"[OK ] open @ {old_baud}")

    # 2) 解锁（旧速率）
    ser.write(unlock_cmd); ser.flush(); time.sleep(pause)
    print(f"[.. ] unlock @ {old_baud}")

    # 3) 写入新波特率（旧速率）
    ser.write(set_cmd); ser.flush(); time.sleep(pause)
    print(f"[.. ] set baud → {target_baud}  (write-reg)")

    # 4) 关闭并以新速率重开
    ser.close()
    ser = serial.Serial(device, baudrate=target_baud, timeout=0.5)
    if not ser.is_open:
        print(f"[ERR] reopen {device} @ {target_baud} failed")
        return
    print(f"[OK ] reopen @ {target_baud}")

    # 5) 再次解锁 + 保存（新速率）
    ser.write(unlock_cmd); ser.flush(); time.sleep(pause)
    print(f"[.. ] unlock @ {target_baud}")
    ser.write(save_cmd); ser.flush(); time.sleep(pause)
    print(f"[.. ] save settings")

    ser.close()
    print(f"[DONE] Baud persisted: {old_baud}  →  {target_baud}")

# —— 两个便捷入口（保留原有 115200，新增 230400） ——
def change_to_115200():
    _change_baud(115200, set_baud_115200_cmd)

def change_to_230400():
    _change_baud(230400, set_baud_230400_cmd)

if __name__ == "__main__":
    # 默认行为：切到 115200，与原逻辑等价
    change_to_115200()

    # 需要 230400 时，改成：
    # change_to_230400()
