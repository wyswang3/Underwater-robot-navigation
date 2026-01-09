#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time
import serial

# ===== 基本配置（按需修改） =====
DEVICE = '/dev/ttyUSB0'   # Windows 用 'COMx'
BAUD   = 230400
PAUSE_UNLOCK = 0.1        # 步骤间等待
PAUSE_CALIB  = 1.0        # 归零动作建议多给一点时间

# ===== 固定指令（已含 CRC，高字节在前，按厂商变体） =====
unlock_cmd = bytes([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1])  # 解锁
calib_cmd  = bytes([0x50, 0x06, 0x00, 0x01, 0x00, 0x04, 0xD4, 0x48])  # XY 相对归零
save_cmd   = bytes([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])  # 保存

def send(ser: serial.Serial, cmd: bytes, label: str, pause: float):
    print(f"[.. ] {label}  cmd={cmd.hex(' ').upper()}")
    ser.write(cmd)
    ser.flush()
    time.sleep(pause)
    print(f"[OK ] {label} 完成")

def main():
    print(f"[INFO] 打开串口：{DEVICE} @ {BAUD}")
    try:
        ser = serial.Serial(DEVICE, BAUD, timeout=0.5)
    except serial.SerialException as e:
        print(f"[ERR ] 无法打开串口：{e}")
        return

    if not ser.is_open:
        print("[ERR ] 串口未打开")
        return

    try:
        print("[TIP ] 请将 IMU 平放并保持静止（防震、避免磁干扰）")
        time.sleep(0.5)

        # 1) 解锁
        send(ser, unlock_cmd, "解锁寄存器", PAUSE_UNLOCK)

        # 2) XY 相对归零
        send(ser, calib_cmd, "XY 轴相对归零", PAUSE_CALIB)

        # 3) 保存
        send(ser, save_cmd, "保存设置", PAUSE_UNLOCK)

        print("[DONE] XY 轴角度归零完成并已保存。注意：这是“相对归零”，不会改变 Z/Yaw。")
    finally:
        ser.close()
        print("[INFO] 串口已关闭")

if __name__ == "__main__":
    main()