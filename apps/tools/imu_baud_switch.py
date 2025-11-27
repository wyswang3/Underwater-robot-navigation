#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time, serial

DEVICE   = "/dev/ttyUSB1"
OLD_BAUD = 115200
NEW_BAUD = 230400

# 固定指令（含 CRC）
unlock_cmd          = bytes([0x50,0x06,0x00,0x69,0xB5,0x88,0x22,0xA1])
set_baud_115200_cmd = bytes([0x50,0x06,0x00,0x04,0x00,0x06,0x45,0x88])
set_baud_230400_cmd = bytes([0x50,0x06,0x00,0x04,0x00,0x07,0x84,0x48])
save_cmd            = bytes([0x50,0x06,0x00,0x00,0x00,0x00,0x84,0x4B])

def _read_reply(ser, expect_fn=0x06, timeout=0.4):
    """读取一帧应答；返回 (ok, raw_bytes, exc_code)。异常帧: fn|0x80, exc_code=0x01/0x02/0x03"""
    t0 = time.time()
    buf = b""
    ser.timeout = 0.1
    while time.time() - t0 < timeout:
        b = ser.read(ser.in_waiting or 1)
        if b:
            buf += b
            # Modbus 写单寄存器回显固定 8+2 字节；我们只做粗判
            if len(buf) >= 8:
                break
        else:
            time.sleep(0.01)
    if not buf:
        return False, b"", None
    # 异常帧：功能码 = 原功能码|0x80
    if len(buf) >= 3 and (buf[1] & 0x80):
        return False, buf, buf[2]
    # 正常帧：功能码为 expect_fn
    ok = (len(buf) >= 2 and buf[1] == expect_fn)
    return ok, buf, None

def _send(ser, name, cmd, wait_reply=True):
    ser.reset_input_buffer(); ser.reset_output_buffer()
    ser.write(cmd); ser.flush()
    time.sleep(0.05)
    if not wait_reply:
        print(f"[TX ] {name} sent ({len(cmd)}B)")
        return True
    ok, raw, exc = _read_reply(ser)
    if ok:
        print(f"[ACK] {name} ok  resp={raw.hex(' ')}")
        return True
    if exc is not None:
        print(f"[ERR] {name} exception code=0x{exc:02X} resp={raw.hex(' ')}")
    else:
        print(f"[ERR] {name} no/unknown reply resp={raw.hex(' ')}")
    return False

def change_to(target_baud, set_cmd, device=DEVICE, old_baud=OLD_BAUD):
    print(f"[INFO] change baud {device}: {old_baud} -> {target_baud}")

    # 路径 A：同连接完成 解锁→设置→保存，然后断开；等待设备切速
    try:
        ser = serial.Serial(device, baudrate=old_baud, timeout=0.3, inter_byte_timeout=0.1)
    except Exception as e:
        print(f"[FATAL] open {device}@{old_baud} failed: {e}")
        return

    ok = True
    ok &= _send(ser, "unlock(old)", unlock_cmd)
    time.sleep(0.3)
    ok &= _send(ser, f"set_baud->{target_baud}", set_cmd)
    time.sleep(0.3)
    ok &= _send(ser, "save", save_cmd)
    ser.close()

    # 给设备一点时间切换波特率
    time.sleep(0.5)

    # 尝试用新波特率重连
    try:
        ser = serial.Serial(device, baudrate=target_baud, timeout=0.4)
        ser.close()
        print(f"[OK ] reopened @ {target_baud}  (路径A成功)")
        return
    except Exception as e:
        print(f"[WARN] reopen {device}@{target_baud} failed after path A: {e}")

    # 路径 B：先 set 后立即断开；再用旧波特率重连做 save（兼容“set 不立即生效”的设备）
    try:
        ser = serial.Serial(device, baudrate=old_baud, timeout=0.3, inter_byte_timeout=0.1)
        _send(ser, "unlock(old)#2", unlock_cmd)
        time.sleep(0.2)
        _send(ser, f"set_baud->{target_baud}#2", set_cmd)
        time.sleep(0.2)
        _send(ser, "save#2", save_cmd)
        ser.close()
        time.sleep(0.5)
        ser = serial.Serial(device, baudrate=target_baud, timeout=0.4)
        ser.close()
        print(f"[OK ] reopened @ {target_baud}  (路径B成功)")
        return
    except Exception as e:
        print(f"[FAIL] path B still failed: {e}")
        print("[HINT] 检查: 1) 端口是否正确; 2) 设备是否支持 230400; 3) 需要上电重启后生效？")

if __name__ == "__main__":
    # 改到 230400（如需 115200 换 set_cmd）
    change_to(NEW_BAUD, set_baud_230400_cmd)