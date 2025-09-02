import time
import serial

DEVICE = '/dev/ttyUSB0'  # USB转接器对应的设备号（根据实际情况修改）
old_baud = 9600
new_baud = 115200

# 指令定义
unlock_cmd = bytes([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1])
set_baud_cmd = bytes([0x50, 0x06, 0x00, 0x04, 0x00, 0x06, 0x45, 0x88])
save_cmd = bytes([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])

def main():
    # 以旧波特率打开串口
    ser = serial.Serial(DEVICE, baudrate=old_baud, timeout=0.5)
    if not ser.is_open:
        print(f"Failed to open {DEVICE}")
        return
    print(f"Open {DEVICE} at {old_baud} baud success.")

    # 1. 解锁
    ser.write(unlock_cmd)
    ser.flush()
    time.sleep(0.1)

    # 2. 设置新波特率
    ser.write(set_baud_cmd)
    ser.flush()
    time.sleep(0.1)

    # 关闭串口，以新波特率重新打开
    ser.close()
    ser = serial.Serial(DEVICE, baudrate=new_baud, timeout=0.5)
    if not ser.is_open:
        print(f"Failed to reopen {DEVICE} at {new_baud} baud")
        return
    print(f"Reopen {DEVICE} at {new_baud} baud success.")

    # 3. 再次解锁（以新波特率）
    ser.write(unlock_cmd)
    ser.flush()
    time.sleep(0.1)

    # 4. 保存设置
    ser.write(save_cmd)
    ser.flush()
    time.sleep(0.1)

    print(f"Baudrate changed and saved. The IMU should now work at {new_baud} baud.")

    ser.close()

if __name__ == "__main__":
    main()
