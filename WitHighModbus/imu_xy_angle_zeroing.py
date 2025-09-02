import time
import serial

# 指令定义
unlock_cmd = bytes([0x50, 0x06, 0x00, 0x69, 0xB5, 0x88, 0x22, 0xA1])  # 解锁
calib_cmd = bytes([0x50, 0x06, 0x00, 0x01, 0x00, 0x04, 0xD4, 0x48])  # 相对归零校准XY轴
save_cmd = bytes([0x50, 0x06, 0x00, 0x00, 0x00, 0x00, 0x84, 0x4B])  # 保存


def send_command(ser, cmd):
    ser.write(cmd)
    ser.flush()


if __name__ == "__main__":
    # 根据实际设备名称和波特率进行修改
    # 假设您已知道IMU当前通信波特率并已在此处设定
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.5)
    if ser.is_open:
        print("Serial port opened.")

        # 1. 解锁
        send_command(ser, unlock_cmd)
        time.sleep(0.1)  # 延时100ms

        # 2. 校准XY轴相对归零
        send_command(ser, calib_cmd)
        time.sleep(1.0)  # 延时1s

        # 3. 保存
        send_command(ser, save_cmd)
        time.sleep(0.1)

        print("XY axis angle zeroing completed and saved.")

        ser.close()
    else:
        print("Failed to open serial port.")
