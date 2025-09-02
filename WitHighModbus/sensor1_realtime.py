# sensor1_realtime.py
# coding: UTF-8

import os
import sys
import time
import csv
import threading
import signal
import numpy as np

import device_model
from filters import RealTimeIMUFilter

# 用于批量处理的缓冲区：存放 (timestamp, accel_sample, gyro_sample)
sample_buffer = []

def main():
    FS = 50  # 采样频率 (Hz)

    # 生成日志文件名，保存在脚本所在目录
    base_dir = os.path.dirname(os.path.abspath(__file__))
    date_str = time.strftime("%Y%m%d")
    raw_filename = os.path.join(base_dir, f"imu_raw_data_{date_str}.csv")
    filtered_filename = os.path.join(base_dir, f"imu_filtered_data_{date_str}.csv")

    file_lock = threading.Lock()
    shutdown_event = threading.Event()

    # 打开原始数据日志文件
    try:
        raw_log_file = open(raw_filename, 'a', newline='', encoding='utf-8')
        raw_writer = csv.writer(raw_log_file)
        if raw_log_file.tell() == 0:
            # 原始数据列（包含 HX, HY, HZ, AngX, AngY, AngZ）
            raw_header = [
                'Timestamp',
                'AccX', 'AccY', 'AccZ',
                'AsX', 'AsY', 'AsZ',
                'HX', 'HY', 'HZ',
                'AngX', 'AngY', 'AngZ'
            ]
            raw_writer.writerow(raw_header)
            raw_log_file.flush()
            print(f"已创建原始数据日志文件: {raw_filename}")
        else:
            print(f"已打开原始数据日志文件: {raw_filename}")
    except IOError as e:
        print(f"无法打开原始数据日志文件 {raw_filename}: {e}")
        sys.exit(1)

    # 打开滤波后数据日志文件（不打印到终端，只保存）
    try:
        filt_log_file = open(filtered_filename, 'a', newline='', encoding='utf-8')
        filt_writer = csv.writer(filt_log_file)
        if filt_log_file.tell() == 0:
            # 滤波后数据列
            filt_header = [
                'Timestamp',
                'AccX_filt', 'AccY_filt', 'AccZ_filt',
                'AsX_filt', 'AsY_filt', 'AsZ_filt',
                'Yaw_filt(deg)'
            ]
            filt_writer.writerow(filt_header)
            filt_log_file.flush()
            print(f"已创建滤波后数据日志文件: {filtered_filename}")
        else:
            print(f"已打开滤波后数据日志文件: {filtered_filename}")
    except IOError as e:
        print(f"无法打开滤波后数据日志文件 {filtered_filename}: {e}")
        sys.exit(1)

    # 初始化实时滤波器（零偏校准 + 低通滤波 + Z轴解缠）
    rt_filter = RealTimeIMUFilter(fs=FS, cutoff=4.0, calibrate=True)

    def updateData(device):
        global sample_buffer
        if shutdown_event.is_set():
            return

        # 生成高精度时间戳（精确到毫秒）
        from datetime import datetime
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')

        try:
            # 读取6轴主要数据
            accel_sample = np.array([
                device.get("AccX"),
                device.get("AccY"),
                device.get("AccZ")
            ])
            gyro_sample = np.array([
                device.get("AsX"),
                device.get("AsY"),
                device.get("AsZ")
            ])
            # 其它原始数据
            HX = device.get("HX")
            HY = device.get("HY")
            HZ = device.get("HZ")
            AngX = device.get("AngX")
            AngY = device.get("AngY")
            AngZ = device.get("AngZ")
        except Exception as e:
            print(f"读取IMU数据失败: {e}")
            return

        # 写入原始数据（高精度时间戳）
        raw_row = [
            timestamp,
            accel_sample[0], accel_sample[1], accel_sample[2],
            gyro_sample[0], gyro_sample[1], gyro_sample[2],
            HX, HY, HZ,
            AngX, AngY, AngZ
        ]
        with file_lock:
            raw_writer.writerow(raw_row)
            raw_log_file.flush()
        print(f"写入原始数据: {raw_row}")

        # 将主要数据加入缓冲区，后续批处理滤波
        sample_buffer.append((time.time(), accel_sample, gyro_sample))
        if sample_buffer and (time.time() - sample_buffer[0][0] >= 0.1):
            last_result = None
            for (t, a, g) in sample_buffer:
                res = rt_filter.process_sample(t, a, g)
                if res is not None:
                    last_result = res
            if last_result is not None:
                filt_accel, filt_gyro, yaw_deg = last_result
                filt_row = [timestamp] + list(filt_accel) + list(filt_gyro) + [yaw_deg]
                with file_lock:
                    filt_writer.writerow(filt_row)
                    filt_log_file.flush()
            sample_buffer.clear()

    def do_shutdown():
        print("正在关闭程序...")
        shutdown_event.set()
        time.sleep(0.5)
        with file_lock:
            raw_log_file.close()
            filt_log_file.close()
        device.closeDevice()
        print("设备已关闭，日志文件保存完毕。")
        sys.exit(0)

    def signal_handler(sig, frame):
        print("\n检测到退出信号。")
        do_shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 初始化 IMU 设备（波特率115200），绑定回调
    device = device_model.DeviceModel(
        deviceName="IMU实时测试设备",
        portName="/dev/ttyUSB0",  # 请根据实际情况修改
        baud=115200,
        ADDR=0x50,
        callback_method=updateData
    )
    device.openDevice()
    device.startLoopRead()

    print("实时IMU采集、滤波和保存程序正在运行。输入 's' 并按回车以退出。")

    def input_listener():
        while not shutdown_event.is_set():
            if input().strip().lower() == 's':
                do_shutdown()
    listener_thread = threading.Thread(target=input_listener, daemon=True)
    listener_thread.start()

    while not shutdown_event.is_set():
        time.sleep(1)

if __name__ == "__main__":
    main()
