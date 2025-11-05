# sensor1.py
# coding: UTF-8

import device_model
import time
import sys
import signal
import csv
from datetime import datetime
import threading
from tool import AutoFlushThread, InputListenerThread

def main():
    # 生成当天日期字符串，作为文件名后缀
    date_str = datetime.now().strftime('%Y%m%d')
    LOG_FILENAME = f"imu_data_{date_str}.csv"  # 如 imu_data_20250312.csv

    # 创建写文件的线程锁和停止事件
    file_lock = threading.Lock()
    shutdown_event = threading.Event()

    # 尝试打开CSV日志文件（追加模式）
    try:
        log_file = open(LOG_FILENAME, 'a', newline='', encoding='utf-8')
    except IOError as e:
        print(f"无法打开日志文件 {LOG_FILENAME}: {e}")
        sys.exit(1)

    csv_writer = csv.writer(log_file)

    # 如果文件为空，则写入表头
    if log_file.tell() == 0:
        csv_writer.writerow([
            'Timestamp',
            'AccX', 'AccY', 'AccZ',
            'AsX', 'AsY', 'AsZ',
            'HX', 'HY', 'HZ',
            'AngX', 'AngY', 'AngZ'
        ])
        log_file.flush()

    # 定义关闭程序的过程
    def do_shutdown():
        print("正在关闭程序...")
        # 停止设备数据读取
        device.stopLoopRead()
        shutdown_event.set()
        flush_thread.join()
        with file_lock:
            log_file.close()
        device.closeDevice()
        print("设备已关闭，日志文件已保存，程序退出。")
        sys.exit(0)

    # 定义信号处理函数（捕捉 Ctrl+C 和 SIGTERM）
    def signal_handler(sig, frame):
        print("\n检测到退出信号，正在准备关闭程序...")
        do_shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 定义数据更新回调函数，设备调用此函数传入最新数据
    def updateData(device):
        if shutdown_event.is_set():
            return

        # 使用高精度时间戳
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        acc_x = device.get("AccX")
        acc_y = device.get("AccY")
        acc_z = device.get("AccZ")
        as_x  = device.get("AsX")
        as_y  = device.get("AsY")
        as_z  = device.get("AsZ")
        hx    = device.get("HX")
        hy    = device.get("HY")
        hz    = device.get("HZ")
        ang_x = device.get("AngX")
        ang_y = device.get("AngY")
        ang_z = device.get("AngZ")

        data_row = [
            timestamp,
            acc_x, acc_y, acc_z,
            as_x, as_y, as_z,
            hx, hy, hz,
            ang_x, ang_y, ang_z
        ]
        print(f"IMU数据记录: {data_row}")

        try:
            with file_lock:
                csv_writer.writerow(data_row)
                log_file.flush()  # 立即写入磁盘
        except Exception as e:
            print(f"写入日志文件失败: {e}")

    # 启动自动刷新线程（定时刷新文件）
    flush_thread = AutoFlushThread(log_file, file_lock, shutdown_event)
    flush_thread.start()

    # 启动输入监听线程，输入 's' 即可退出程序
    input_thread = InputListenerThread(shutdown_event, do_shutdown)
    input_thread.start()

    # 初始化IMU设备，注意波特率设置为115200
    device = device_model.DeviceModel(
        deviceName="IMU测试设备",
        portName="/dev/ttyUSB1",  # 根据实际情况修改为对应串口
        baud=230400,
        ADDR=0x50,
        callback_method=updateData
    )

    device.openDevice()
    device.startLoopRead()

    print("IMU采集程序正在运行。输入 's' 并按回车以退出，或使用 Ctrl+C 强行退出。")
    try:
        while not shutdown_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        do_shutdown()

if __name__ == "__main__":
    main()
