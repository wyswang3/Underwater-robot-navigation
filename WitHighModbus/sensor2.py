# sensor2.py
# coding: UTF-8

import os
import time
import sys
import csv
from datetime import datetime
import threading
import signal
from tool_new import AutoFlushThread, InputListenerThread, SerialReaderThread

# 全局数据缓冲区，保存 16 个通道最新数据
data_buffer = {f'CH{i}': None for i in range(16)}
buffer_lock = threading.Lock()

def main():
    # 获取脚本所在目录，构造保存路径（保存到当前目录下）
    base_dir = os.path.dirname(os.path.abspath(__file__))
    date_str = datetime.now().strftime('%Y%m%d')  # 例如：20250312
    LOG_FILENAME = os.path.join(base_dir, f"motor_data_{date_str}.csv")

    # 创建线程锁和停止事件
    file_lock = threading.Lock()
    shutdown_event = threading.Event()

    # 打开日志文件（追加模式）
    try:
        log_file = open(LOG_FILENAME, 'a', newline='', encoding='utf-8')
        csv_writer = csv.writer(log_file)
        # 如果文件为空，写入表头
        if log_file.tell() == 0:
            header = ['Timestamp'] + [f'CH{i}' for i in range(16)]
            csv_writer.writerow(header)
            log_file.flush()
            print(f"已创建日志文件并写入标题: {LOG_FILENAME}")
        else:
            print(f"已打开现有日志文件: {LOG_FILENAME}")
    except IOError as e:
        print(f"无法打开日志文件 {LOG_FILENAME}: {e}")
        sys.exit(1)

    def log_received_data(data):
        """
        接收到串口数据后，将数据解析后存入全局缓冲区；
        当16个通道的数据齐全时，写入一行日志，并清空缓冲区。
        """
        current_time = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        try:
            # 预期数据格式： "CH7:2025\t1.631A" 或 "CH8:2200\t1.773V"
            if not data:
                return

            parts = data.split('\t')
            if len(parts) != 2:
                return

            channel_part, value_part = parts
            if ':' not in channel_part:
                return

            channel, adc_val = channel_part.split(':', 1)
            value_part = value_part.strip()
            # 提取数值和单位（保留字符串形式）
            if value_part.endswith('V') or value_part.endswith('A'):
                unit = value_part[-1]
                numeric = value_part[:-1].strip()
                value_str = f"{numeric}{unit}"
            else:
                value_str = value_part

            # 将解析结果写入缓冲区（线程安全）
            with buffer_lock:
                data_buffer[channel] = value_str
                # 检查是否所有通道都有数据
                if all(data_buffer[f'CH{i}'] is not None for i in range(16)):
                    row = [current_time] + [data_buffer[f'CH{i}'] for i in range(16)]
                    with file_lock:
                        csv_writer.writerow(row)
                        log_file.flush()  # 立即刷新数据到磁盘
                    # 仅输出一次完整采样的结果
                    print(f"写入日志行: {row}")
                    # 清空缓冲区，准备下一轮数据采集
                    for key in data_buffer.keys():
                        data_buffer[key] = None
        except Exception as e:
            print(f"写入日志文件失败: {e}")

    def do_shutdown():
        print("正在关闭程序...")
        shutdown_event.set()

        print("等待 AutoFlushThread 结束...")
        flush_thread.join()
        print("AutoFlushThread 已结束.")

        print("等待 InputListenerThread 结束...")
        input_thread.join()
        print("InputListenerThread 已结束.")

        print("等待 SerialReaderThread 结束...")
        serial_reader_thread.join()
        print("SerialReaderThread 已结束.")

        try:
            with file_lock:
                log_file.close()
            print(f"已关闭日志文件 {LOG_FILENAME}")
        except Exception as e:
            print(f"关闭日志文件时出错: {e}")

        print("程序已退出。")
        sys.exit(0)

    def signal_handler(sig, frame):
        print("\n检测到退出信号，正在准备关闭程序...")
        do_shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # 启动自动刷新线程，每隔60秒刷新一次日志文件
    flush_thread = AutoFlushThread(log_file, file_lock, shutdown_event, interval=60)
    flush_thread.start()
    print("AutoFlushThread 已启动.")

    # 启动输入监听线程，输入 's' 即可退出程序
    input_thread = InputListenerThread(shutdown_event, do_shutdown)
    input_thread.start()
    print("InputListenerThread 已启动.")

    # 启动串口读取线程（Linux下使用 '/dev/ttyUSB1'，波特率115200）
    serial_reader_thread = SerialReaderThread('/dev/ttyUSB1', 115200, shutdown_event, data_callback=log_received_data)
    # 如需在Windows测试可替换为：SerialReaderThread('COM3', 115200, shutdown_event, data_callback=log_received_data)
    serial_reader_thread.start()
    print("SerialReaderThread 已启动.")

    print("程序正在运行。输入 's' 并按回车以退出，或使用 Ctrl+C 强行退出。")
    try:
        while not shutdown_event.is_set():
            time.sleep(1)
    except KeyboardInterrupt:
        do_shutdown()

if __name__ == "__main__":
    main()
