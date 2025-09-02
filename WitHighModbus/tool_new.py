# tool_new.py
import threading
import time
import serial

class AutoFlushThread(threading.Thread):
    """
    线程类，用于每分钟自动刷新日志文件。
    """
    def __init__(self, log_file, file_lock, shutdown_event, interval=60):
        super().__init__(daemon=True)
        self.log_file = log_file
        self.file_lock = file_lock
        self.shutdown_event = shutdown_event
        self.interval = interval
    
    def run(self):
        while not self.shutdown_event.is_set():
            is_set = self.shutdown_event.wait(timeout=self.interval)
            if is_set:
                break
            try:
                with self.file_lock:
                    self.log_file.flush()
                    print("日志文件已每分钟自动保存一次。")
            except Exception as e:
                print(f"自动保存日志文件失败: {e}")

class InputListenerThread(threading.Thread):
    """
    线程类，用于监听用户输入，当用户输入's'时触发关闭程序。
    """
    def __init__(self, shutdown_event, shutdown_callback):
        super().__init__(daemon=True)
        self.shutdown_event = shutdown_event
        self.shutdown_callback = shutdown_callback
    
    def run(self):
        while not self.shutdown_event.is_set():
            try:
                user_input = input()
                if user_input.lower() == 's':
                    print("检测到 's' 输入，正在准备关闭程序...")
                    self.shutdown_callback()
            except EOFError:
                break
            except Exception as e:
                print(f"输入监听错误: {e}")
                break


class SerialReaderThread(threading.Thread):
    """
    线程类，用于读取指定串口的数据并进行处理。
    """

    def __init__(self, port, baudrate, shutdown_event, data_callback=None):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.shutdown_event = shutdown_event
        self.data_callback = data_callback
        self.serial_conn = None

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"无法打开串口 {self.port}: {e}")
            return

        while not self.shutdown_event.is_set():
            try:
                if self.serial_conn.in_waiting > 0:
                    received_bytes = self.serial_conn.readline()
                    try:
                        received_str = received_bytes.decode('utf-8').strip()
                    except UnicodeDecodeError:
                        received_str = received_bytes.hex()
                    if received_str:
                        if self.data_callback:
                            self.data_callback(received_str)
                else:
                    time.sleep(0.1)
            except serial.SerialException as e:
                print(f"{self.port} 串口读取错误: {e}")
                break
            except Exception as ex:
                print(f"{self.port} 读取过程中发生未知错误: {ex}")
                break

        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            # 仅在关闭时记录错误或提示信息（此处可选择不输出）
            # print(f"已关闭串口 {self.port}")


