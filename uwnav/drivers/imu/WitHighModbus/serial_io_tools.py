# serial_io_tools.py
import threading
import time
import os
import serial

class AutoFlushThread(threading.Thread):
    """每 interval 秒刷新日志文件；可选落盘 fsync。"""
    def __init__(self, log_file, file_lock, shutdown_event, interval=60, verbose=True, fsync=False):
        super().__init__(daemon=True)
        self.log_file = log_file
        self.file_lock = file_lock
        self.shutdown_event = shutdown_event
        self.interval = interval
        self.verbose = verbose
        self.fsync = fsync

    def run(self):
        while not self.shutdown_event.wait(timeout=self.interval):
            try:
                with self.file_lock:
                    self.log_file.flush()
                    if self.fsync and hasattr(self.log_file, "fileno"):
                        os.fsync(self.log_file.fileno())
                if self.verbose:
                    print("【LOG】周期性刷新完成")
            except Exception as e:
                print(f"【LOG】自动保存失败: {e}")

class InputListenerThread(threading.Thread):
    """监听用户输入；输入 key 时调用回调并触发关停。"""
    def __init__(self, shutdown_event, shutdown_callback, key='s', prompt="输入 's' 回车以退出"):
        super().__init__(daemon=True)
        self.shutdown_event = shutdown_event
        self.shutdown_callback = shutdown_callback
        self.key = key.lower()
        self.prompt = prompt

    def run(self):
        print(self.prompt)
        while not self.shutdown_event.is_set():
            try:
                user_input = input()
                if user_input.lower() == self.key:
                    print("检测到退出指令，正在关闭…")
                    # 回调应当幂等：多次调用也没副作用
                    try:
                        self.shutdown_callback()
                    finally:
                        self.shutdown_event.set()
                        break
            except EOFError:
                break
            except Exception as e:
                print(f"【INPUT】错误: {e}")
                break

class SerialReaderThread(threading.Thread):
    """
    串口读取线程：
    - line_mode=True：按行读取（适合带换行的调试输出）
    - line_mode=False：原始读取固定字节数（适合 Modbus/二进制协议）
    """
    def __init__(self, port, baudrate, shutdown_event, data_callback=None,
                 line_mode=True, eol=b'\n', read_size=64, encoding='utf-8'):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.shutdown_event = shutdown_event
        self.data_callback = data_callback
        self.line_mode = line_mode
        self.eol = eol
        self.read_size = read_size
        self.encoding = encoding
        self.serial_conn = None

    def run(self):
        try:
            self.serial_conn = serial.Serial(self.port, self.baudrate, timeout=1)
        except serial.SerialException as e:
            print(f"【SERIAL】打开 {self.port} 失败: {e}")
            return

        try:
            while not self.shutdown_event.is_set():
                try:
                    if self.serial_conn.in_waiting == 0:
                        time.sleep(0.05)
                        continue

                    if self.line_mode:
                        received = self.serial_conn.readline()
                    else:
                        received = self.serial_conn.read(self.read_size)

                    if not received:
                        continue

                    # 文本优先，失败回退为十六进制
                    try:
                        payload = received.decode(self.encoding, errors='strict').strip()
                    except UnicodeDecodeError:
                        payload = received.hex()

                    if payload and self.data_callback:
                        self.data_callback(payload)

                except serial.SerialException as e:
                    print(f"【SERIAL】{self.port} 读取错误: {e}")
                    break
                except Exception as ex:
                    print(f"【SERIAL】{self.port} 未知错误: {ex}")
                    break
        finally:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
                print(f"【SERIAL】已关闭 {self.port}")
