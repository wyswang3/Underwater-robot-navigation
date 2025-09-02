# tool.py、
# coding: UTF-8

import threading
import time

class AutoFlushThread(threading.Thread):
    """
    线程类，用于每分钟自动刷新日志文件。
    """
    def __init__(self, log_file, file_lock, shutdown_event, interval=60):
        """
        初始化 AutoFlushThread。

        :param log_file: 打开的日志文件对象。
        :param file_lock: 用于锁定文件的 threading.Lock 对象。
        :param shutdown_event: 用于通知线程停止的 threading.Event 对象。
        :param interval: 刷新间隔时间（秒），默认为60秒。
        """
        super().__init__(daemon=True)
        self.log_file = log_file
        self.file_lock = file_lock
        self.shutdown_event = shutdown_event
        self.interval = interval
    
    def run(self):
        """
        线程运行方法，每隔指定时间刷新日志文件。
        """
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
        """
        初始化 InputListenerThread。

        :param shutdown_event: 用于通知线程停止的 threading.Event 对象。
        :param shutdown_callback: 当检测到用户输入's'时调用的回调函数。
        """
        super().__init__(daemon=True)
        self.shutdown_event = shutdown_event
        self.shutdown_callback = shutdown_callback
    
    def run(self):
        """
        线程运行方法，监听用户输入。
        """
        while not self.shutdown_event.is_set():
            try:
                user_input = input()
                if user_input.lower() == 's':
                    print("检测到 's' 输入，正在准备关闭程序...")
                    self.shutdown_callback()
            except EOFError:
                # 处理 EOF（例如输入流关闭）
                break
            except Exception as e:
                print(f"输入监听错误: {e}")
                break
