import time
import numpy as np
from collections import deque
import threading
from datetime import datetime
from queue import Queue, Empty
from drivers.imu.WitHighModbus import device_model
from drivers.imu.WitHighModbus.filters import RealTimeIMUFilter

# ---- 调试开关 ----
DEBUG_PRINT_EVERY = 25   # 每 N 条样本打印一次概览；0 关闭
WARN_IF_IDLE_SEC  = 3.0  # N 秒没收到样本则报警
FLUSH_PERIOD_SEC  = 0.5  # 定期刷盘周期
# -------------------

class IMUData:
    """封装IMU数据，存储原始数据和滤波后的数据"""
    def __init__(self):
        self.timestamp = 0.0
        self.acc = np.zeros(3)   # 加速度数据
        self.gyr = np.zeros(3)   # 角速度数据
        self.filtered_acc = np.zeros(3)  # 滤波后的加速度
        self.filtered_gyr = np.zeros(3)  # 滤波后的角速度
        self.yaw = 0.0           # 滤波后的偏航角

    def update(self, timestamp, acc, gyr):
        """更新数据"""
        self.timestamp = timestamp
        self.acc = acc
        self.gyr = gyr

    def filter_data(self, filter_instance):
        """使用滤波器处理数据"""
        filtered_data = filter_instance.process_sample(self.timestamp, self.acc, self.gyr)
        if filtered_data:
            self.filtered_acc, self.filtered_gyr, self.yaw = filtered_data

class IMUDevice:
    def __init__(self, port, baud, address, callback_method=None):
        self.port = port
        self.baud = baud
        self.address = address
        self.device = None
        self.filter = RealTimeIMUFilter(fs=50, cutoff=4.0, calibrate=True)
        self.sample_q = Queue(maxsize=4000)
        self.shutdown_event = threading.Event()
        self.imu_data = IMUData()  # 实例化 IMUData 类
        self.callback_method = callback_method  # 回调方法，用于实时传递数据

    def open_device(self):
        """打开IMU设备连接"""
        try:
            self.device = device_model.DeviceModel(
                deviceName="IMUDevice",
                portName=self.port,
                baud=self.baud,
                ADDR=self.address,
                callback_method=self.update_data
            )
            self.device.openDevice()
            self.device.startLoopRead()
            print(f"IMU device connected at {self.port} with baud {self.baud}")
        except Exception as e:
            print(f"Error opening device: {e}")

    def close_device(self):
        """关闭设备连接"""
        if self.device:
            self.device.stopLoopRead()
            self.device.closeDevice()

    def update_data(self, data):
        """数据采集回调：采集并将数据放入结构体中"""
        if self.shutdown_event.is_set():
            return
        ts_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")
        t = time.time()
        try:
            acc = np.array([data.get("AccX"), data.get("AccY"), data.get("AccZ")], dtype=float)
            gyr = np.array([data.get("AsX"), data.get("AsY"), data.get("AsZ")], dtype=float)
            self.imu_data.update(t, acc, gyr)  # 更新IMU数据结构
            self.sample_q.put_nowait(self.imu_data)

            # 如果回调函数存在，调用回调传递数据给上层
            if self.callback_method:
                self.callback_method(self.imu_data)

        except Exception as e:
            print(f"Data update error: {e}")

    def monitor_data(self):
        """监控数据，检测是否有长时间没有收到新数据"""
        now = time.time()
        if self._stats["last_recv_t"] and (now - self._stats["last_recv_t"] > WARN_IF_IDLE_SEC):
            print(f"[WARN] {self._stats['recv']} 样本后 {now - self._stats['last_recv_t']:.1f}s 未收到新数据，检查端口/波特率/设备模式")

    def process_data(self):
        """数据处理：滤波并传递给回调"""
        while not self.shutdown_event.is_set():
            try:
                item = self.sample_q.get(timeout=0.1)
                # 执行滤波
                self.imu_data.filter_data(self.filter)

                # 通过回调传递数据给上层系统
                if self.callback_method:
                    self.callback_method(self.imu_data)

            except Empty:
                continue

    def start(self):
        """启动数据采集和处理"""
        data_thread = threading.Thread(target=self.process_data)
        data_thread.daemon = True
        data_thread.start()
        return data_thread
