import threading
import socket
import serial
import time
from queue import Queue
from drivers.dvl.hover_h1000.protocol import parse_line

class DVLData:
    """用于存储解析后的 DVL 数据"""
    def __init__(self, timestamp: float, east_vel: float, north_vel: float, up_vel: float, status: bool):
        self.timestamp = timestamp
        self.east_vel = east_vel
        self.north_vel = north_vel
        self.up_vel = up_vel
        self.status = status

    def __repr__(self):
        return f"DVLData(timestamp={self.timestamp}, east_vel={self.east_vel}, north_vel={self.north_vel}, up_vel={self.up_vel}, status={self.status})"

class DVLDevice:
    """与 DVL 设备通信并解析数据"""
    def __init__(self, device_type: str, port: str = "/dev/ttyUSB0", ip: str = "192.168.2.102", data_port: int = 10001, callback_method=None):
        self.device_type = device_type
        self.port = port
        self.ip = ip
        self.data_port = data_port
        self.callback_method = callback_method
        self.serial_conn = None
        self.data_socket = None
        self._stop_event = threading.Event()
        self.sample_q = Queue(maxsize=4000)

    def open(self):
        """打开串口或 TCP 连接"""
        if self.device_type == "ethernet":
            try:
                self.data_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.data_socket.connect((self.ip, self.data_port))
                print(f"Connected to {self.ip} on port {self.data_port}")
            except Exception as e:
                print(f"Failed to connect to DVL device via TCP: {e}")
                return False
        elif self.device_type == "serial":
            try:
                self.serial_conn = serial.Serial(self.port, 115200, timeout=1)
                print(f"Connected to DVL device via serial {self.port}")
            except serial.SerialException as e:
                print(f"Failed to connect to DVL device via serial: {e}")
                return False
        return True

    def close(self):
        """关闭设备连接"""
        if self.device_type == "ethernet" and self.data_socket:
            self.data_socket.close()
        elif self.device_type == "serial" and self.serial_conn:
            self.serial_conn.close()
        print("Connection closed.")

    def read_line(self):
        """从串口或 TCP 连接读取一行数据"""
        try:
            if self.device_type == "ethernet":
                line = self.data_socket.recv(1024).decode("ascii").strip()
            else:
                line = self.serial_conn.readline().decode("ascii").strip()
            return line
        except Exception as e:
            print(f"Error reading data: {e}")
            return None

    def listen(self):
        """持续监听并解析 DVL 数据"""
        while not self._stop_event.is_set():
            line = self.read_line()
            if line:
                parsed_data = parse_line(line)  # 调用协议解析函数
                if parsed_data and self.callback_method:
                    self.callback_method(parsed_data)  # 将数据传递给回调函数
            time.sleep(0.01)  # 防止占用过多 CPU

    def start_listening(self):
        """启动数据监听线程"""
        if self.open():
            self._stop_event.clear()
            listen_thread = threading.Thread(target=self.listen, daemon=True)
            listen_thread.start()
            print("Data listening thread started.")
        else:
            print("Failed to start listening thread.")

    def stop_listening(self):
        """停止数据监听线程"""
        self._stop_event.set()
        print("Data listening thread stopped.")
        self.close()

    def add_to_queue(self, parsed_data):
        """将解析后的数据放入队列"""
        try:
            dvl_data = DVLData(
                timestamp=time.time(),
                east_vel=parsed_data["ve"],
                north_vel=parsed_data["vn"],
                up_vel=parsed_data["vu"],
                status=parsed_data["valid"]
            )
            self.sample_q.put_nowait(dvl_data)
        except Exception as e:
            print(f"Error adding data to queue: {e}")
