import threading
import serial
import time
import csv
import logging
from uwnav.drivers.dvl.hover_h1000.protocol import parse_line

# ---- 调试开关 ----
DEBUG_PRINT_EVERY = 5   # 每 N 条样本打印一次概览；0 关闭
WARN_IF_IDLE_SEC  = 3.0  # N 秒没收到样本则报警
FLUSH_PERIOD_SEC  = 0.5  # 定期刷盘周期
# -------------------

class DVLData:
    """封装DVL数据，存储原始数据"""
    def __init__(self):
        self.timestamp = 0.0
        self.velocity = [0.0, 0.0, 0.0]  # 速度数据（单位：mm/s）
        self.depth = 0.0  # 深度数据（单位：米）

    def update(self, timestamp, velocity, depth):
        """更新DVL数据"""
        self.timestamp = timestamp
        self.velocity = velocity
        self.depth = depth

class DVLSerialInterface:
    """
    串口通信接口类，用于与 DVL 设备建立连接，接收数据并传递给协议解析函数。
    """
    
    def __init__(self, port: str, baud: int = 115200, callback_method=None):
        """
        初始化串口通信类。
        
        :param port: 串口设备路径（如 /dev/ttyUSB0）
        :param baud: 串口波特率（默认为115200）
        :param callback_method: 数据解析后的回调方法，默认不传入
        """
        self.port = port
        self.baud = baud
        self.callback_method = callback_method
        self.serial_conn = None
        self._stop_event = threading.Event()  # 停止线程的事件标志

    def open(self):
        """
        打开串口连接。
        """
        try:
            self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
            logging.info(f"串口 {self.port} 连接成功，波特率 {self.baud}")
        except serial.SerialException as e:
            logging.error(f"无法打开串口 {self.port}: {e}")
            return False
        return True

    def close(self):
        """
        关闭串口连接。
        """
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            logging.info(f"串口 {self.port} 关闭成功")
        
    def read_line(self):
        """
        从串口读取一行数据，行末为 \r\n。
        
        :return: 返回一行字符串（去除结尾的 \r\n）
        """
        try:
            line = self.serial_conn.readline().decode("ascii").strip()
            return line
        except serial.SerialException as e:
            logging.error(f"串口读取错误: {e}")
            return None

    def listen(self):
        """
        串口数据监听线程，持续读取串口数据并调用协议解析函数。
        """
        while not self._stop_event.is_set():
            line = self.read_line()  # 读取一行数据
            if line:
                parsed_data = parse_line(line)  # 调用协议解析函数
                if parsed_data and self.callback_method:
                    self.callback_method(parsed_data)  # 回调处理解析后的数据
            time.sleep(0.01)  # 控制读取速度，避免过度占用 CPU

    def start_listening(self):
        """
        启动数据监听线程，开始从串口读取数据。
        """
        if self.open():
            self._stop_event.clear()  # 重置停止事件标志
            self.listener_thread = threading.Thread(target=self.listen, daemon=True)
            self.listener_thread.start()
            logging.info("数据监听线程启动成功")
        else:
            logging.error("无法启动数据监听线程，串口未打开")

    def stop_listening(self):
        """
        停止数据监听线程，关闭串口连接。
        """
        self._stop_event.set()
        self.listener_thread.join()
        self.close()
        logging.info("数据监听线程已停止")

class DVLLogger:
    def __init__(self, out_dir):
        """
        初始化DVLLogger，创建目录和文件，并准备写入器
        :param out_dir: 输出文件目录
        """
        self.out_dir = out_dir
        self.raw_file, self.raw_writer, self.fil_file, self.fil_writer = self.make_writers()

    def make_writers(self):
        """
        创建写入器，准备原始数据文件和调整后数据文件
        :return: 返回文件对象和写入器
        """
        date_str = time.strftime("%Y%m%d")
        raw_path = f"{self.out_dir}/dvl_raw_data_{date_str}.csv"
        fil_path = f"{self.out_dir}/dvl_adjusted_data_{date_str}.csv"

        # 打开文件
        raw_f = open(raw_path, "a", newline="", encoding="utf-8")
        fil_f = open(fil_path, "a", newline="", encoding="utf-8")

        raw_w, fil_w = csv.writer(raw_f), csv.writer(fil_f)

        # 写入表头
        if raw_f.tell() == 0:
            raw_w.writerow(["Timestamp", "VelocityX", "VelocityY", "VelocityZ", "Depth"])

        if fil_f.tell() == 0:
            fil_w.writerow(["Timestamp", "Adjusted_VelocityX", "Adjusted_VelocityY", "Adjusted_VelocityZ", "Adjusted_Depth"])

        return raw_f, raw_w, fil_f, fil_w

    def store_data(self, dvl_data):
        """
        保存原始DVL数据
        :param dvl_data: DVLData 实例，包含原始数据
        """
        if dvl_data:
            self.raw_writer.writerow([dvl_data.timestamp, *dvl_data.velocity, dvl_data.depth])

    def save_adjusted_data(self, dvl_data):
        """
        保存调整后的DVL数据
        :param dvl_data: DVLData 实例，包含调整后的数据
        """
        adjusted_velocity = [v / 1000 for v in dvl_data.velocity]  # 将速度从 mm/s 转为 m/s
        adjusted_depth = dvl_data.depth  # 深度单位已经是米
        self.fil_writer.writerow([dvl_data.timestamp, *adjusted_velocity, adjusted_depth])

    def close(self):
        """关闭文件"""
        self.raw_file.close()
        self.fil_file.close()

# 主程序
def main():
    out_dir = './output_data'  # 设置输出文件目录
    dvl_logger = DVLLogger(out_dir)

    # 初始化 DVL 串口通信接口
    dvl_device = DVLSerialInterface(port='/dev/ttyUSB1', baud=115200, callback_method=dvl_logger.store_data)
    dvl_device.start_listening()

    try:
        while True:
            time.sleep(1)  # 主线程保持运行
    except KeyboardInterrupt:
        print("程序终止")
    finally:
        dvl_device.stop_listening()

if __name__ == "__main__":
    main()
