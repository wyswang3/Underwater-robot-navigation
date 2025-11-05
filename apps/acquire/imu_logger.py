import csv
import time

class IMULogger:
    def __init__(self, out_dir):
        """
        初始化IMULogger，创建目录和文件，并准备写入器
        :param out_dir: 输出文件目录
        """
        self.out_dir = out_dir
        self.raw_file, self.raw_writer, self.fil_file, self.fil_writer = self.make_writers()

    def make_writers(self):
        """
        创建写入器，准备原始数据文件和滤波后数据文件
        :return: 返回文件对象和写入器
        """
        date_str = time.strftime("%Y%m%d")
        raw_path = f"{self.out_dir}/imu_raw_data_{date_str}.csv"
        fil_path = f"{self.out_dir}/imu_filtered_data_{date_str}.csv"

        # 打开文件
        raw_f = open(raw_path, "a", newline="", encoding="utf-8")
        fil_f = open(fil_path, "a", newline="", encoding="utf-8")

        raw_w, fil_w = csv.writer(raw_f), csv.writer(fil_f)

        # 写入表头
        if raw_f.tell() == 0:
            raw_w.writerow(["Timestamp", "AccX", "AccY", "AccZ", "AsX", "AsY", "AsZ", "HX", "HY", "HZ", "AngX", "AngY", "AngZ"])

        if fil_f.tell() == 0:
            fil_w.writerow(["Timestamp", "AccX_filt", "AccY_filt", "AccZ_filt", "AsX_filt", "AsY_filt", "AsZ_filt", "Yaw_filt(deg)"])

        return raw_f, raw_w, fil_f, fil_w

    def store_data(self, imu_data):
        """
        保存原始数据
        :param imu_data: IMUData 实例，包含原始数据
        """
        if imu_data:
            self.raw_writer.writerow([imu_data.timestamp, *imu_data.acc, *imu_data.gyr])

    def save_filtered_data(self, imu_data):
        """
        保存滤波后的数据
        :param imu_data: IMUData 实例，包含滤波后的数据
        """
        if imu_data.filtered_acc is not None:
            self.fil_writer.writerow([imu_data.timestamp, *imu_data.filtered_acc, *imu_data.filtered_gyr, imu_data.yaw])

    def close(self):
        """关闭文件"""
        self.raw_file.close()
        self.fil_file.close()
