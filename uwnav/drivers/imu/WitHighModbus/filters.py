# filters.py
# coding: UTF-8

import numpy as np
from scipy.signal import butter, lfilter, lfilter_zi
class AngleUnwrapper:
    """实时角度解缠类，用于保持航向角连续性"""
    def __init__(self):
        self.cumulative_offset = 0.0
        self.last_unwrapped = None

    def unwrap(self, angle: float) -> float:
        if self.last_unwrapped is None:
            self.last_unwrapped = angle
            return angle
        delta = angle - (self.last_unwrapped - self.cumulative_offset)
        if delta > np.pi:
            self.cumulative_offset -= 2 * np.pi
        elif delta < -np.pi:
            self.cumulative_offset += 2 * np.pi
        unwrapped = angle + self.cumulative_offset
        self.last_unwrapped = unwrapped
        return unwrapped

class RealTimeIMUFilter:
    """
    实时IMU滤波器：
      1. 静置校准阶段：在启动后1秒内累计六轴数据（加速度3轴和角速度3轴），
         计算各轴平均值作为零偏（角速度原单位为 deg/s，校准后转换为 rad/s）。
      2. 校准完成后，对每个新采样样本：
         - 扣除加速度和角速度各轴零偏；
         - 对扣除零偏后的数据进行2阶 Butterworth低通滤波（加速度单位保持不变，
           角速度单位为 deg/s）；
         - 对角速度Z轴，先转换为 rad/s，再扣除校准零偏，
           积分计算航向角，利用 AngleUnwrapper 解缠，
           最后转换为角度制输出（deg）。
      返回：
         filtered_accel: 加速度滤波结果（单位与原始数据一致）
         filtered_gyro: 角速度滤波结果（单位 deg/s，已扣除零偏）
         yaw_deg: 计算得到的航向角（单位 deg）
    """
    def __init__(self, fs: float = 50, cutoff: float = 4.0, calibrate: bool = True):
        if fs <= 0:
            raise ValueError("采样频率 fs 必须大于 0.")
        self.fs = fs
        self.cutoff = cutoff
        self.calibrate = calibrate

        # 计算归一化截止频率，必须在 (0, 1) 之间
        norm_cutoff = cutoff / (0.5 * fs)
        if norm_cutoff <= 0 or norm_cutoff >= 1.0:
            raise ValueError("截止频率 cutoff 必须满足 0 < cutoff < 0.5*fs.")

        # 初始零偏均设为0（后续静置校准阶段更新）
        self.accel_bias = np.zeros(3)   # 加速度零偏
        self.gyro_bias = np.zeros(3)    # 角速度零偏（校准后转换为 rad/s）
        self.yaw_bias = 0.0             # 航向角零偏（取 gyro_bias[2]）

        # 设计2阶Butterworth低通滤波器
        # 对加速度数据：输入单位与原始数据一致
        # 对角速度数据：输入单位为 deg/s
        self.accel_b, self.accel_a = butter(2, norm_cutoff, btype='low')
        self.gyro_b, self.gyro_a = butter(2, norm_cutoff, btype='low')

        # 初始化滤波状态，为每个轴保持滤波器状态
        self.accel_states = [None] * 3
        self.gyro_states = [None] * 3

        # 用于航向角积分与解缠
        self.yaw_integrated = 0.0
        self.unwrapper = AngleUnwrapper()
        self.last_timestamp = None

        # 校准阶段变量：累计前1秒静置数据
        self.calib_start_time = None
        self.accel_calib_accum = np.zeros(3)
        self.gyro_calib_accum = np.zeros(3)
        self.calib_count = 0

    def _apply_filter(self, sample, axis, filter_type='accel'):
        if filter_type == 'accel':
            b, a = self.accel_b, self.accel_a
            states = self.accel_states
        else:
            b, a = self.gyro_b, self.gyro_a
            states = self.gyro_states
        if states[axis] is None:
            zi = lfilter_zi(b, a) * sample
            filtered, zo = lfilter(b, a, [sample], zi=zi)
            states[axis] = zo
        else:
            filtered, zo = lfilter(b, a, [sample], zi=states[axis])
            states[axis] = zo
        return filtered[0]

    def process_sample(self, timestamp, accel_sample, gyro_sample):
        """
        处理单个采样样本：
          - accel_sample: 长度为3的原始加速度数组
          - gyro_sample: 长度为3的原始角速度数组，单位为 deg/s
        若处于校准阶段（前1秒静置），累计数据并返回 None；
        校准结束后，分别对六轴数据扣除零偏，再进行低通滤波；
        对角速度Z轴，转换为 rad/s后扣除零偏后积分计算航向角，
        最后将航向角转换为角度制输出。
        返回：(filtered_accel, filtered_gyro, yaw_deg)
        """
        # 静置校准阶段
        if self.calibrate:
            if self.calib_start_time is None:
                self.calib_start_time = timestamp
            if timestamp - self.calib_start_time < 1.0:
                self.accel_calib_accum += accel_sample
                self.gyro_calib_accum += gyro_sample
                self.calib_count += 1
                return None
            else:
                # 计算各轴平均零偏
                self.accel_bias = self.accel_calib_accum / self.calib_count
                gyro_bias_deg = self.gyro_calib_accum / self.calib_count  # 单位: deg/s
                self.gyro_bias = np.deg2rad(gyro_bias_deg)  # 转换为 rad/s
                # 对航向角，取Z轴的零偏
                self.yaw_bias = self.gyro_bias[2]
                self.calibrate = False
                print(f"校准完成：accel_bias = {self.accel_bias}, gyro_bias (rad/s) = {self.gyro_bias}")

        # 对后续数据，先扣除零偏
        corrected_accel = accel_sample - self.accel_bias
        # 对角速度：扣除零偏（先将原始值减去对应轴的零偏，注意 gyro_bias 目前单位为 rad/s，
        # 而原始数据 gyro_sample 单位为 deg/s，所以先将 gyro_bias 转换为 deg/s进行扣除）
        gyro_bias_deg = np.rad2deg(self.gyro_bias)
        corrected_gyro = gyro_sample - gyro_bias_deg

        # 对每个轴进行低通滤波
        filtered_accel = np.array([
            self._apply_filter(corrected_accel[i], i, filter_type='accel')
            for i in range(3)
        ])
        filtered_gyro = np.array([
            self._apply_filter(corrected_gyro[i], i, filter_type='gyro')
            for i in range(3)
        ])

        # 对角速度Z轴：先转换为 rad/s，再扣除航向角零偏（已在校准中扣除），
        # 此处仅转换滤波后的结果，确保积分计算使用 rad/s
        filtered_gyro_z_rad = np.deg2rad(filtered_gyro[2])

        # 计算采样间隔 dt
        if self.last_timestamp is None:
            dt = 1.0 / self.fs
        else:
            dt = timestamp - self.last_timestamp
        self.last_timestamp = timestamp

        # 积分计算航向角（单位 rad）
        self.yaw_integrated += filtered_gyro_z_rad * dt
        yaw_unwrapped_rad = self.unwrapper.unwrap(self.yaw_integrated)
        # 转换为角度制
        yaw_deg = np.rad2deg(yaw_unwrapped_rad)

        return filtered_accel, filtered_gyro, yaw_deg
