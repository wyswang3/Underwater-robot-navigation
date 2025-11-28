# filters.py
# coding: UTF-8
import math
import numpy as np

def _finite(x):  # 小工具
    return (x is not None) and not (isinstance(x, float) and math.isnan(x))

class AngleUnwrapper:
    """实时角度解缠（弧度）"""
    def __init__(self):
        self.cumulative_offset = 0.0
        self.last_unwrapped = None

    def unwrap(self, angle_rad: float) -> float:
        if self.last_unwrapped is None:
            self.last_unwrapped = angle_rad
            return angle_rad
        delta = angle_rad - (self.last_unwrapped - self.cumulative_offset)
        if delta > math.pi:
            self.cumulative_offset -= 2.0 * math.pi
        elif delta < -math.pi:
            self.cumulative_offset += 2.0 * math.pi
        unwrapped = angle_rad + self.cumulative_offset
        self.last_unwrapped = unwrapped
        return unwrapped


class RealTimeIMUFilter:
    """
    轻量实时 IMU 滤波器（面向 50–200 Hz）
    - 校准：前 N 样本（默认 ≈ 1 s）求均值作为零偏
    - 滤波：一阶指数低通 (per-axis)；可叠两层近似二阶
    - 航向：用 Z 轴角速度(已滤)按 dt 积分（弧度），再解缠，最终输出角度(deg)

    参数
    ----
    fs:            期望采样频率（仅用于默认校准样本数与初始 dt）
    cutoff:        低通截止频率(Hz)，建议 3–8
    calibrate:     是否做启动校准
    calib_seconds: 校准时长（秒），最终会折算为样本数
    second_order:  True 时叠两层一阶，逼近二阶响应（开销仍很小）
    clamp_gyro_dps:对原始陀螺做幅值夹持（deg/s），0 关闭
    """
    def __init__(self,
                 fs: float = 100,
                 cutoff: float = 4.0,
                 calibrate: bool = True,
                 calib_seconds: float = 1.0,
                 second_order: bool = False,
                 clamp_gyro_dps: float = 0.0):

        if fs <= 0:
            raise ValueError("fs 必须 > 0")
        if cutoff <= 0 or cutoff >= 0.5 * fs:
            raise ValueError("cutoff 必须满足 0 < cutoff < 0.5*fs")

        self.fs = float(fs)
        self.cutoff = float(cutoff)
        self.second_order = bool(second_order)
        self.clamp_gyro_dps = float(clamp_gyro_dps)

        # 一阶低通时间常数 tau；alpha 会按 dt 自适应：alpha = dt/(tau+dt)
        self._tau = 1.0 / (2.0 * math.pi * self.cutoff)

        # 校准
        self._do_calib = bool(calibrate)
        self._calib_N = max(1, int(round(calib_seconds * fs)))
        self._calib_count = 0
        self._acc_sum = np.zeros(3, dtype=np.float64)
        self._gyr_sum_dps = np.zeros(3, dtype=np.float64)  # 存 deg/s 以减少单位换算

        # 零偏（acc 单位=原始；gyr_bias_dps 单位=deg/s）
        self.accel_bias = np.zeros(3, dtype=np.float64)
        self.gyro_bias_dps = np.zeros(3, dtype=np.float64)

        # 一阶低通状态（两层以实现“伪二阶”）
        self._acc_lp1 = np.zeros(3, dtype=np.float64)
        self._acc_lp2 = np.zeros(3, dtype=np.float64)
        self._gyr_lp1_dps = np.zeros(3, dtype=np.float64)
        self._gyr_lp2_dps = np.zeros(3, dtype=np.float64)
        self._lp_inited = False

        # 航向
        self._yaw_rad = 0.0
        self._unwrap = AngleUnwrapper()

        # 时间
        self._last_ts = None
        self._default_dt = 1.0 / self.fs

    def _lowpass_step(self, y_prev: np.ndarray, x: np.ndarray, dt: float) -> np.ndarray:
        """一阶低通： y += alpha*(x - y) ，alpha = dt/(tau+dt)"""
        alpha = dt / (self._tau + dt)
        # 矢量化就地运算
        y_prev += alpha * (x - y_prev)
        return y_prev

    def _maybe_clamp_gyro(self, gyr_dps: np.ndarray) -> None:
        if self.clamp_gyro_dps > 0.0:
            np.clip(gyr_dps, -self.clamp_gyro_dps, self.clamp_gyro_dps, out=gyr_dps)

    def process_sample(self, timestamp: float, accel_sample: np.ndarray, gyro_sample_dps: np.ndarray):
        """
        accel_sample: shape=(3,)  原单位（你现在是 1g 为单位，就保持一致）
        gyro_sample_dps: shape=(3,)  单位 deg/s（与你现有代码约定一致）
        返回：(filtered_accel(3,), filtered_gyro_dps(3,), yaw_deg) 或 None（校准期）
        """
        # --- 计算 dt（自适应）---
        if self._last_ts is None:
            dt = self._default_dt
        else:
            dt = float(timestamp - self._last_ts)
            # 防断样保护
            if not (0.2 * self._default_dt <= dt <= 5.0 * self._default_dt):
                dt = self._default_dt
        self._last_ts = float(timestamp)

        # --- 校准阶段 ---
        if self._do_calib:
            self._acc_sum += accel_sample
            self._gyr_sum_dps += gyro_sample_dps
            self._calib_count += 1
            if self._calib_count >= self._calib_N:
                self.accel_bias[:] = self._acc_sum / self._calib_count
                self.gyro_bias_dps[:] = self._gyr_sum_dps / self._calib_count
                self._do_calib = False
                # 初始化低通状态为“去偏后的当前值”，避免首帧突变
                acc0 = accel_sample - self.accel_bias
                gyr0_dps = gyro_sample_dps - self.gyro_bias_dps
                self._acc_lp1[:] = acc0
                self._gyr_lp1_dps[:] = gyr0_dps
                if self.second_order:
                    self._acc_lp2[:] = acc0
                    self._gyr_lp2_dps[:] = gyr0_dps
                self._lp_inited = True
                # print 一下也可
            return None  # 校准期间不输出

        # --- 去偏 & 可选夹持 ---
        acc_corr = accel_sample - self.accel_bias
        gyr_corr_dps = gyro_sample_dps - self.gyro_bias_dps
        self._maybe_clamp_gyro(gyr_corr_dps)

        # --- 低通（自适应 alpha）---
        if not self._lp_inited:
            self._acc_lp1[:] = acc_corr
            self._gyr_lp1_dps[:] = gyr_corr_dps
            if self.second_order:
                self._acc_lp2[:] = acc_corr
                self._gyr_lp2_dps[:] = gyr_corr_dps
            self._lp_inited = True

        self._lowpass_step(self._acc_lp1, acc_corr, dt)
        self._lowpass_step(self._gyr_lp1_dps, gyr_corr_dps, dt)
        if self.second_order:
            self._lowpass_step(self._acc_lp2, self._acc_lp1, dt)
            self._lowpass_step(self._gyr_lp2_dps, self._gyr_lp1_dps, dt)
            acc_f = self._acc_lp2
            gyr_f_dps = self._gyr_lp2_dps
        else:
            acc_f = self._acc_lp1
            gyr_f_dps = self._gyr_lp1_dps

        # --- 航向积分（用 Z 轴，单位弧度/秒）---
        gz_rad = math.radians(float(gyr_f_dps[2]))
        self._yaw_rad += gz_rad * dt
        yaw_unwrapped = self._unwrap.unwrap(self._yaw_rad)
        yaw_deg = math.degrees(yaw_unwrapped)

        # 返回“与原接口一致”的三个量
        return acc_f.copy(), gyr_f_dps.copy(), yaw_deg
