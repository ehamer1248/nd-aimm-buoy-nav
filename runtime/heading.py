import time
from math import atan2, sqrt, pi
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR, BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

class BNO085Heading:
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(BNO085Heading, cls).__new__(cls)
            # Initialize I2C and BNO085 only once
            cls._instance.i2c = busio.I2C(board.SCL, board.SDA, frequency=800000)
            cls._instance.bno = BNO08X_I2C(cls._instance.i2c)
            cls._instance.bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
            cls._instance.bno.enable_feature(BNO_REPORT_GEOMAGNETIC_ROTATION_VECTOR)
        return cls._instance

    def find_heading(dqw, dqx, dqy, dqz):
        norm = sqrt(dqw**2 + dqx**2 + dqy**2 + dqz**2)
        dqw, dqx, dqy, dqz = dqw/norm, dqx/norm, dqy/norm, dqz/norm
        ysqr = dqy**2
        t3 = +2.0 * (dqw * dqz + dqx * dqy)
        t4 = +1.0 - 2.0 * (ysqr + dqz**2)
        yaw_raw = atan2(t3, t4)
        yaw = yaw_raw * 180.0 / pi
        return 360 - yaw if yaw > 0 else abs(yaw)

    def read_heading(self):
        quat_i, quat_j, quat_k, quat_real = self.bno.quaternion
        return self.find_heading(quat_real, quat_i, quat_j, quat_k)

