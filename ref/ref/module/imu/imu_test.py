import time
import serial
import numpy as np
from parsers.hipnuc_serial_parser import hipnuc_parser

# sudo chmod a+rw /dev/ttyUSB0
# yaw pitch roll 

class IMU:
    def __init__(self):
        self.quat_data = None
        self.gvec_data = None

    def cmd_read(self, port, baudrate):
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None
        read_count = 0  
        max_read_count = 1  

        with serial.Serial(port, int(baudrate), timeout=1) as ser:
            while read_count < max_read_count:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    hipnuc_frames = serial_parser.parse(data)
                    if hipnuc_frames:
                        latest_hipnuc_frame = hipnuc_frames[-1]
                        if latest_hipnuc_frame.frame_type is not None:
                            # 提取并转换四元数数据
                            quat_data_0 = np.array(latest_hipnuc_frame.quat[0]).astype(np.double)
                            quat_data_1 = np.array(latest_hipnuc_frame.quat[1]).astype(np.double)
                            quat_data_2 = np.array(latest_hipnuc_frame.quat[2]).astype(np.double)
                            quat_data_3 = np.array(latest_hipnuc_frame.quat[3]).astype(np.double)
                            self.quat_data = np.array([quat_data_0, quat_data_1, quat_data_2, quat_data_3])

                            # 打印四元数
                            print("Quaternion (w, x, y, z):", self.quat_data)

                            # 提取并转换陀螺仪数据
                            gvec_data_0 = np.array(latest_hipnuc_frame.gyr[0]).astype(np.double) * (np.pi / 180)
                            gvec_data_1 = np.array(latest_hipnuc_frame.gyr[1]).astype(np.double) * (np.pi / 180)
                            gvec_data_2 = np.array(latest_hipnuc_frame.gyr[2]).astype(np.double) * (np.pi / 180)
                            self.gvec_data = np.array([gvec_data_0, gvec_data_1, gvec_data_2])

                            # 四元数转欧拉角并转换为角度值后打印
                            euler_angles = self.quaternion_to_euler(self.quat_data)
                            roll_deg = np.rad2deg(euler_angles[0])
                            pitch_deg = np.rad2deg(euler_angles[1])
                            yaw_deg = np.rad2deg(euler_angles[2])
                            print("Euler Angles (Roll, Pitch, Yaw):", roll_deg, pitch_deg, yaw_deg)
                        read_count += 1
                time.sleep(0.01)

    def quaternion_to_euler(self, quat):
        """
        将四元数转换为欧拉角（Roll, Pitch, Yaw）
        参数：
        - quat：四元数，格式为 [w, x, y, z]
        返回值：欧拉角数组，格式为 [Roll, Pitch, Yaw]，单位为弧度
        """
        w, x, y, z = quat

        # Pitch (y-axis rotation)
        t0 = +2.0 * (w * y - z * x)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        roll = np.arctan2(t0, t1)

        # Roll (x-axis rotation)
        t2 = +2.0 * (w * x + y * z)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch = np.arcsin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (x * x + y * y)
        yaw = np.arctan2(t3, t4)

        return np.array([roll, pitch, yaw])


if __name__ == "__main__":
    port = "/dev/ttyUSB0"
    baudrate = 115200
    imu = IMU()
    imu.cmd_read(port, baudrate)
