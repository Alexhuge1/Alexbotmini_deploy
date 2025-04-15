import time
import serial
import numpy as np
from.parsers.hipnuc_serial_parser import hipnuc_parser
# sudo chmod a+rw /dev/ttyUSB0
# yaw pitch roll 
class IMU:
    def __init__(self):
        self.quat_data = None
        self.gvec_data = None

    def cmd_read(self, port, baudrate):
        serial_parser = hipnuc_parser()
        latest_hipnuc_frame = None
        read_count = 0  # 记录已读取的次数
        max_read_count = 1  # 设置最大读取次数

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

                            # 提取并转换陀螺仪数据
                            gvec_data_0 = np.array(latest_hipnuc_frame.gyr[0]).astype(np.double) * (np.pi / 180)
                            gvec_data_1 = np.array(latest_hipnuc_frame.gyr[1]).astype(np.double) * (np.pi / 180)
                            gvec_data_2 = np.array(latest_hipnuc_frame.gyr[2]).astype(np.double) * (np.pi / 180)
                            self.gvec_data = np.array([gvec_data_0, gvec_data_1, gvec_data_2])
                        read_count += 1
                time.sleep(0.01)
                

        return self.quat_data, self.gvec_data
