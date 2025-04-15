import numpy as np

class utils:
    def quaternion_to_euler_array(quat):
        """
        将四元数转换为欧拉角（Roll, Pitch, Yaw）
        参数：
        - quat：四元数，格式为 [w, x, y, z]
        返回值：欧拉角数组，格式为 [Roll, Pitch, Yaw]，单位为弧度
        """
        w, x, y, z = quat
        # woll, pitch, yaw

      # Pitch (y-axis rotation)
        t0 = +2.0 * (w * y - z * x)
        t1 = +1.0 - 2.0 * (y * y + z * z)
        roll_x = np.arctan2(t0, t1)

        # Roll (x-axis rotation)
        t2 = +2.0 * (w * x + y * z)
        t2 = np.clip(t2, -1.0, 1.0)
        pitch_y = np.arcsin(t2)

        # Yaw (z-axis rotation)
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (x * x + y * y)
        yaw_z = np.arctan2(t3, t4)

        # print change into degree mode:
        # 将弧度制的roll_x, pitch_y, yaw_z转换为角度制(仅仅用于debug)
        roll_x_deg = roll_x * 180 / 3.14
        pitch_y_deg = pitch_y * 180 / 3.14
        yaw_z_deg = yaw_z * 180 / 3.14
        print('roll_x, pitch_y, yaw_z is:(deg)', roll_x_deg, pitch_y_deg, yaw_z_deg)

        # Returns roll, pitch, yaw in a NumPy array in radians
        return np.array([roll_x, pitch_y, yaw_z])
    

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

    
    def pd_control(target_q, q, kp, target_dq, dq, kd):
        '''Calculates torques from position commands
        '''
        return (target_q - q) * kp + (target_dq - dq) * kd
