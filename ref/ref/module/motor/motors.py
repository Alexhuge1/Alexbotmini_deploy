import numpy as np
from fi_fsa import fi_fsa_v2
import time
from scipy.interpolate import CubicSpline


class MOTOR:
    def __init__(self):
        self.server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                               '192.168.137.104', '192.168.137.105', '192.168.137.106',
                               '192.168.137.107', '192.168.137.108', '192.168.137.109',
                               '192.168.137.110', '192.168.137.111', '192.168.137.112']
        self.motors_num = len(self.server_ip_list)
        self.q = []
        self.dq = []
        self.current_positions = []

    def get_motors_ip(self):
        """
        获取电机对应的服务器IP地址列表，并进行相关验证
        """
        server_ip_list_test = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
        if not isinstance(server_ip_list_test, list):
            raise TypeError("Can not find motor, please check the wire or reboot motor.")
        if len(server_ip_list_test)!= self.motors_num:
            print('Lost connection of motors')
            print("Server IP List:", server_ip_list_test)
            print("Motors Num:", len(server_ip_list_test))
            raise ValueError('Lost connection of motors. The number of motors is not correct.')
        return server_ip_list_test

    def get_pvc(self):
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        """
        self.q = []
        self.dq = []
        for ip in self.server_ip_list:
            position, velocity, current = fi_fsa_v2.fast_get_pvc(ip)
            pos = np.array(position).astype(np.double) * (np.pi / 180)
            vel = np.array(velocity).astype(np.double) * (np.pi / 180)
            self.q.append(pos)
            self.dq.append(vel)
        self.current_positions = np.array([pos.item() if np.ndim(pos) == 0 else pos[0] for pos in self.q])
        return self.q, self.dq

    def set_position_mode(self):
        """
        设置所有电机为位置控制模式
        """
        for ip in self.server_ip_list:
            fi_fsa_v2.fast_set_enable(ip)
            fi_fsa_v2.fast_set_mode_of_operation(ip, fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL)

    def set_current_mode(self):
        """
        设置所有电机为电流控制模式
        """
        for ip in self.server_ip_list:
            fi_fsa_v2.fast_set_enable(ip)
            fi_fsa_v2.fast_set_mode_of_operation(ip, fi_fsa_v2.FSAModeOfOperation.CURRENT_CONTROL)

    def set_current(self, current_value):
        """
        设置电机电流值
        参数:
        - current_value: 要设置的电流值，应该是一个和电机数量对应的numpy数组或列表，元素单位等需根据实际情况确定
        """
        if len(current_value)!= self.motors_num:
            raise ValueError("The length of current_value should match the number of motors.")
        for idx, ip in enumerate(self.server_ip_list):
            fi_fsa_v2.fast_set_current_control(ip, current_value[idx])

    def set_position(self, target_position):
        """
        设置电机的位置控制模式，使用样条插值将电机平滑地设置到目标位置
        参数:
        - target_position: 目标位置值，应该是一个和电机数量对应的numpy数组，元素单位等需根据实际情况确定（示例中类似角度[deg]）
        """
        if len(target_position)!= self.motors_num:
            raise ValueError("The length of target_position should match the number of motors.")

        _, _ = self.get_pvc()

        # 定义插值的点数，可根据实际情况调整
        num_interpolation_points = 10
        interpolation_times = np.linspace(0, 1, num_interpolation_points)

        for motor_idx in range(self.motors_num):
            start_pos = self.current_positions[motor_idx]
            end_pos = target_position[motor_idx]

            # 创建三次样条插值对象
            cs = CubicSpline([0, 1], [start_pos, end_pos])

            # 根据插值时间点计算对应的位置序列
            interpolation_positions = cs(interpolation_times)

            for pos in interpolation_positions:
                interpolated_target = np.array([pos])
                for ip in self.server_ip_list:
                    fi_fsa_v2.fast_set_position_control(ip, interpolated_target)
                time.sleep(0.001)  # 适当的延迟，让电机有时间响应，可根据实际调整


if __name__ == "__main__":
    motor = MOTOR()
    server_ip_list = motor.get_motors_ip()
    motor.set_position_mode()

    # 假设要设置的目标位置，这里示例为全0位置，你可以替换为实际需要的目标位置数组
    target_position = [10,10,10,10,10,10,10,10,10,10,10,10]
    motor.set_position(target_position)
    
