import numpy as np
from fi_fsa import fi_fsa_v2
import time
from scipy.interpolate import CubicSpline
import numpy as np
import time

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
            raise ValueError('Lost connection of motors. We only find',self.motors_num,'motor(s)')
        return server_ip_list_test

    def get_pvc(self):#(degree)
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        """
        self.q = []
        self.dq = []
        for ip in self.server_ip_list:
            position, velocity, current = fi_fsa_v2.fast_get_pvc(ip)
            # pos = np.array(position).astype(np.double) * (np.pi / 180)
            # vel = np.array(velocity).astype(np.double) * (np.pi / 180)
            pos = np.array(position).astype(np.double)
            vel = np.array(velocity).astype(np.double)
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

    def set_position(self, target_position):
        # change motors into target position
        for i in range(len(self.server_ip_list)):
            print(i,self.server_ip_list[i],target_position[i])
            # 从 target_position 列表中取出一个元素作为位置参数
            position = target_position[i]
            fi_fsa_v2.fast_set_position_control(self.server_ip_list[i], position)
            time.sleep(0.004)

if __name__ == "__main__":
    motor = MOTOR()
    server_ip_list = motor.get_motors_ip()
    motor.set_position_mode()
    target2_position = np.array([-40, 0, 0, 68, -28, 28, 40, 0, 0, -68, 28, 28])
    init_position = np.array([-10, 0, 0, 18, -8, 8, 10, 0, 0, -18, 8, 8])
    num_interpolation = 20  # 增加插值点数，使运动轨迹更细腻
    # 生成从 init_position 到 target2_position 的插值序列
    interpolation_sequence1 = np.linspace(init_position, target2_position, num_interpolation + 1)
    # 生成从 target2_position 回到 init_position 的插值序列
    interpolation_sequence2 = np.linspace(target2_position, init_position, num_interpolation + 1)

    while True:
        # 从 init_position 到 target2_position 的柔顺运动
        for position in interpolation_sequence1:
            motor.set_position(position)
            # time.sleep(0.005)  # 减小时间间隔，加快位置更新频率
        # 从 target2_position 回到 init_position 的柔顺运动
        for position in interpolation_sequence2:
            motor.set_position(position)
            # time.sleep(0.005)

# if __name__ == "__main__":
#     motor = MOTOR()
#     server_ip_list = motor.get_motors_ip()
#     current_positions = motor.get_pvc()
#     current_positions_array = np.array(current_positions)
#     print("current_position is ", current_positions_array)