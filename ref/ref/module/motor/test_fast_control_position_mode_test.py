from fi_fsa import fi_fsa_v2
import time
import math
from scipy.interpolate import CubicSpline
import numpy as np

server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112']


def set_position(target_position):
    server_ip_list = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
    if server_ip_list:
        # 获取电机数量
        motors_num = len(server_ip_list)

        # enable all the motors
        for i in range(len(server_ip_list)):
            fi_fsa_v2.fast_set_enable(server_ip_list[i])

        # set work at position control mode
        for i in range(len(server_ip_list)):
            fi_fsa_v2.fast_set_mode_of_operation(
                server_ip_list[i], fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL
            )

        # 获取当前电机位置
        current_positions = []
        for i in range(len(server_ip_list)):
            position, _, _ = fi_fsa_v2.fast_get_pvc(server_ip_list[i])
            current_positions.append(position)

        # 检查 target_position 的长度是否足够
        if len(target_position) < motors_num:
            raise ValueError("target_position 的长度不足")

        # 定义插值的点数，可根据实际情况调整
        num_interpolation_points = 10
        interpolation_times = np.linspace(0, 1, num_interpolation_points)

        # 对每个电机进行样条插值操作
        for motor_idx in range(motors_num):
            start_pos = current_positions[motor_idx]
            end_pos = target_position[motor_idx]

            # 创建三次样条插值对象
            cs = CubicSpline([0, 1], [start_pos, end_pos])

            # 根据插值时间点计算对应的位置序列
            interpolation_positions = cs(interpolation_times)

            for pos in interpolation_positions:
                # 确保将 pos 转换为合适的类型
                interpolated_target = float(pos)
                for i in range(len(server_ip_list)):
                    fi_fsa_v2.fast_set_position_control(server_ip_list[i], interpolated_target,velocity_ff=0.01)
                time.sleep(0.0001)  # 适当的延迟，让电机有时间响应，可根据实际调整


if __name__ == "__main__":
    # 目标位置，这里示例为全 0 位置，可根据实际修改
    # target_position = [10,10,10,10,10,10,10,10,10,10,10,10]
    target_position = [0,0,0,0,0,0,0,0,0,0,0,0]
    set_position(target_position)
