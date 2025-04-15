import numpy as np
from fi_fsa import fi_fsa_v2


class Motor:
    def __init__(self):
        """
        初始化电机对象，在初始化时获取该电机对应的服务器IP地址
        """
        self.server_ip = self._get_server_ip()

    def _get_server_ip(self):
        """
        私有方法，用于获取电机对应的服务器IP地址
        返回值：
        - server_ip：获取到的电机对应的服务器IP地址，如果获取失败返回None
        """
        ip_list = fi_fsa_v2.broadcast_func_with_filter(filter_type="Actuator")
        if ip_list:
            return ip_list[0]  # 这里简单返回列表中的第一个IP，实际可能需要更合理的分配逻辑，比如按顺序一一对应等
        return None

    def get_pvc(self):
        """
        获取电机的位置（position）、速度（velocity）和电流（current）信息
        返回值：
        - position：以numpy.ndarray（float64类型）表示的电机位置信息，对应类似q变量（关节位置）的概念
        - velocity：以numpy.ndarray（float64类型）表示的电机速度信息，对应类似dq变量（关节速度）的概念
        - current：电机的电流值（具体类型根据fi_fsa_v2模块返回确定，这里暂未做类型转换处理）
        """
        position, velocity, current = fi_fsa_v2.fast_get_pvc(self.server_ip)
        position = np.array(position).astype(np.double)
        velocity = np.array(velocity).astype(np.double)
        return position, position, current

    def set_position_control(self, target_position):
        """
        设置电机的位置控制模式，将电机设置到目标位置
        参数：
        - target_position：目标位置值（具体单位等需根据实际情况确定，示例中传入的单位类似角度[deg]）
        """
        fi_fsa_v2.fast_set_enable(self.server_ip)
        fi_fsa_v2.fast_set_mode_of_operation(
            self.server_ip, fi_fsa_v2.FSAModeOfOperation.POSITION_CONTROL
        )
        fi_fsa_v2.fast_set_position_control(self.server_ip, target_position)


if __name__ == "__main__":
    # 创建电机对象
    motor = Motor()
    if motor.server_ip:
        # 获取电机的PVC信息并打印
        position, velocity, current = motor.get_pvc()
        print(
            f"Position = {position}, Velocity = {velocity}, Current = {current}"
        )

        # 设置电机到目标位置（这里示例设置为0，实际可按需调整）
        target_position = 0  # [deg]
        motor.set_position_control(target_position)
    else:
        print("无法获取电机对应的服务器IP地址，无法进行后续操作。")
