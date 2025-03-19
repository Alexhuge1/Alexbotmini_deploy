from sim2real.module.motor.motors import MOTOR
import numpy as np

class MotorManager:
    def __init__(self):
        self.motor = MOTOR()
        self.server_ip_list = [
            '192.168.137.101', '192.168.137.102', '192.168.137.103',
            '192.168.137.104', '192.168.137.105', '192.168.137.106',
            '192.168.137.107', '192.168.137.108', '192.168.137.109',
            '192.168.137.110', '192.168.137.111', '192.168.137.112'
        ]
        
    def initialize(self, kps, kds, initial_position, default_angles):
        self.motor.get_motors_ip()
        self.motor.set_position_mode()
        self.motor.get_pvc()
        self.motor.set_pd_imm_all(kps, kds)
        self.motor.set_position(initial_position)
        self.motor.set_position(default_angles)
        
    def get_motor_states(self):
        self.motor.get_pvc()
        return self.motor.q, self.motor.dq
        
    def set_position(self, position):
        self.motor.set_position(position)