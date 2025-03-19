import numpy as np
import torch
import time
import math
from collections import deque
from sim2real.module.utils import utils
from config.robot_config import RobotConfig
from hardware.motor_manager import MotorManager
from hardware.imu_manager import IMUManager
from hardware.joystick_manager import JoystickManager

class Robot:
    def __init__(self):
        self.config = RobotConfig()
        self.motor_manager = MotorManager()
        self.imu_manager = IMUManager()
        self.joystick = JoystickManager()
        self.sim_actions = []
        self.real_actions = []

    def initialize(self):
        self.imu_manager.initialize()
        self.motor_manager.initialize(
            self.config.kps,
            self.config.kds,
            self.config.initial_position,
            self.config.default_joint_angles
        )

    def get_obs(self):
        q, dq = self.motor_manager.get_motor_states()
        quat, omega = self.imu_manager.get_imu_data()
        return q, dq, quat, omega

    def run_alexbotmini(self, policy):
        """
        Run the alexbotmini robot using the provided policy and configuration.
        Args:
            policy: The policy used for controlling the simulation.
        Returns:
            None
        """
        hist_obs = deque()
        for _ in range(self.config.Env.frame_stack):
            hist_obs.append(np.zeros([1, self.config.Env.num_single_obs], dtype=np.double))

        target_q = np.zeros((self.config.num_actions), dtype=np.double)
        action = np.zeros((self.config.num_actions), dtype=np.double)
        count_lowlevel = 0
        
        # 使用numpy的广播功能简化赋值
        default_angle = np.array(self.config.default_joint_angles, dtype=np.double)
        default_angle_rad = np.deg2rad(default_angle)

        dt = 0.01
        policy_input = np.zeros([1, self.config.Env.num_observations], dtype=np.float32)

        while True:
            start_time = time.time()
            
            # 获取观测值
            q, dq, quat, omega = self.get_obs()
            q = np.array(q[-12:])
            dq = np.array(dq[-12:])
            q = np.deg2rad(q)
            dq = np.deg2rad(dq)
            omega = np.deg2rad(omega)

            if count_lowlevel % 1 == 0:
                obs = np.zeros([1, self.config.Env.num_single_obs], dtype=np.float32)
                
                # 计算欧拉角
                eu_ang = utils.quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

                # 获取机器人速度指令
                vx, vy, dyaw = self.joystick.get_command()
                
                # 构建观测向量
                obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * dt / 0.64)
                obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * dt / 0.64)
                obs[0, 2] = vx * self.config.Normalization.ObsScales.lin_vel
                obs[0, 3] = vy * self.config.Normalization.ObsScales.lin_vel
                obs[0, 4] = dyaw * self.config.Normalization.ObsScales.ang_vel
                obs[0, 5:17] = (q - default_angle_rad) * self.config.Normalization.ObsScales.dof_pos
                obs[0, 17:29] = dq * self.config.Normalization.ObsScales.dof_vel
                obs[0, 29:41] = action
                obs[0, 41:44] = omega
                obs[0, 44:47] = eu_ang

                # 限制观测值范围
                obs = np.clip(obs, -self.config.Normalization.clip_observations, 
                            self.config.Normalization.clip_observations)

                # 更新历史观测
                hist_obs.append(obs)
                hist_obs.popleft()

                # 构建策略输入
                for i in range(self.config.Env.frame_stack):
                    policy_input[0, i * self.config.Env.num_single_obs:
                                 (i + 1) * self.config.Env.num_single_obs] = hist_obs[i][0, :]

                # 执行策略
                policy_input_tensor = torch.from_numpy(policy_input)
                action = policy(policy_input_tensor)[0].detach().numpy()
                action = np.clip(action, -self.config.Normalization.clip_actions, 
                               self.config.Normalization.clip_actions)
                
                # 计算目标关节角度
                target_q = action * self.config.action_scale + default_angle_rad
                target_q = np.clip(target_q, -self.config.target_q_limit, 
                                 self.config.target_q_limit)
                print('target_q = :(deg)', np.rad2deg(target_q))  # deg
                
                end_time = time.time()
                execution_time = end_time - start_time
                if execution_time < dt:
                    time.sleep(dt - execution_time)
                print(f"exec_time: {execution_time} ")

            count_lowlevel += 1    
            # # Generate PD control
            # tau = utils.pd_control(target_q, q, robot_config.kps, target_dq, dq, robot_config.kds)  # Calc torques
            # tau = np.clip(tau, -robot_config.tau_limit, robot_config.tau_limit)  # Clamp torques