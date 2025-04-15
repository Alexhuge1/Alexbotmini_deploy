import math
import time
import numpy as np
import torch
import os
from collections import deque
from sim2real.module.motor.motors import MOTOR
from sim2real.module.imu.imu import IMU
from sim2real.module.utils import utils


#----------------------- 配置模块 -----------------------
class RobotConfig:
    """机器人硬件配置参数"""
    # PD控制参数
    kps = np.array([180, 200, 120, 180, 120, 120, 180, 200, 120, 180, 120, 120], dtype=np.double) * 0.38
    kds = np.array([10, 10, 8, 10, 6, 6, 10, 10, 8, 10, 6, 6], dtype=np.double) * 0.8

    # 运动限制参数
    target_q_limit = np.deg2rad([60, 36, 36, 60, 45, 45, 60, 36, 36, 60, 45, 45])
    default_joint_angles = np.deg2rad([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8])
    action_scale = 0.25
    
    # 观测参数
    class ObsParams:
        frame_stack = 15
        c_frame_stack = 3
        single_obs_dim = 47
        clip_observations = 18.0
        clip_actions = 18.0
        obs_scales = {
            'lin_vel': 2.0,
            'ang_vel': 1.0,
            'dof_pos': 1.0,
            'dof_vel': 0.05,
            'quat': 1.0,
            'height': 5.0
        }

#----------------------- 硬件接口模块 -----------------------
class HardwareInterface:
    """硬件设备管理类"""
    def __init__(self):
        self.motor = self._init_motors()
        self.imu = self._init_imu()
        
    def _init_motors(self):
        """初始化电机控制器"""
        motor = MOTOR()
        motor.set_position_mode()
        motor.set_pd_imm_all(RobotConfig.kps, RobotConfig.kds)
        motor.set_position(np.rad2deg(RobotConfig.default_joint_angles))
        time.sleep(1)  # 等待初始化完成
        return motor
        
    def _init_imu(self):
        """初始化IMU传感器"""
        os.system('sudo chmod a+rw /dev/ttyUSB0')
        return IMU("/dev/ttyUSB0", 115200)
    
    def get_sensor_data(self):
        """获取传感器数据"""
        self.motor.get_pvc()
        quat, omega = self.imu.cmd_read()
        
        return {
            'q': np.deg2rad(self.motor.q[-12:]),
            'dq': np.deg2rad(self.motor.dq[-12:]),
            'quat': quat,
            'omega': omega
        }

#----------------------- 控制核心模块 -----------------------
class AlexBotMiniController:
    """机器人主控制类"""
    def __init__(self, policy):
        self.config = RobotConfig()
        self.hardware = HardwareInterface()
        self.policy = policy
        
        # 初始化观测历史
        self.obs_history = deque(
            [np.zeros(self.config.ObsParams.single_obs_dim)] * self.config.ObsParams.frame_stack,
            maxlen=self.config.ObsParams.frame_stack
        )
        
        # 控制参数
        self.dt = 0.01
        self.cmd = {'vx':0.0, 'vy':0.0, 'dyaw':0.0}
        
    def _build_observation(self, sensor_data, action):
        """构建观测向量"""
        obs = np.zeros(self.config.ObsParams.single_obs_dim)
        
        # 时间特征
        t = time.time()
        obs[0] = math.sin(2 * math.pi * t / 0.64)
        obs[1] = math.cos(2 * math.pi * t / 0.64)
        
        # 控制命令
        obs[2] = self.cmd['vx'] * self.config.ObsParams.obs_scales['lin_vel']
        obs[3] = self.cmd['vy'] * self.config.ObsParams.obs_scales['lin_vel']
        obs[4] = self.cmd['dyaw'] * self.config.ObsParams.obs_scales['ang_vel']
        
        # 关节状态
        q_offset = sensor_data['q'] - self.config.default_joint_angles
        obs[5:17] = q_offset * self.config.ObsParams.obs_scales['dof_pos']
        obs[17:29] = sensor_data['dq'] * self.config.ObsParams.obs_scales['dof_vel']
        
        # 动作历史
        obs[29:41] = action
        
        # IMU数据
        print(sensor_data)
        eu_ang = utils.quaternion_to_euler_array(sensor_data['quat'])
        eu_ang[eu_ang > math.pi] -= 2 * math.pi
        obs[41:44] = sensor_data['omega']  # 角速度
        obs[44:47] = eu_ang  # 欧拉角
        
        
        return np.clip(obs, -self.config.ObsParams.clip_observations, self.config.ObsParams.clip_observations)
    
    def _compute_action(self, obs):
        """生成控制动作"""
        # 构建策略输入
        policy_input = np.concatenate(self.obs_history)
        policy_input_tensor = torch.from_numpy(policy_input.reshape(1, -1)).to(dtype=torch.float32)
        
        # 执行策略
        action = self.policy(policy_input_tensor)[0].detach().numpy()
        return np.clip(action, -self.config.ObsParams.clip_actions, self.config.ObsParams.clip_actions)
    
    def run(self):
        """主控制循环"""
        target_q = self.config.default_joint_angles.copy()
        
        while True:
            cycle_start = time.time()
            
            # 1. 获取传感器数据
            sensor_data = self.hardware.get_sensor_data()
            
            # 2. 生成观测值
            current_obs = self._build_observation(sensor_data, target_q)
            self.obs_history.append(current_obs)
            
            # 3. 计算控制动作
            action = self._compute_action(current_obs)
            
            # 4. 生成目标位置
            target_q = action * self.config.action_scale + self.config.default_joint_angles
            target_q = np.clip(target_q, -self.config.target_q_limit, self.config.target_q_limit)
            
            # 5. 发送电机指令
            self.hardware.motor.set_position(np.rad2deg(target_q))
            
            # 6. 维持控制频率
            elapsed = time.time() - cycle_start
            if elapsed < self.dt:
                time.sleep(self.dt - elapsed)

#----------------------- 主程序 -----------------------
if __name__ == '__main__':
    # 加载控制策略
    device = torch.device("cpu")
    policy = torch.jit.load('sim2real/loadmodel/test06_20250217_canuse/policy_1.pt', map_location=device)
    
    # 启动控制器
    controller = AlexBotMiniController(policy)
    controller.run()