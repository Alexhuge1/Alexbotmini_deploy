import math
import time
import numpy as np
import torch
from tqdm import tqdm
from collections import deque

# import robot config
# from scipy.spatial.transform import Rotation as R
# from config.custom.alexbotmini_config import alexbotminiCfg as cfg
# sudo chmod a+rw /dev/ttyUSB0
# import module motor
from sim2real.module.motor.motors import MOTOR
from sim2real.module.utils import utils

# import module imu
from sim2real.module.imu.imu import IMU

######################################################################
# init robot & add robot config
class robot_config:
    #     PD Drive parameters:
    # stiffness = {'1': 180.0, '2': 120.0, '3': 120.0, '4': 180.0, '5': 45 , '6': 45}
    # damping = {'1': 10, '2': 8, '3': 8.0, '4': 10, '5': 2.5 , '6' : 2.5}
    kps = np.array([180, 200, 120, 180, 120, 120, 180, 200, 120, 180, 120, 120], dtype=np.double) * 0.4
    kds = np.array([10, 8, 8, 10, 6, 6, 10, 8, 8, 10, 6, 6], dtype=np.double) * 0.8

    target_q_limit = np.array([60, 36, 36, 60, 45, 45, 60, 36, 36, 60, 45, 45,], dtype=np.double)
    target_q_limit = np.deg2rad(target_q_limit)
    initial_position=np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.double)
    default_joint_angles=np.array([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8], dtype=np.double)
    num_actions=12
    action_scale = 0.25
    class normalization:
        class obs_scales:
            lin_vel = 2.
            ang_vel = 1.
            dof_pos = 1.
            dof_vel = 0.05
            quat = 1.
            height_measurements = 5.0
        clip_observations = 18.
        clip_actions = 18.
    
    class env:
        # change the observation dim
        frame_stack = 15
        c_frame_stack = 3
        num_single_obs = 47
        num_observations = int(frame_stack * num_single_obs)
        single_num_privileged_obs = 73
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)
        num_actions = 12
        num_envs = 2048
        episode_length_s = 24  # episode length in seconds
        use_ref_actions = False
# motor init
server_ip_list = ['192.168.137.101', '192.168.137.102', '192.168.137.103',
                  '192.168.137.104', '192.168.137.105', '192.168.137.106',
                  '192.168.137.107', '192.168.137.108', '192.168.137.109',
                  '192.168.137.110', '192.168.137.111', '192.168.137.112'   
                  ]
motors_num = 12
server_ip_list_test = []
motor = MOTOR()

# imu init
# If there are multiple USB devices here, replace them with the actual serial port names.
__import__('os').system('sudo chmod a+rw /dev/ttyUSB0')
port = "/dev/ttyUSB0"
baudrate = 115200
n = 1
imu = IMU(port, baudrate)

# robot init
target_q = np.zeros((robot_config.num_actions), dtype=np.double)
action = np.zeros((robot_config.num_actions), dtype=np.double)

class cmd:
    # TODO: changed into joystick
    vx = 0.0
    vy = 0.0
    dyaw = 0.0

######################################################################
class robot:
    def __init__(self):
        self.init()
        self.sim_actions = []
        self.real_actions = []  # 初始化存储实际动作的列表

    def init(self):
        # motor init， FFTAI_fsa position control is base on current(force) control
        motor.get_motors_ip()
        motor.set_position_mode()
        motor.get_pvc()
        motor.set_pd_imm_all(robot_config.kps,robot_config.kds)
        motor.set_position(robot_config.initial_position)
        time.sleep(1)

        # 设置电机初始条件
        motor.set_position(robot_config.default_joint_angles)
        time.sleep(1)
        # motor.get_pvc() 

        # imu init
        # imu.cmd_read(port, baudrate)

    def get_obs(self):
        motor.get_pvc()
        quat, omega = imu.cmd_read()
        q = motor.q
        dq = motor.dq
        # print("quat value:", quat)
        # print("gvec value:", gvec)
        return (q, dq, quat, omega)

    def run_alexbotmini(self, policy):
        """
        Run the alexbotmini robot using the provided policy and configuration.
        Args:
            policy: The policy used for controlling the simulation.
            cfg: The configuration object containing simulation settings.
        Returns:
        None
        """
        hist_obs = deque()
        for _ in range(robot_config.env.frame_stack):
            hist_obs.append(np.zeros([1, robot_config.env.num_single_obs], dtype=np.double))

        target_q = np.zeros((robot_config.num_actions), dtype=np.double)
        action = np.zeros((robot_config.num_actions), dtype=np.double)

        count_lowlevel = 0

        default_angle = np.zeros((robot_config.num_actions),dtype=np.double)

        default_angle[0] = robot_config.default_joint_angles[0]
        default_angle[1] = robot_config.default_joint_angles[1]
        default_angle[2] = robot_config.default_joint_angles[2]
        default_angle[3] = robot_config.default_joint_angles[3]
        default_angle[4] = robot_config.default_joint_angles[4]
        default_angle[5] = robot_config.default_joint_angles[5]
        default_angle[6] = robot_config.default_joint_angles[6] 
        default_angle[7] = robot_config.default_joint_angles[7]
        default_angle[8] = robot_config.default_joint_angles[8]
        default_angle[9] = robot_config.default_joint_angles[9]
        default_angle[10] = robot_config.default_joint_angles[10]
        default_angle[11] = robot_config.default_joint_angles[11]
        default_angle_rad = np.deg2rad(default_angle)

        dt=0.01

        policy_input = np.zeros([1, robot_config.env.num_observations], dtype=np.float32)
        while True:
            start_time = time.time()
            # Obtain an observation
            q, dq, quat, omega = self.get_obs()
            q = q[-12:]
            dq = dq[-12:]
            q = np.array(q)
            dq = np.array(dq)
            q = np.deg2rad(q)
            dq = np.deg2rad(dq)
            omega = np.deg2rad(omega)

            if count_lowlevel % 1 == 0:
                obs = np.zeros([1, robot_config.env.num_single_obs], dtype=np.float32)
                # 这里假设quat是符合要求的格式（比如一维数组等，如果不符合还需进一步处理，如前面提到的解包相关调整）
                eu_ang = utils.quaternion_to_euler_array(quat)
                eu_ang[eu_ang > math.pi] -= 2 * math.pi

               
                obs[0, 0] = math.sin(2 * math.pi * count_lowlevel * dt / 0.64)
                obs[0, 1] = math.cos(2 * math.pi * count_lowlevel * dt / 0.64)
                obs[0, 2] = cmd.vx * robot_config.normalization.obs_scales.lin_vel
                obs[0, 3] = cmd.vy * robot_config.normalization.obs_scales.lin_vel
                obs[0, 4] = cmd.dyaw * robot_config.normalization.obs_scales.ang_vel
                obs[0, 5:17] = (q-default_angle_rad)* robot_config.normalization.obs_scales.dof_pos
                obs[0, 17:29] = dq* robot_config.normalization.obs_scales.dof_vel
                obs[0, 29:41] = action
                obs[0, 41:44] = omega  # angular velocity
                obs[0, 44:47] = eu_ang # euler angle
                # Limit the data within the range from -cfg.normalization.clip_observations to cfg.normalization.clip_observations
                obs = np.clip(obs, -robot_config.normalization.clip_observations, robot_config.normalization.clip_observations)
                print('obs[0, 0] ', obs[0, 0] )
                print('obs[0, 1] ', obs[0, 1] )
                print('obs[0, 2] ', obs[0, 2] )
                print('obs[0, 3] ', obs[0, 3] )
                print('obs[0, 4] ', obs[0, 4] )
                print('obs[0, 5:17]', obs[0, 5:17])
                print('obs[0, 17:29]', obs[0, 17:29])
                print('obs[0, 29:41]', obs[0, 29:41])
                print('obs[0, 41:44]', obs[0, 41:44])
                print('obs[0, 44:47]', obs[0, 44:47])

                hist_obs.append(obs)
                hist_obs.popleft()

                for i in range(robot_config.env.frame_stack):
                    policy_input[0, i * robot_config.env.num_single_obs:(i + 1) * robot_config.env.num_single_obs] = hist_obs[i][0, :]
                # policy_input = torch.tensor(policy_input)
                policy_input_tensor = torch.from_numpy(policy_input)

                action = policy(policy_input_tensor)[0].detach().numpy()
                action = np.clip(action, -robot_config.normalization.clip_actions, robot_config.normalization.clip_actions)
                target_q = action * robot_config.action_scale + default_angle_rad

                ############# 并联脚 ##################

                target_q = np.clip(target_q, -robot_config.target_q_limit, robot_config.target_q_limit)  # rad
                motor.set_position(np.rad2deg(target_q))  # 设置电机位置时转换回角度
                

                # print('target_q = :(rad)', target_q)  # rad
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


if __name__ == '__main__':
    device = torch.device("cpu")
    policy = torch.jit.load('sim2real/loadmodel/test06_20250217(canuse)/policy_1.pt', map_location=device)
    robot = robot()  # 创建robot类实例
    robot.run_alexbotmini(policy)
