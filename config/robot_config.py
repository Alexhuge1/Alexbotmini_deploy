import numpy as np
import math

class RobotConfig:
    kps = np.array([180, 200, 120, 180, 120, 120, 180, 200, 120, 180, 120, 120], dtype=np.double) * 0.38
    kds = np.array([10, 8, 8, 10, 6, 6, 10, 8, 8, 10, 6, 6], dtype=np.double) * 0.8

    target_q_limit = np.array([60, 36, 36, 60, 45, 45, 60, 36, 36, 60, 45, 45,], dtype=np.double)
    target_q_limit = np.deg2rad(target_q_limit)
    initial_position = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], dtype=np.double)
    default_joint_angles = np.array([-10, 0, 0, 18, 8, 8, 10, 0, 0, -18, -8, 8], dtype=np.double)
    num_actions = 12
    action_scale = 0.25

    class Normalization:
        class ObsScales:
            lin_vel = 2.
            ang_vel = 1.
            dof_pos = 1.
            dof_vel = 0.05
            quat = 1.
            height_measurements = 5.0
        clip_observations = 18.
        clip_actions = 18.
    
    class Env:
        frame_stack = 15
        c_frame_stack = 3
        num_single_obs = 47
        num_observations = int(frame_stack * num_single_obs)
        single_num_privileged_obs = 73
        num_privileged_obs = int(c_frame_stack * single_num_privileged_obs)
        num_actions = 12
        num_envs = 2048
        episode_length_s = 24
        use_ref_actions = False