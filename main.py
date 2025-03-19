import torch
from robot import Robot

def main():
    device = torch.device("cpu")
    policy = torch.jit.load('sim2real/loadmodel/test06_20250217_canuse/policy_1.pt', map_location=device)
    
    robot = Robot()
    robot.initialize()
    robot.run_alexbotmini(policy)

if __name__ == '__main__':
    main()