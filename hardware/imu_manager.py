from sim2real.module.imu.imu import IMU

class IMUManager:
    def __init__(self, port="/dev/ttyUSB0", baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.imu = None
        
    def initialize(self):
        __import__('os').system('sudo chmod a+rw /dev/ttyUSB0')
        self.imu = IMU(self.port, self.baudrate)
        
    def get_imu_data(self):
        if self.imu is None:
            raise RuntimeError("IMU not initialized")
        return self.imu.cmd_read()