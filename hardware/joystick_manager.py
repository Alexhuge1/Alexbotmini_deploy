class JoystickManager:
    def __init__(self):
        self.vx = 0.0
        self.vy = 0.0
        self.dyaw = 0.0
        
    def get_command(self):
        # TODO: Implement actual joystick reading
        return self.vx, self.vy, self.dyaw