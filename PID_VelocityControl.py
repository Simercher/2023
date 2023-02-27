class PID_VelocityControl():
    def __init__(self, kp = 0, ki = 0, kd = 0, d_time = 50, max_velocity = 75, min_velocity = 0, max_PID = 75, min_PID = 0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.error = 0
        self.pre_error = 0
        self.P = 0
        self.I = 0
        self.D = 0
        self.PID = 0
        self.d_time = d_time
        self.velocity = 0
        self.max_Velocity = max_velocity
        self.min_Velocity = min_velocity
        self.max_PID = max_PID
        self.min_PID = min_PID
    def calPID(self, target, encoder):
        self.error = target - encoder
        self.P = self.kp * self.error
        self.I += self.ki * (self.error * self.d_time)
        self.D = self.kd * ((self.error - self.pre_error) / self.d_time)
        self.pre_error = self.error
        self.PID = self.P + self.I + self.D
    def setVelocity(self):
        self.convertPID()
        self.velocity += self.PID
        if self.velocity >= self.max_Velocity:
            self.velocity = self.max_Velocity
        elif self.velocity < self.min_Velocity:
            self.velocity = self.min_Velocity
        return self.velocity
    def convertPID(self):
        self.PID = (self.PID - self.min_PID) / (self.max_PID - self.min_PID) * (self.max_Velocity - self.min_Velocity)