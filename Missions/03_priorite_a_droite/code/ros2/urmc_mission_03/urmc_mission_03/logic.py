"""Same right-of-way and odometry controller in simulation and on the robot."""
from urmc_core.priority import PriorityBrain

class Mission(PriorityBrain):
    def __init__(self,mission=3,phase='checker',speed=.07,kp=.8):
        super().__init__(mission,phase,speed,kp,finish_after_cross=True)

    def step(self,o,now):
        return super().step(o,now,dict(speed=self.speed,steering_gain=self.kp))
