# Abstract base class for the Robot module
# Manages robot connection and communication, sends commands and gets sensor data.
from abc import ABC, abstractmethod

class Robot(ABC):
    def __init__(self):
        self.robot_pos = [0.0, 0.0, 0.0] # x, y, theta
        self.state = None

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def terminate(self):
        pass

    @abstractmethod
    def odom_reset(self):
        # Reset odometry
        self.robot_pos = [0.0, 0.0, 0.0]
        print(f'Odometry reset!: {self}')

    # Should be passing *
    @abstractmethod
    def odom_update(self, pos):
        self.robot_pos = pos
        print(f'Pose updated!: {self.robot_pos}')