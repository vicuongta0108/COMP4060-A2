from abc import ABC, abstractmethod
from epucklib.epuck_state import EPuckState

class Robot(ABC):
    def __init__(self):
        self.robot_pos = None
        self.state = EPuckState()

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
        pass

    # Should be passing *
    @abstractmethod
    def odom_update(self, pos):
        pass