from abc import ABC, abstractmethod
from epucklib.epuck_state import EPuckState

class Robot(ABC):
    def __init__(self, com_port):
        if not isinstance(com_port, str):
            raise TypeError("com_port must be a string")

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

    @abstractmethod
    def odom_update(self):
        pass