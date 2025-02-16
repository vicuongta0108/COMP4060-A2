# Abstract base class for the Robot module
from abc import ABC, abstractmethod

class Robot(ABC):
    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def terminate(self):
        pass