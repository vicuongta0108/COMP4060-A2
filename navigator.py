# Abstract base class for Navigator module
from abc import ABC, abstractmethod

class Navigator(ABC):
    def __init__(self):
        self.target = None
        self.target_reached = False

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
    def set_target(self, target):
        self.target = target