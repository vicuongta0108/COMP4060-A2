# Abstract base class for Planner module
from abc import ABC, abstractmethod

class Planner(ABC):
    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def terminate(self):
        pass