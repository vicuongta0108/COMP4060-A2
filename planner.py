from abc import ABC, abstractmethod

class Planner(ABC):
    def __init__(self, target):
        self.target_reached = False
        self.target = target

    @abstractmethod
    def setup(self):
        pass

    # Return True if still running, False if done planning
    @abstractmethod
    def update(self):
        pass
    
    @abstractmethod
    def terminate(self):
        pass