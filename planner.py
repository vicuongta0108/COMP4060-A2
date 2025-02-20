from abc import ABC, abstractmethod

class Planner(ABC):
    def __init__(self, target):
        self._running = False
        self.target_reached = False
        self.controller = None
        self.target = target

    @abstractmethod
    def setup(self):
        pass

    # Return True if still running, False if done planning
    @abstractmethod
    def update(self):
        return self._running

    @abstractmethod
    def terminate(self):
        pass