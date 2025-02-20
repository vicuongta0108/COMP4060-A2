from abc import ABC, abstractmethod

class Planner(ABC):
    def __init__(self):
        self._running = True
        self.target = None
        self.target_reached = False
        self.controller = None

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