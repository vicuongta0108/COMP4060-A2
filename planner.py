# Abstract base class for Planner module
# High level planning of what the robot should do next, sets target movements for the
# navigator to use and manages state machines.
from abc import ABC, abstractmethod

class Planner(ABC):
    def __init__(self):
        self.running = True

    @abstractmethod
    def setup(self):
        pass

    # Return True if still running, False if done planning
    @abstractmethod
    def update(self):
        return self.running

    @abstractmethod
    def terminate(self):
        pass