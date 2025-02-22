from abc import ABC, abstractmethod

class Planner(ABC):
    def __init__(self):
        pass

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def terminate(self):
        pass