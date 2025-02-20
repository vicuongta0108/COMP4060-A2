from abc import ABC, abstractmethod

class Navigator(ABC):
    def __init__(self):
        self.target = None
        self.has_hit_target = False

    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def terminate(self, epuckcomm):
        pass

    @abstractmethod
    def set_target(self, epuckcomm, target):
        pass