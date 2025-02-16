# Abstract base class for Navigator module
from abc import ABC, abstractmethod

class Navigator(ABC):
    @abstractmethod
    def setup(self):
        pass

    @abstractmethod
    def update(self):
        pass

    @abstractmethod
    def terminate(self):
        pass