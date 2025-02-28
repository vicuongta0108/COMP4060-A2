from planner import Planner
from navigator import Navigator

class PlannerMoveOnce(Planner):
    def __init__(self, target):
        super().__init__()
        self._controller = None
        self._target_set = False
        self._target = target # tuple of target example use: PlannerMoveOnce( (300, 1, math.pi/2) )

    def setup(self):
        # self.navigator = Navigator() # monitoring navigator
        # self._controller._navigator._target = self._target
        self._target_set = False
        print("Planner finish setup!")

    def update(self):
        if not self._target_set:
            self._controller._navigator.set_target(self._target)
            self._target_set = True
        
        return not self._controller._navigator.has_hit_target

    def terminate(self):
        self._controller = None
        self._target_set = False
        self._target = None
        print("Planner terminated")