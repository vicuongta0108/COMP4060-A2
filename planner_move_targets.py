from planner import Planner
from navigator import Navigator

class PlannerMoveTargets(Planner):
    def __init__(self, target, cycle):
        super().__init__()
        self._controller = None
        self._target_set = False
        self._target = target # tuple of target example use: PlannerMoveOnce( (300, 1, math.pi/2) )
        self._target_index = 0
        self._cycle = cycle
        self._cycle_index = 0

    def setup(self):
        # self.navigator = Navigator() # monitoring navigator
        # self._controller._navigator._target = self._target
        self._target_set = False
        self._target_index = 0
        self._cycle_index = 0
        print("Planner finish setup!")

    def update(self):
        # if not self._target_set and self._target_index < len(self._target) and self._controller._navigator.has_hit_target:
        #     self._controller._navigator.set_target(self._target[self._target_index])
        #     self._target_index += 1
        #     self._target_set = True
        if self._controller._navigator.has_hit_target and self._target_index < len(self._target):
            self._controller._navigator.set_target(self._target[self._target_index])
            self._target_index += 1
        
        if self._target_index == len(self._target) and self._cycle_index < self._cycle:
            self._target_index = 0
            self._cycle_index += 1
        
        return self._target_index < len(self._target)

    def terminate(self):
        self._controller = None
        self._target_set = False
        self._target = None
        self._target_index = 0
        self._cycle_index = 0
        print("Planner terminated")