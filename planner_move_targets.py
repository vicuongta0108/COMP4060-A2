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
        self._first_cycle = False
        self._cycle_index = 0

    def setup(self):
        # self.navigator = Navigator() # monitoring navigator
        # self._controller._navigator._target = self._target
        self._target_set = False
        self._target_index = 0
        self._cycle_index = 1
        self._first_cycle = True
        # print('Cycle No #1')
        # print("Planner finish setup!")

    def update(self):
        # if not self._target_set and self._target_index < len(self._target) and self._controller._navigator.has_hit_target:
        #     self._controller._navigator.set_target(self._target[self._target_index])
        #     self._target_index += 1
        #     self._target_set = True
        # print('Cycle       :', self._cycle)
        # print('Cycle index :', self._cycle_index)
        # print('Target index:', self._target_index)
        # print('Target len  :', len(self._target))
        if self._first_cycle:
            print('Cycle No #1')
            self._first_cycle = False
        

        if self._controller._navigator.has_hit_target and self._target_index < len(self._target):
            print('Setting new target:', self._target[self._target_index])
            self._controller._navigator.set_target(self._target[self._target_index])
            self._target_index += 1

        if self._target_index == len(self._target) and self._cycle_index < self._cycle:
            print('Cycle No. #', self._cycle_index + 1)
            self._target_index = 0
            self._cycle_index += 1
            
        
        return self._target_index < len(self._target) or not self._controller._navigator.has_hit_target

    def terminate(self):
        self._controller = None
        self._target_set = False
        self._target = None
        self._target_index = 0
        self._first_cycle = False
        self._cycle_index = 0
        print("Planner terminated")