from planner import Planner
from navigator import Navigator

class PlannerMoveOnce(Planner):
    def __init__(self, target):
        super().__init__()
        self.target = target # tuple of target example use: PlannerMoveOnce( (300, 1, math.pi/2) )
        self.navigator = Navigator() # monitoring navigator

    def setup(self):
        print("Planner finish setup!")

    def update(self):
        if not self.navigator.has_hit_target:
            self.navigator.set_target(self.target)  # give target to monitoring navigator
            self.navigator.has_hit_target = False # returns false when the target is reached
        return self.navigator.has_hit_target

    def terminate(self):
        print("Planner terminated")