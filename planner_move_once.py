from planner import Planner

class PlannerMoveOnce(Planner):
    # Take a simple target as a tuple (distance_mm, speed_percent, delta_theta_rad) in its constructor
    def __init__(self, target):
        super().__init__()

    def setup(self):
        pass

    # Give the target tuple to monitoring navigator and return False when the target is reached
    def update(self):
        if not self.target_reached:
            self.controller.navigator.set_target(self.target)
            return True

        if self.controller.navigator.target_reached():
            return False
        return True

    def terminate(self):
        pass