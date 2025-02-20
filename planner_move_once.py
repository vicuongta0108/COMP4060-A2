from planner import Planner
import navigator as Navigator

class PlannerMoveOnce(Planner):
    # Take a simple target as a tuple (distance_mm, speed_percent, delta_theta_rad) in its constructor
    def __init__(self, target):
        super().__init__(target)

    def setup(self):
        self.target_reached = False

    # Give the target tuple to monitoring navigator and return False when the target is reached
    def update(self):
        if not self.target_reached:
            Navigator.set_target(self.target)
            self.target_reached = True
            return True
        return False

    def terminate(self):
        print("Planner terminated!")