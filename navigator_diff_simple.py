# Takes a target movement and converts this into robot motion commands.
from navigator import Navigator
from robot_epuck import RobotEPuck
from epuck_lib import diff_drive_inverse_kin, move_steps
import time

MAX_SPEED = 1000
MIN_SPEED = 100

class NavigatorDiffSimple(Navigator):
    def __init__(self):
        super().__init__()
        self.target_mm = None
        self.moved_mm = None
        self.epuckcomm = None

    def setup(self):
        self.target_mm = 0
        self.moved_mm = 0
        self.epuckcomm = RobotEPuck.setup()

    def update(self):
        # Moved distance
        self.moved_mm += RobotEPuck.odom_update()
        speed = (1 - self.moved_mm / self.target_mm) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
        


    # Stop robot movement
    def terminate(self):
        RobotEPuck.terminate()

    # Return target distance for robot to move
    def set_target(self, target):
        self.target_mm = move_steps(self.epuckcomm, *diff_drive_inverse_kin(target))