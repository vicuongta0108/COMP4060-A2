# Takes a target movement and converts this into robot motion commands.
from navigator import Navigator
from robot import Robot
from epuck_lib import diff_drive_inverse_kin, move_steps
MAX_SPEED = 1000

class NavigatorDiffSimple(Navigator):
    def __init__(self):
        super().__init__()
        self.target = None
        self.moving = None
        self.epuckcomm = Robot.setup

    def setup(self):
        pass

    def set_target(self, target):
        distance_mm, speed_percent, delta_theta_rad = target

        distance_moved = move_steps(self.epuckcomm, *diff_drive_inverse_kin(distance_mm, speed_percent, delta_theta_rad))






