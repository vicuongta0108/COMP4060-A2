from navigator import Navigator
from robot import Robot
from epuck_lib import *

MAX_SPEED = 1200

class NavigatorDiffDirectPercent(Navigator):
    def __init__(self, target):
        super().__init__()
        self._controller = None
        self._target = target

    def setup(self):
        if self._target is None:
            self._target = (0.4, 0.4)

    def update(self):
        state, _ = self._controller._robot.odom_update()
        
        left_speed = self._target[0] * MAX_SPEED
        right_speed = self._target[1] * MAX_SPEED
        
        state.act_left_motor_speed = left_speed
        state.act_right_motor_speed = right_speed
                
        self._controller._robot._state = state

    def terminate(self):
        self._controller._robot._state.stop_all()  # Stop robot motors
        self._target = None

    # Use the given target tuple to solve how the robot should move.
    def set_target(self, target):
        self._target = target
        