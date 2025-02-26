from navigator import Navigator
from robot import Robot
from epuck_lib import *

MAX_SPEED = 154 # mm/s
MIN_SPEED = 500

class NavigatorDiffSimple(Navigator):
    def __init__(self):
        super().__init__()
        self._controller = None
        self._speed_n_steps = None
        self.has_hit_target = False
        self._left_steps_moved = 0
        self._right_steps_moved = 0
        self._last_l_steps = 0
        self._last_r_steps = 0

    def setup(self):
        self._speed_n_steps = None
        self._left_steps_moved = 0
        self._right_steps_moved = 0
        self.has_hit_target = False  # Reset target tracking
        self._last_l_steps = 0
        self._last_r_steps = 0
        print("Navigator finish setup!")

    def update(self):
        # print(f'last left steps moved = {self._last_l_steps}')
        # print(f'last right steps moved = {self._last_r_steps}')
        # print(f'left steps moved = {self._left_steps_moved}')
        # print(f'right steps moved = {self._right_steps_moved}')
        state = self._controller._robot._state
        self._last_l_steps = state.sens_left_motor_steps
        self._last_r_steps = state.sens_right_motor_steps
        state, _ = self._controller._robot.odom_update()
        self._left_steps_moved += abs(steps_delta(self._last_l_steps, state.sens_left_motor_steps))
        self._right_steps_moved += abs(steps_delta(self._last_r_steps, state.sens_right_motor_steps))
        # print(f'last left steps moved = {self._last_l_steps}')
        # print(f'last right steps moved = {self._last_r_steps}')
        print(f'left steps moved = {self._left_steps_moved}')
        print(f'right steps moved = {self._right_steps_moved}')
        if self._left_steps_moved < self._speed_n_steps[2] or self._right_steps_moved < self._speed_n_steps[3]:
            self.has_hit_target = False
            self._controller._robot._state.act_left_motor_speed = self._speed_n_steps[0]
            self._controller._robot._state.act_right_motor_speed = self._speed_n_steps[1]
        else:
            self.has_hit_target = True
            self._controller._robot._state.stop_all()

    def terminate(self):
        self._controller._robot._state.stop_all()  # Stop robot motors
        self._left_steps_moved = 0
        self._right_steps_moved = 0
        self.has_hit_target = False
        self._speed_n_steps = None
        self._last_l_steps = 0
        self._last_r_steps = 0
        print("Navigator terminated")

    # Use the given target tuple and A1 kinematics to solve how the robot should move.
    def set_target(self, target):
        # self.target_mm = diff_drive_inverse_kin(target)
        actual_speed = target[1] * MAX_SPEED / 100
        self._speed_n_steps = diff_drive_inverse_kin(target[0], actual_speed, target[2])
        