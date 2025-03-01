from epucklib.state import State
from planner import Planner
import random
import time
import numpy as np
import math

TIME_OUT = 5 # seconds

# Constants from planner_linefollow.py
MAX_SPEED = 1200  # mm/s
Kp, Ki, Kd = 0.5, 0.01, 1.5  # PID tuning parameters

class State_FindLine(State):
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)
        self._search_timer = 0
        self._direction = 1  # 1 for right, -1 for left
        
    def enter(self):
        self._search_timer = 0
        # Randomly choose initial search direction
        self._direction = random.choice([-1, 1])
        self.threshold = 300
        
        if self._debug:
            print("Entering Find Line State")
        
    def update(self):
        self._search_timer += 1
        state, _ = self.controller._robot.odom_update()
        
        # Get ground sensor readings
        l_sens = state.sens_ground_prox[0]
        r_sens = state.sens_ground_prox[2]
        print("error in find line:", abs(l_sens - r_sens))
        
        # # If line detected, transition to get_on_line
        if abs(l_sens - r_sens) > self.threshold:
            return self.transition_to(self.parent.s_get_on_line)
        
        # Switch direction every ~10 seconds to expand search
        if self._search_timer > 25:
            self._direction *= -1
            self._search_timer = 0
            
        if self._direction > 0:
            # state.act_left_motor_speed = outer_speed
            # state.act_right_motor_speed = inner_speed
            self.controller._navigator.set_target((0.4, 0.05))
        else:
            self.controller._navigator.set_target((0.05, 0.4))
            # state.act_left_motor_speed = inner_speed
            # state.act_right_motor_speed = outer_speed
            
        self.controller._robot._state = state
        return self

    def leave(self):
        if self._debug:
            print("Leaving Find Line State")

class State_StopRobot(State):
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)

    def enter(self):
        if self._debug:
            print("Entering Stop Robot State")
    
    def update(self):
        # state, _ = self.controller._robot.odom_update()
        # state.act_left_motor_speed = 0
        # state.act_right_motor_speed = 0
        self.controller._navigator.set_target((0, 0))
        return self
    
    def leave(self):
        if self._debug:
            print("Leaving Stop Robot State")

    
class State_GetOnLine(State):
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)
        self._align_timer = 0
        self._max_align_time = 15  # ~1.5 seconds
        
    def enter(self):
        self._align_timer = 0
        self.threshold = 20
        if self._debug:
            print("Entering Get On Line State")
        
    def update(self):
        self._align_timer += 1

        state, _ = self.controller._robot.odom_update()

        # self.controller._navigator.set_target((0.3, 0.3))
        
        # Get ground sensor readings
        l_sens = state.sens_ground_prox[0]
        r_sens = state.sens_ground_prox[2]

        error = abs(l_sens - r_sens)
        # print("error:", error)

        if self._align_timer > self._max_align_time:
            print("Taking too long to align")
            return self.transition_to(self.parent.s_find_line)
        
        # If line lost, go back to finding
        # if abs(l_sens - r_sens) >= self.threshold:
        #     print("Line not found")
        #     return self.transition_to(self.parent.s_find_line)
        # If sensors balanced, we're on the line
        if abs(error) < self.threshold:
            print("Line found")
            return self.transition_to(self.parent.s_stop_robot)
            # return self.transition_to(self.parent.s_follow_line)

        
            
        # Turn towards the brighter sensor
        turn_speed = MAX_SPEED * 0.4
        # state, _ = self.controller._robot.odom_update()
        
        if l_sens > r_sens:
            state.act_left_motor_speed = -turn_speed
            state.act_right_motor_speed = turn_speed
        else:
            state.act_left_motor_speed = turn_speed
            state.act_right_motor_speed = -turn_speed
            
        self.controller._robot._state = state
            
        # # If taking too long to align, go back to finding
        # if 
        #     return self.transition_to(self.parent.s_find_line)
            
        return self

    def leave(self):
        if self._debug:
            print("Leaving Get On Line State")

class State_FollowLine(State):
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)
        self._lost_line_counter = 0
        self._last_error = 0
        self._integral = 0
        
    def enter(self):
        self._last_error = 0
        self._integral = 0
        self._lost_line_counter = 0
        if self._debug:
            print("Entering Follow Line State")
        
    def update(self):
        # Get line position error
        l_sens = self.controller._robot._state.sens_ground_prox[0]
        r_sens = self.controller._robot._state.sens_ground_prox[2]
        error = l_sens - r_sens
        
        # Check if line is lost
        if abs(error) < 0.1:
            self._lost_line_counter += 1
            if self._lost_line_counter > 20:  # Lost for ~0.6 seconds
                return self.transition_to(self.parent.s_find_line)
        else:
            self._lost_line_counter = 0
            
        # PID control
        self._integral += error
        derivative = error - self._last_error
        self._last_error = error
        
        controller_output = (Kp * error) + (Ki * self._integral) + (Kd * derivative)
        
        # Set motor speeds
        left_speed = MAX_SPEED - controller_output
        right_speed = MAX_SPEED + controller_output
        
        state, _ = self.controller._robot.odom_update()
        state.act_left_motor_speed = left_speed
        state.act_right_motor_speed = right_speed
        self.controller._robot._state = state
        
        return self

    def leave(self):
        if self._debug:
            print("Leaving Follow Line State")

class PlannerFindFollowLine(Planner):
    def __init__(self):
        super().__init__()
        self._controller = None
        self._state = None
        
    def setup(self):
        # Create states
        self.s_find_line = State_FindLine(self._controller, self, debug=True)
        self.s_get_on_line = State_GetOnLine(self._controller, self, debug=True)
        self.s_stop_robot = State_StopRobot(self._controller, self, debug=True)
        # self.s_follow_line = State_FollowLine(self._controller, self)
        self._state = self.s_find_line  # Start with find line state
        self._state.enter()
        
    def update(self):
        # Get current state and update it
        # print('state =', self._state)
        self._state = self._state.update()
        return True
            
    def terminate(self):
        if self._state:
            self._state.leave()
        if self._controller:
            self._controller._robot._state.stop_all()
        self._controller = None
        self._state = None
    