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
        # print("error in find line:", abs(l_sens - r_sens))

        print("l_sens:", l_sens)
        print("r_sens:", r_sens)
        print("abs(l_sens - r_sens):", abs(l_sens - r_sens))
        if (l_sens < 500 or r_sens < 500) and abs(l_sens - r_sens) < 100:
            return self.transition_to(self.parent.s_perpendicular_line)
        
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
            # self.controller._navigator.set_target((0.4, 0.05))
            self.controller._navigator.set_target((0.2, 0.2))
        else:
            self.controller._navigator.set_target((0.2, 0.2))
            # self.controller._navigator.set_target((0.05, 0.4))
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
        if l_sens < 100 and r_sens < 100:
            return self.transition_to(self.parent.s_perpendicular_line)
        
        if abs(error) < self.threshold:
            print("Line found")
            return self.transition_to(self.parent.s_follow_line)
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

# class State_MoveBacAfterPerpendicular(State):
#     def __init__(self, controller, parent, debug=True):
#         super().__init__(controller, parent, debug)
#         self._align_timer = 0
#         self._max_align_time = 15  # ~1.5 seconds
        
#     def enter(self):
#         self._align_timer = 0
#         self.threshold = 20
#         if self._debug:
#             print("Entering Get On Line State")
        
#     def update(self):
        
#         self.controller._navigator.set_target((-0.1, -0.2))


#         return self.transition_to(self.parent.find_line)

#     def leave(self):
#         if self._debug:
#             print("Leaving Get On Line State")

class State_PerpendicularLine(State):

    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)
        self._align_timer = 0
        self._max_align_time = 20  # ~1.5 seconds

    def enter(self):
        self._align_timer = 0
        self.threshold = 30
        if self._debug:
            print("Entering Get On Line Perpendicular State")
    
    def update(self):
        self.controller._navigator.set_target((-0.1, -0.2))
        
        if self._align_timer > self._max_align_time:
            # print("Taking too long to align")
            return self.transition_to(self.parent.s_find_line)

        self._align_timer += 1
        return self
    
    def leave(self):
        if self._debug:
            print("Leaving Get On Line State")


class State_FollowLine(State):
    Kp, Ki, Kd = 0.02, 0.0017, 0.0015 # PID tuning parameters for 20% speed
# Kp, Ki, Kd = 0.0408, 0.0030, 0.0033 # 0.0037, 0.0005 # PID tuning parameters for 30% speed
    def __init__(self, self_controller, parent, debug=True):
        super().__init__(self_controller, parent, debug)
        self._last_error = 0
        self.sens_ground_prox = None
        self._last_error_time = 0
        self._state = None
        # self.THRESHOLD = 2 # Adjust this value to change the threshold for the line sensor

    def enter(self):
        self._last_error = 0
        self._base_speed = 20 # % speed
        # self.THRESHOLD = 0
        self._integral = 0
        self._last_error_time = time.time()
        self._state = None

    def get_line_position(self):
        l_sens_ground = self.sens_ground_prox[0]
        r_sens_ground = self.sens_ground_prox[2]

        print("l_sens_ground:", l_sens_ground, "r_sens_ground:", r_sens_ground)
    

        error = l_sens_ground - r_sens_ground
        # if error < self.THRESHOLD:
            # error = 0

        # print(error)
        return error
    
    def compute_PID(self, error):
        current_time = time.time()
        self._integral += error * (current_time - self._last_error_time) # Accumulate the error
        derivative = (error - self._last_error) / (current_time - self._last_error_time) # Anticipate the next error
        self._last_error = error # Update the last error
        self._last_error_time = current_time
        output = (Kp * error) + (Ki * self._integral) + (Kd * derivative)
        return output

    def update(self):
        
        self._state, _ = self.controller._robot.odom_update() # Get the current state of the robot
        self.sens_ground_prox = self._state.sens_ground_prox
        error = self.get_line_position() # get the error
        controller_output = self.compute_PID(error) # Compute the PID controller output
        print("error:", error, "controller_output:", controller_output)
        left_speed = self._base_speed + controller_output
        right_speed = self._base_speed - controller_output

        print("left_speed:", left_speed / 100, "right_speed:", right_speed / 100)

        self.controller._navigator.set_target((left_speed / 100, right_speed / 100))
        return self

    def leave(self):
        self.controller._robot._state.stop_all()  # Stop robot motors
        self.controller = None
        self._last_error = 0
        self.sens_ground_prox = None


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
        self.s_follow_line = State_FollowLine(self._controller, self)
        self.s_perpendicular_line = State_PerpendicularLine(self._controller, self)
        self._state = self.s_follow_line  # Start with find line state
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
    