from epucklib.state import State
from abc import ABC
import random
import time
import numpy as np
from planner import Planner
import math

TIME_OUT = 5 # seconds

# Constants from planner_linefollow.py
MAX_SPEED = 154  # mm/s
Kp, Ki, Kd = 0.5, 0.01, 1.5  # PID tuning parameters

class PlannerFindFollowLine(Planner):
    def __init__(self):
        super().__init__()
        self._controller = None
        self._last_error = 0
        self._integral = 0
        self.sens_ground_prox = None
        
        # Create states
        self._states = {
            "s_find_line": FindLineState(self),
            "s_get_on_line": GetOnLineState(self),
            "s_follow_line": FollowLineState(self)
        }
        self._current_state = "s_find_line"
        
    def setup(self):
        self._last_error = 0
        self._integral = 0
        self.sens_ground_prox = self._controller._robot._state.sens_ground_prox
        
    def get_line_position(self):
        l_sens_ground = self.sens_ground_prox[0]
        r_sens_ground = self.sens_ground_prox[2]
        error = l_sens_ground - r_sens_ground
        return error
    
    def compute_PID(self, error):
        self._integral += error
        derivative = error - self._last_error
        self._last_error = error
        
        output = (Kp * error) + (Ki * self._integral) + (Kd * derivative)
        return output
        
    def set_motor_speeds(self, left_speed, right_speed):
        state, _ = self._controller._robot.odom_update()
        state.act_left_motor_speed = left_speed
        state.act_right_motor_speed = right_speed
        self._controller._robot._state = state
        
    def update(self):
        # Get current state and update it
        current = self._states[self._current_state]
        next_state = current.update()
        
        # Handle state transition if needed
        if next_state and next_state in self._states:
            current.exit()
            self._current_state = next_state
            self._states[next_state].enter()
            
    def terminate(self):
        self._controller._robot._state.stop_all()
        self._controller = None
        self._last_error = 0
        self._integral = 0
        self.sens_ground_prox = None

class FindLineState(State):
    def __init__(self, planner):
        super().__init__()
        self._planner = planner
        self._search_timer = 0
        self._direction = 1  # 1 for right, -1 for left
        
    def enter(self):
        self._search_timer = 0
        # Randomly choose initial search direction
        self._direction = random.choice([-1, 1])
        
    def update(self):
        self._search_timer += 1
        
        # Get ground sensor readings
        l_sens = self._planner.sens_ground_prox[0]
        r_sens = self._planner.sens_ground_prox[2]
        
        # If line detected, transition to get_on_line
        if abs(l_sens - r_sens) > 0.2:
            return "s_get_on_line"
            
        # Arc search pattern
        inner_speed = MAX_SPEED * 0.3
        outer_speed = MAX_SPEED * 0.7
        
        # Switch direction every ~3 seconds to expand search
        if self._search_timer > 100:
            self._direction *= -1
            self._search_timer = 0
            
        if self._direction > 0:
            self._planner.set_motor_speeds(outer_speed, inner_speed)
        else:
            self._planner.set_motor_speeds(inner_speed, outer_speed)
            
        return None

class GetOnLineState(State):
    def __init__(self, planner):
        super().__init__()
        self._planner = planner
        self._align_timer = 0
        self._max_align_time = 50  # ~1.5 seconds
        
    def enter(self):
        self._align_timer = 0
        
    def update(self):
        self._align_timer += 1
        
        # Get ground sensor readings
        l_sens = self._planner.sens_ground_prox[0]
        r_sens = self._planner.sens_ground_prox[2]
        
        # If line lost, go back to finding
        if abs(l_sens - r_sens) < 0.1:
            return "s_find_line"
            
        # If sensors balanced, we're on the line
        if abs(l_sens - r_sens) < 0.05:
            return "s_follow_line"
            
        # Turn towards the brighter sensor
        turn_speed = MAX_SPEED * 0.4
        if l_sens > r_sens:
            self._planner.set_motor_speeds(-turn_speed, turn_speed)
        else:
            self._planner.set_motor_speeds(turn_speed, -turn_speed)
            
        # If taking too long to align, go back to finding
        if self._align_timer > self._max_align_time:
            return "s_find_line"
            
        return None

class FollowLineState(State):
    def __init__(self, planner):
        super().__init__()
        self._planner = planner
        self._lost_line_counter = 0
        
    def enter(self):
        self._planner._last_error = 0
        self._planner._integral = 0
        self._lost_line_counter = 0
        
    def update(self):
        # Get line position error
        error = self._planner.get_line_position()
        
        # Check if line is lost
        if abs(error) < 0.1:
            self._lost_line_counter += 1
            if self._lost_line_counter > 20:  # Lost for ~0.6 seconds
                return "s_find_line"
        else:
            self._lost_line_counter = 0
            
        # Use PID controller
        controller_output = self._planner.compute_PID(error)
        
        # Set motor speeds
        left_speed = MAX_SPEED - controller_output
        right_speed = MAX_SPEED + controller_output
        
        self._planner.set_motor_speeds(left_speed, right_speed)
        return None

    def leave(self):
        print("Leaving Follow Line State")
    