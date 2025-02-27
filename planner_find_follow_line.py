from epucklib.state import State
from abc import ABC
import random
import time
import numpy as np

MAX_SPEED = 154 # mm/s
TIME_OUT = 5 # seconds

class PlannerFindFollowLine(State):
    def __init__(self):
        super().__init__()
        self.s_find_line = State_FindLine(self.controller, self)
        self.s_get_on_line = State_GetOnLine(self.controller, self)
        self.s_follow_line = State_FollowLine(self.controller, self)
        self._state = self.s_find_line # Start with this state

    def update(self):
        self._state = self._state.update() # Update the state
        self.transition_to(self.parent.s_follow_line) # Transition to the next state

class State_FindLine(ABC):
    _debug = False
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)
        
    def enter(self):  # called every time state is entered.
        print("Entering Find Line State")
        # Make robot move randomly to find the line
        state, _ = self.controller.robot.odom_update()
        speed = random.randint(-MAX_SPEED, MAX_SPEED)
        state.act_left_motor_speed = speed
        state.act_right_motor_speed = speed
        
    # called every tick, a state transition to itself
    # manage the navigator and access robot state in here.
    def update(self):  
        l_sens_ground = self.controller.robot.state.sens_ground_prox[0]
        r_sens_ground = self.controller.robot.state.sens.sens_ground_prox[2]
        # Check if the line is found (check sensors values and replace 500, I put 500 here as an example)
        if l_sens_ground < 500 and r_sens_ground < 500:
            return self.transition_to(self.parent.s_get_on_line)
        
        return self # Return the current state

    def leave(self):   #called when state is exited.
        print("Leaving Find Line State")

class State_FollowLine(State):
    _debug = False
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)

    def enter(self):
        print("Entering Follow Line State")

    def update(self):
        # Get the line position
        error = self.controller.get_line_position()
        # Compute the PID controller output
        controller_output = self.controller.compute_PID(error)
        # Set motor speeds
        left_speed = MAX_SPEED - controller_output
        right_speed = MAX_SPEED + controller_output
        state, _ = self.controller.robot.odom_update()
        state.act_left_motor_speed = left_speed
        state.act_right_motor_speed = right_speed

        # Need a condition to transition back to find line state

        return self # Return the current state

    def leave(self):
        print("Leaving Follow Line State")

class State_GetOnLine(State):
    _debug = False
    def __init__(self, controller, parent, debug=True):
        super().__init__(controller, parent, debug)
        self._start_time = time.time()
    
    def enter(self):
        print("Entering Get On Line State")
        self._start_time = time.time()

    def update(self):
        # Return to find line state if time out
        if time.time() - self._start_time > TIME_OUT:
            return self.transition_to(self.parent.s_find_line)
        
        return self # Return the current state
        

    def leave(self):
        print("Leaving Get On Line State")
    