from planner import Planner
import time

Kp, Ki, Kd = 0.3, 0.17, 0.2# PID tuning parameters

class PlannerLineFollow(Planner):
    def __init__(self):
        super().__init__()
        self._controller = None
        self._last_error = 0
        # self._base_speed = 20
        self.sens_ground_prox = None
        self._last_error_time = 0
        self.THRESHOLD = 2 # Adjust this value to change the threshold for the line sensor

    def setup(self):
        self._last_error = 0
        self._base_speed = 40
        # self.sens_ground_prox = self._controller._robot._state.
        self.THRESHOLD = 0
        self._integral = 0
        self._last_error_time = time.time()

    def get_line_position(self):
        l_sens_ground = self.sens_ground_prox[0]
        r_sens_ground = self.sens_ground_prox[2]

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
        
        self.sens_ground_prox = self._controller._robot._state.sens_ground_prox
        error = self.get_line_position() # get the error
        controller_output = self.compute_PID(error) # Compute the PID controller output

        left_speed = self._base_speed + controller_output
        right_speed = self._base_speed - controller_output
        # print((left_speed, right_speed))

        self._controller._navigator.set_target((left_speed / 100, right_speed / 100))
        return True

    def terminate(self):
        self._controller._robot._state.stop_all()  # Stop robot motors
        self._controller = None
        self._last_error = 0
        self.sens_ground_prox = None
