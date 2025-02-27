from planner import Planner

MAX_SPEED = 154 # mm/s
Kp, Ki, Kd = 0.5, 0.01, 1.5 # PID tuning parameters

class PlannerLineFollow(Planner):
    def __init__(self):
        super().__init__()
        self._controller = None
        self._last_error = 0
        self.sens_ground_prox = None
        self.THRESHOLD = 0 # Adjust this value to change the threshold for the line sensor

    def setup(self):
        self._last_error = 0
        self.sens_ground_prox = self._controller._robot._state.sens_ground_prox
        self.THRESHOLD = 500

    def get_line_position(self):
        l_sens_ground = self.sens_ground_prox[0]
        c_sens_ground = self.sens_ground_prox[1]
        r_sens_ground = self.sens_ground_prox[2]

        left_black = l_sens_ground < self.THRESHOLD
        center_black = c_sens_ground < self.THRESHOLD
        right_black = r_sens_ground < self.THRESHOLD

        if center_black:
            error = 0
        elif left_black:
            error = l_sens_ground - c_sens_ground # Move left
        elif right_black:
            error = c_sens_ground - r_sens_ground # Move right
        else:
            error = 0

        return error
    
    def compute_PID(self, error):
        integral += self.get_line_position() # Accumulate the error
        derivative = error - self._last_error # Anticipate the next error
        self._last_error = error # Update the last error

        output = (Kp * error) + (Ki * integral) + (Kd * derivative)
        return output

    def update(self):
        error = self.get_line_position() # get the error
        controller_output = self.compute_PID(error) # Compute the PID controller output

        state, _ = self._controller._robot.odom_update() # Get the current state of the robot
        left_speed = MAX_SPEED - controller_output
        right_speed = MAX_SPEED + controller_output

        state.act_left_motor_speed = left_speed
        state.act_right_motor_speed = right_speed

        self._controller._robot._state = state

    def terminate(self):
        self._controller._robot._state.stop_all()  # Stop robot motors
        self._controller = None
        self._last_error = 0
        self.sens_ground_prox = None
