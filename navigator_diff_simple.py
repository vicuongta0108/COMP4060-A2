from navigator import Navigator
from robot import Robot
from epuck_lib import diff_drive_forward_kin, diff_drive_inverse_kin, move_steps

MAX_SPEED = 1000
MIN_SPEED = 500

class NavigatorDiffSimple(Navigator):
    def __init__(self):
        super().__init__()
        self.robot = None
        self.moved_mm = None
        self.target_mm = None

    def setup(self, com_port):
        self.robot = Robot(com_port)
        self.moved_mm = 0
        self.target_mm = 0
        print("Navigator finish setup!")

    def update(self):
        # Update Odometry
        self.moved_mm += self.robot.odom_update()
        # Tell the robot to move
        speed = (1 - self.moved_mm / self.target_mm) * (MAX_SPEED - MIN_SPEED) + MIN_SPEED
        self.robot.state.act_left_motor_speed = speed
        self.robot.state.act_right_motor_speed = speed
        # Monitor how much the robot has moved
        


    def terminate(self):
        self.robot.stop_all()
        self.robot.close()
        print("Navigator terminated")

    # Use the given target tuple and A1 kinematics to solve how the robot should move.
    def set_target(self, target):
        self.target_mm = diff_drive_inverse_kin(*target)