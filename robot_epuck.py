from robot import Robot
from epucklib.epuck_com import EPuckCom
from epucklib.epuck_state import EPuckState
from epuck_lib import calc_and_print_pose
import time

class RobotEPuck(Robot):
    def __init__(self, com_port: str):
        super().__init__()
        if not isinstance(com_port, str):
            raise TypeError("COM_PORT must be a string")
        else:
            self.com_port = com_port
            self.epuckcomm = None
            self.state = EPuckState()

    def setup(self):
        self.epuckcomm = EPuckCom(self.com_port, debug=False)

        if not self.epuckcomm.connect():
            print("Could not connect, quitting")
            return

        self.epuckcomm.enable_sensors = True

        return self.epuckcomm

    def update(self):
        self.epuckcomm.send_command()
        print("Giving time for robot to get ready...")
        time.sleep(0.5)
        print("Robot is ready!")

    def terminate(self):
        self.epuckcomm.stop_all()
        self.epuckcomm.close()

    def odom_reset(self):
        self.robot_pos = (0, 0, 0)

    def odom_update(self):
        current_pose = self.robot_pos
        r_last_left_steps = self.epuckcomm.state.sens_left_motor_steps
        r_last_right_steps = self.epuckcomm.state.sens_right_motor_steps
        # Calculate new robot pose 
        self.robot_pos = calc_and_print_pose(current_pose, r_last_left_steps, r_last_right_steps)[0] # get the new pose only
        return self.robot_pos
