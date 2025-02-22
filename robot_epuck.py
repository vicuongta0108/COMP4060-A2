from robot import Robot
from epucklib.epuck_com import EPuckCom
from epuck_lib import steps_delta, diff_drive_forward_kin

class RobotEPuck(Robot):
    def __init__(self, com_port: str):
        super().__init__(com_port)
        self.epuckcomm = None

    def get_epuckcomm(self):
        epuckcomm = EPuckCom(self.com_port, debug=False)
        if not self.epuckcomm.connect():
            print("Cannot connect to robot. Quitting :(")
            return None

        return epuckcomm

    def setup(self):
        self.epuckcomm = self.get_epuckcomm()

        if not self.epuckcomm:
            return

        self.epuckcomm.enable_sensors = True

    def update(self):
        self.epuckcomm.send_command()

    def terminate(self):
        self.epuckcomm.stop_all()
        self.epuckcomm.close()

    def odom_reset(self):
        self.robot_pose = (0.0, 0.0, 0.0)

    def odom_update(self):
        last_l_step = self.state.sens_left_motor_steps
        last_r_step = self.state.sens_right_motor_steps

        left_steps = steps_delta(last_l_step, self.state.sens_left_motor_steps)
        right_steps = steps_delta(last_r_step, self.state.sens_right_motor_steps)
        self.robot_pose = diff_drive_forward_kin(self.robot_pose, left_steps, right_steps) # update new pose

        return self.robot_pose