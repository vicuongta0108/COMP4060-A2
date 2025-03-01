from robot import Robot
from epucklib.epuck_com import EPuckCom
from epuck_lib import steps_delta, diff_drive_forward_kin
import sys

class RobotEPuck(Robot):
    def __init__(self, com_port: str):
        super().__init__(com_port)
        self._epuckcomm = None
        self._controller = None
        self._robot_pose = None
        self._com_port = com_port
        self._state = None

    def get_epuckcomm(self):
        self._epuckcomm = EPuckCom(self._com_port, debug=False)
        if not self._epuckcomm.connect():
            print("Cannot connect to robot. Quitting :(")
            sys.exit(0)

        return self._epuckcomm

    def setup(self):
        self._epuckcomm = self.get_epuckcomm()

        if not self._epuckcomm:
            return

        self._epuckcomm.enable_sensors = True
        self._epuckcomm.data_update()
        self._state = self._epuckcomm.state
        self._robot_pose = (0.0, 0.0, 0.0)

    def update(self):
        self._epuckcomm.state = self._state
        # print('Left motor speeds: ', self._state.act_left_motor_speed)
        # print('Right motor speeds: ', self._state.act_right_motor_speed)
        # print('Right motor steps: ', self._state.sens_right_motor_steps)
        # print(self._state)
        self._epuckcomm.send_command()

    def terminate(self):
        self._epuckcomm.stop_all()
        self._epuckcomm.close()
        self._epuckcomm = None

    def odom_reset(self):
        self.robot_pose = (0.0, 0.0, 0.0)

    def odom_update(self):
        # Get the last steps 
        last_l_step = self._epuckcomm.state.sens_left_motor_steps
        last_r_step = self._epuckcomm.state.sens_right_motor_steps
        # Update the robot state
        self._epuckcomm.data_update()
        self._state = self._epuckcomm.state
        # Calculate the steps moved
        left_steps = steps_delta(last_l_step, self._state.sens_left_motor_steps)
        right_steps = steps_delta(last_r_step, self._state.sens_right_motor_steps)
        self._robot_pose = diff_drive_forward_kin(self._robot_pose, left_steps, right_steps) # update new pose
        # Return the updated state and robot pose
        return self._state, self._robot_pose