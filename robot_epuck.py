from robot import Robot
from epucklib.epuck_com import EPuckCom
import time

class RobotEPuck(Robot):
    def __init__(self, com_port: str):
        super().__init__()
        if not isinstance(com_port, str):
            raise TypeError("COM_PORT must be a string")
        else:
            self.com_port = com_port
            self.epuckcomm = None

    def setup(self):
        self.epuckcomm = EPuckCom(self.com_port, debug=False)

        if not self.epuckcomm.connect():
            print("Could not connect, quitting")
            return

        self.epuckcomm.enable_sensors = True

    def update(self):
        self.epuckcomm.send_command()
        print("Giving time for robot to get ready...")
        time.sleep(0.5)
        print("Robot is ready!")

    def terminate(self):
        self.epuckcomm.stop_all()
        self.epuckcomm.close()

