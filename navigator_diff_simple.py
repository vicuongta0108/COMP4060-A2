# Takes a target movement and converts this into robot motion commands.
from navigator import Navigator
from epuck_lib import move_steps, diff_drive_inverse_kin

class NavigatorDiffSimple(Navigator):
    def __init__(self):
        super().__init__()

    def setup(self):
        print("NavigatorDiffSimple.setup()")

    def update(self):
        print("NavigatorDiffSimple.update()")

    def terminate(self, epuckcomm):
        epuckcomm.stop_all()
        epuckcomm.close()
        print("Robot stopped!")

    def set_target(self, epuckcomm, target):
        move_steps(epuckcomm, *target)


