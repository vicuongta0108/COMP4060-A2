import math
import robot_epuck, planner_linefollow, navigator_diff_direct_percent, controller

robot = robot_epuck.RobotEPuck("COM11")
targets = (0.2, 0.2)
planner = planner_linefollow.PlannerLineFollow()
navigator = navigator_diff_direct_percent.NavigatorDiffDirectPercent(targets)
controller = controller.Controller()

try:
    controller.setup(planner, navigator, robot)
    controller.start()  # blocking until done
    controller.terminate()
except KeyboardInterrupt:
    controller.terminate()

print("Controller exited")