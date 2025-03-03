import math
import robot_epuck, planner_move_once, navigator_diff_simple, controller

robot = robot_epuck.RobotEPuck("COM10")
planner = planner_move_once.PlannerMoveOnce( (0, 50, 2* math.pi) )
navigator = navigator_diff_simple.NavigatorDiffSimple()
controller = controller.Controller()

controller.setup(planner, navigator, robot)
controller.start()  # blocking until done
controller.terminate()

print("Controller exited")