import math
import robot_epuck, planner_move_targets, navigator_diff_simple, controller

robot = robot_epuck.RobotEPuck("COM10")
targets = ((200, 1, 0), (0, .2, math.pi/2))
planner = planner_move_targets.PlannerMoveTargets( targets, 2 )
navigator = navigator_diff_simple.NavigatorDiffSimple()
controller = controller.Controller()

controller.setup(planner, navigator, robot)
controller.start()  # blocking until done
controller.terminate()

print("Controller exited")