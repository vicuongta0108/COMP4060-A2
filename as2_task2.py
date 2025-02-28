import math
import robot_epuck, planner_move_targets, navigator_diff_simple, controller

robot = robot_epuck.RobotEPuck("COM10")
targets = (  (200, 0.3, 0), (0, 0.1, math.pi/2),)
planner = planner_move_targets.PlannerMoveTargets( targets, 4 )
navigator = navigator_diff_simple.NavigatorDiffSimple()
controller = controller.Controller()

controller.setup(planner, navigator, robot)
controller.start()  # blocking until done
controller.terminate()

print("Controller exited")