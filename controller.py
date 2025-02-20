import time

class Controller:
    def __init__(self, planner, navigator, robot, hz=10):
        self.planner = planner
        self.navigator = navigator
        self.robot = robot
        self._Hz = hz
        self._running = False

    def setup(self, planner, navigator, robot, hz=10):
        # Save local pointers to these as public variables, and runs setup on each submodule
        self.planner = planner.setup()
        self.navigator = navigator.setup()
        self.robot = robot.setup()
        self._Hz = hz
        # Register a pointer to itself in each module so they can find the controller
        self.planner.controller = self
        self.navigator.controller = self
        self.robot.controller = self

    # Run the main program loop
    def start(self):
        loop_time = 1.0 / self._Hz
        while self._running:
            # Call the update functions of the other modules in each iteration
            if not self.planner.update():
                self._running = False
            self.planner.update()
            self.navigator.update()
            self.robot.update()
            # Sleep to maintain the target Hz
            elapsed_time = time.time() - loop_time
            sleep_time = max(1.0 / self._Hz, loop_time - elapsed_time)
            time.sleep(sleep_time)

    # Clean up and call terminate on the submodules
    def terminate(self):
        self.planner.terminate()
        self.navigator.terminate()
        self.robot.terminate()