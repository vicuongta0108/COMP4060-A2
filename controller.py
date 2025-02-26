import time

class Controller:
    def __init__(self):
        self._planner = None
        self._navigator = None
        self._robot = None
        self._Hz = None
        self._running = False

    def setup(self, planner, navigator, robot, hz=10):
        # Save local pointers to these as public variables, and runs setup on each submodule
        self._planner = planner
        self._navigator = navigator
        self._robot = robot
        self._Hz = hz
        # Register a pointer to itself in each module so they can find the controller ???
        # Note: please confirm this is correct
        self._planner._controller = self
        self._navigator._controller = self
        self._robot._controller = self
        
        self._planner.setup()
        self._navigator.setup()
        self._robot.setup()

    # Run the main program loop
    def start(self):
        loop_time = 1.0 / self._Hz
        running = True
        while running:
            # Call the update functions of the other modules in each iteration
            if not self._planner.update():
                running = False
            self._navigator.update()
            self._robot.update()
            # Sleep to maintain the target Hz
            elapsed_time = time.time() - loop_time
            sleep_time = max(0.001, elapsed_time)
            # print(loop_time)
            time.sleep(loop_time)

    # Clean up and call terminate on the submodules
    def terminate(self):
        self._planner.terminate()
        self._navigator.terminate()
        self._robot.terminate()