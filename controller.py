import time

class Controller:
    def __init__(self, planner, navigator, controller, hz=10):
        self.planner = planner
        self.navigator = navigator
        self.controller = controller
        self.Hz = hz


