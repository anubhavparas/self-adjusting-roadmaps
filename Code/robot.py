class Robot:
    def __init__(self, radius, clearance):
        self.radius = radius
        self.clearance = clearance

class TurtleBot(Robot):
    def __init__(self, radius, clearance, wheel_rad, dist_bet_wheels):
        super().__init__(radius, clearance)
        self.wheel_rad = wheel_rad
        self.dist_bet_wheels = dist_bet_wheels