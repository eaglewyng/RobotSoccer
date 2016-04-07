import math

class Velocity:
    def __init__(self, x = 0.0, y = 0.0):
        self.x = x
        self.y = y

    def magnitude(self):
        return math.sqrt(self.x**2 + self.y**2)

