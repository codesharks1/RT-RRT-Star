import math
class Vector:
    def __init__(self, x, y):
        self._x = x
        self._y = y

    def x(self):
        return self._x

    def y(self):
        return self._y

    def scalarProduct(self, a):
        return Vector(a * self.x(), a * self.y())

    def norm(self):
        return math.sqrt(self.squaredLength())

    def squaredLength(self):
        return self.x() ** 2 + self.y() ** 2