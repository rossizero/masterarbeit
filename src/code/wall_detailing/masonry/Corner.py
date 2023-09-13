import math

import numpy as np
import quaternion


class Line:
    """
    A line defined by two points
    """
    def __init__(self, p1: np.array, p2: np.array):
        self.p1 = p1
        self.p2 = p2

    def on_line(self, point: np.array, between: bool = True):
        """
        :param point: the point to check
        :param between: if the point should be between the two points of the line
        """
        diff12 = self.p1 - self.p2
        diff1p = self.p1 - point
        diffp2 = point - self.p2

        a = np.linalg.norm(diff12)
        b = np.linalg.norm(diff1p)
        c = np.linalg.norm(diffp2)
        between = a**2 + b**2 >= c**2 and a**2 + c**2 >= b**2 if between else True

        if np.allclose(diffp2, 0) or np.allclose(diff1p, 0):
            return True
        d = np.linalg.norm(np.cross(diff12, diff1p)) / np.linalg.norm(diffp2)
        return d < 1e-9 and between

    def length(self):
        return np.linalg.norm(self.p1 - self.p2)


class Corner:
    def __init__(self, bottom_point: np.array, top_point: np.array):
        self.line = Line(bottom_point, top_point)
        self.walls = set()

    def get_rotation(self) -> np.quaternion:
        ret = np.quaternion(1, 0, 0, 0)
        if len(self.walls) == 2:
            r1 = list(self.walls)[0].get_rotation()
            r2 = list(self.walls)[1].get_rotation()
            diff = r2 * r1.inverse()
            angle = round(diff.angle(), 6)
            ret = quaternion.from_euler_angles(0, 0, math.pi + angle)
        return ret

    def __eq__(self, other: "Corner"):
        return self.line.on_line(other.line.p1) and self.line.on_line(other.line.p2)


class Corners:
    def __init__(self):
        self.corners = []

    def add_corner(self, corner: Corner):
        for c in self.corners:
            if c == corner:
                c.walls.update(corner.walls)
                return
        self.corners.append(corner)

