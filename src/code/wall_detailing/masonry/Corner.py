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

    def distance_to_line(self, point: np.array):
        """
        :return: distance to line
        """
        l = self.p2 - self.p1
        p = point - self.p1
        projection_distance = np.dot(p, l) / np.dot(l, l)  # squared length of l
        projected_point = self.p1 + projection_distance * l
        d = np.linalg.norm(point - projected_point)
        return d

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


class Corn:
    def __init__(self, point: np.array):
        self.point = point
        self.layers = set()

    def __eq__(self, other: "Corn"):
        return np.allclose(self.point, other.point)

    def get_main_layer(self):
        """
        :return: the (or a) layer in which this corner point lies in
        """
        # we want to always get the same main layer for any set of layers of the same two walls
        # so in case there are two layers, in which the corner point lies inside (aka the layers overlap) we
        # would get a random main layer (depending on the order of the set) every time. By sorting them (operator has
        # been overridden) we prevent that
        li = list(self.layers)
        li.sort()

        for l in li:
            main = Line(l.left_edge, l.right_edge).on_line(self.point, between=True)
            if main:
                return l

        # Prevents an error in which none of the wall is actually the main wall.
        # Indicates that mistakes have been made earlier (most possibly while modeling).
        assert False

    def get_rotation(self) -> np.quaternion:
        ret = np.quaternion(1, 0, 0, 0)
        if len(self.layers) == 2:
            # the wall we "place" the corner onto

            main_layer = self.get_main_layer()
            l1 = list(self.layers)[0]
            l2 = list(self.layers)[1]

            # mid of both walls
            m1 = l1.center
            m2 = l2.center

            # lower coordinate of corner
            c1 = self.point

            # unit vector from corner to mid
            wall_orientation_1 = m1 - c1
            wall_orientation_2 = m2 - c1
            wall_orientation_1 = wall_orientation_1 / np.linalg.norm(wall_orientation_1)
            wall_orientation_2 = wall_orientation_2 / np.linalg.norm(wall_orientation_2)

            # mid of those vectors -> the point in the corner between both walls and 45 degrees from both walls
            mid = (wall_orientation_1 + wall_orientation_2) / 2

            # normalise rotation
            mid = quaternion.rotate_vectors(main_layer.parent.get_rotation().inverse(), mid)
            z_rot = np.arctan2(mid[1], mid[0])
            # subtract the 45 degrees from above
            ret = quaternion.from_euler_angles(0, 0, z_rot - math.pi/4.0)
        return ret


class Corns:
    def __init__(self):
        self.corners = []

    def add_corner(self, corner: Corn):
        for c in self.corners:
            if c == corner:
                c.walls.update(corner.layers)
                return
        self.corners.append(corner)
