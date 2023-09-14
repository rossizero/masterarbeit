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
        :return: min distance to line
        """
        diff12 = self.p1 - self.p2
        diff1p = self.p1 - point
        diffp2 = point - self.p2
        d = np.linalg.norm(np.cross(diff12, diff1p)) / np.linalg.norm(diffp2)
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


class Corner:
    def __init__(self, bottom_point: np.array, top_point: np.array):
        self.line = Line(bottom_point, top_point)
        self.walls = set()

    def get_main_wall(self):
        """
        :return: the (or a) wall in which this corner lies in
        """
        # TODO what if 1, 3 or more walls
        wall1 = list(self.walls)[0]
        wall2 = list(self.walls)[1]
        d11 = Line(wall1.get_corners()[0], wall1.get_corners()[2]).on_line(self.line.p1)
        d21 = Line(wall1.get_corners()[0], wall1.get_corners()[2]).on_line(self.line.p2)
        d31 = Line(wall1.get_corners()[1], wall1.get_corners()[3]).on_line(self.line.p1)
        d41 = Line(wall1.get_corners()[1], wall1.get_corners()[3]).on_line(self.line.p2)
        w1 = d11 or d21 or d31 or d41

        d12 = Line(wall2.get_corners()[0], wall2.get_corners()[2]).on_line(self.line.p1)
        d22 = Line(wall2.get_corners()[0], wall2.get_corners()[2]).on_line(self.line.p2)
        d32 = Line(wall2.get_corners()[1], wall2.get_corners()[3]).on_line(self.line.p1)
        d42 = Line(wall2.get_corners()[1], wall2.get_corners()[3]).on_line(self.line.p2)
        w2 = d12 or d22 or d32 or d42

        if w1 and w1:
            return wall1
        elif w1:
            return wall1
        elif w2:
            return wall2
        else:
            return None

    def get_rotation(self) -> np.quaternion:
        ret = np.quaternion(1, 0, 0, 0)
        if len(self.walls) == 2:
            w1 = list(self.walls)[0]
            w2 = list(self.walls)[1]
            r1 = list(self.walls)[0].get_rotation()
            r2 = list(self.walls)[1].get_rotation()
            diff = r2 * r1.inverse()
            angle = round(diff.angle(), 6)

            z_part1 = quaternion.rotate_vectors(r1, np.array([0.0, 0.0, 1.0]))
            z_part1 = z_part1 / np.linalg.norm(z_part1)

            dddd = self.line.p2 - self.line.p1
            dddd = dddd / np.linalg.norm(dddd)

            # z direction of the corner == z direction of the wall
            assert np.allclose(z_part1, dddd, atol=1e-6), f"{z_part1} != {dddd}"

            t = np.round(z_part1 - dddd, 6)

            m1 = w1.get_location(z_offset=-w1.height / 2.0)
            m2 = w2.get_location(z_offset=-w2.height / 2.0)
            m = self.line.p1 - (m1 + m2) / 2.0


            m1 = self.line.p1 - m1
            m2 = self.line.p1 - m2

            x_part1 = quaternion.rotate_vectors(r1, np.array([1.0, 0.0, 0.0]))
            x_part1 = self.line.p1 + x_part1 / np.linalg.norm(x_part1)
            m = m / np.linalg.norm(m)

            #x_part2 = self.line.p1 - quaternion.rotate_vectors(r2, np.array([1.0, 0.0, 0.0]))
            #
            #x_part2 = x_part2 / np.linalg.norm(x_part2)

            lalala = np.dot(x_part1, m) / (np.linalg.norm(x_part1) * np.linalg.norm(m))
            lelele = lalala
            lalala = np.arccos(lalala)
            lululu = math.degrees(lalala)

            ret = quaternion.from_euler_angles(0, 0, math.pi)
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

