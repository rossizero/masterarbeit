from typing import Optional

import numpy as np


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

    def on_line(self, point: np.array, between: bool = True, tolerance: float = 1e-9):
        """
        :param point: the point to check
        :param between: if the point should be between the two points of the line
        """
        diff12 = self.p1 - self.p2
        diff1p = self.p1 - point
        diffp2 = point - self.p2

        a = np.linalg.norm(diff12) + tolerance  # TODO check
        b = np.linalg.norm(diff1p)
        c = np.linalg.norm(diffp2)
        between = a**2 + b**2 >= c**2 and a**2 + c**2 >= b**2 if between else True

        if np.allclose(diffp2, 0) or np.allclose(diff1p, 0):
            return True
        d = np.linalg.norm(np.cross(diff12, diff1p)) / np.linalg.norm(diffp2)
        return d < 1e-9 and between

    def length(self):
        """
        length between this lines p1 and p2
        :return: length between this lines p1 and p2
        """
        return np.linalg.norm(self.p1 - self.p2)

    def intersection(self, other: 'Line') -> Optional[np.array]:
        """
        Calculates the intersection of two lines
        :param other: the other line
        :return: Intersection between the two lines
        """
        p1 = self.p1
        p2 = other.p1

        direction1 = (self.p1 - self.p2) / np.linalg.norm(self.p1 - self.p2)
        direction2 = (other.p1 - other.p2) / np.linalg.norm(other.p1 - other.p2)

        A = np.vstack((direction1, -direction2, [1, 1, 1])).T
        b_bottom = p2 - p1

        try:
            t = np.linalg.solve(A, b_bottom)

            # Calculate the intersection points on both lines
            intersection_point1 = p1 + t[0] * direction1
            intersection_point2 = p2 + t[1] * direction2
            assert np.allclose(intersection_point1, intersection_point2)

        except np.linalg.LinAlgError:  # No solution
            return None
        except AssertionError:  # Weird shit
            return None
        return intersection_point1
