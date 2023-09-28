import math
from typing import List

import numpy as np
import quaternion

from wall_detailing.detailing.wall_layer import WallLayer
from wall_detailing.detailing.wall_layer_group import WallLayerGroup


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
    """
    A class to store information about two layers that form a corner
    """
    def __init__(self, point: np.array):
        self.point = point  # center of the corner
        self.layers = set()  # layers that form the corner
        self.plan_offset = 0

    def __eq__(self, other: "Corn"):
        return np.allclose(self.point, other.point)

    def get_main_layer(self):
        """
        The main layer is considered to be the or one layer in which the corner point lies in.
        Its rotation is being used for calculations.
        :return: the main layer
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
        """
        How do we have to rotate the Corner Plan of a bond to lie above the axis the layers form
        :return: rotation we have to apply to the corner plan of a bond
        """
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
    """
    A class that handles a list of corners.
    If we find a corner that already exists, we simply add the layers to the existing layers that already
    form the corner. (T- Joints or Crossings can be made possible this way)
    """
    def __init__(self):
        self.corners = []

    def add_corner(self, corner: Corn):
        for c in self.corners:
            if c == corner:
                c.walls.update(corner.layers)
                return
        self.corners.append(corner)


def check_for_corners(wall_layer_groups: List[WallLayerGroup]) -> Corns:
    corners = Corns()

    for i, w1 in enumerate(wall_layer_groups):
        for j in range(i+1, len(wall_layer_groups)):
            w2 = wall_layer_groups[j]

            r1 = w1.get_rotation()
            r2 = w2.get_rotation()
            diff = r2 * r1.inverse()
            angle = round(diff.angle(), 6)

            # check if rotation of wall leads to parallel corners
            z_part1 = quaternion.rotate_vectors(r1, np.array([0.0, 0.0, 1.0]))
            z_part2 = quaternion.rotate_vectors(r2, np.array([0.0, 0.0, 1.0]))
            z_parallel = np.isclose(abs(np.dot(z_part1, z_part2)), 1.0)
            degree90 = (angle == round(math.pi / 2, 6) or angle == round(math.pi * 1.5, 6))
            touching = w1.is_touching(w2)
            same_wall_type = w1.module == w2.module

            if not (z_parallel and degree90 and touching and same_wall_type):
                continue

            for l1 in w1.layers:
                for l2 in w2.layers:
                    if not l1.is_touching(l2, tolerance=w1.module.width):
                        continue

                    mid1 = l1.center
                    mid2 = l2.center

                    direction1 = quaternion.rotate_vectors(r1, np.array([1, 0, 0]))
                    direction2 = quaternion.rotate_vectors(r2, np.array([1, 0, 0]))

                    A = np.vstack((direction1, -direction2, [1, 1, 1])).T
                    b_bottom = mid2 - mid1
                    try:
                        t = np.linalg.solve(A, b_bottom)

                        # Calculate the intersection points on both lines
                        intersection_point1 = mid1 + t[0] * direction1
                        intersection_point2 = mid2 + t[1] * direction2
                        assert np.allclose(intersection_point1, intersection_point2)

                        c = Corn(intersection_point1)
                        c.layers.update([l1, l2])
                        corners.add_corner(c)

                        if np.linalg.norm(intersection_point1 - l1.left_edge) < w1.module.width:  # TODO use wall width!
                            l1.left_connections.append(l2)
                        elif np.linalg.norm(intersection_point1 - l1.right_edge) < w1.module.width:
                            l1.right_connections.append(l2)

                        if np.linalg.norm(intersection_point1 - l2.left_edge) < w2.module.width:  # TODO use wall width!
                            l2.left_connections.append(l1)
                        elif np.linalg.norm(intersection_point1 - l2.right_edge) < w2.module.width:
                            l2.right_connections.append(l1)

                    except np.linalg.LinAlgError:
                        #print("no intersection found between", w1.name, "and", w2.name,  "even though they are touching")
                        continue
                    except AssertionError:
                        #print("intersection points are not the same for", w1.name, "and", w2.name,  "even though they are touching")
                        continue
    return corners
