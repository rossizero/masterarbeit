import itertools
import math
import numpy as np
import quaternion

from detailing.wall_layer import WallLayer
from detailing.wall_layer_group import WallLayerGroup
from typing import List, Dict, Tuple, Optional
from masonry.bond.abstract_bond import Bond
from die_mathe.line import Line


class Corn:
    idd = 0
    """
    A class to store information about layers that form a corner, a t-joint or a crossing.
    """

    def __init__(self, point: np.array):
        self.point = point  # center of the corner
        self.layers = set()  # layers that form the corner
        self.plan_offset = 0
        self.touched = False
        self.main_layer = None
        self.id = Corn.idd
        Corn.idd += 1

    def set_plan_offset(self, offset: int):
        """
        :param offset: the offset of the plan
        """
        assert not self.touched
        self.plan_offset = offset
        self.touched = True

    def get_main_layer(self):
        """
        The main layer is considered to be the or one layer in which the corner point lies in.
        Its rotation is being used for calculations.
        :return: the main layer
        """
        if self.main_layer is not None:
            return self.main_layer

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
        # also make sure to call set_main_layer() before reducing the length of layers
        assert False

    def _get_corner_direction(self) -> np.array:
        """
        :return: vector that points in the direction of the corners inside
        """
        if len(self.layers) == 2:
            # the wall we "place" the corner onto

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
            mid = wall_orientation_1 + wall_orientation_2

            return mid
        return np.array([0, 0, 0])

    def get_rotation(self) -> np.quaternion:
        """
        How do we have to rotate the Corner Plan of a bond to lie above the axis the layers form
        :return: rotation we have to apply to the corner plan of a bond
        """
        ret = np.quaternion(1, 0, 0, 0)
        if len(self.layers) == 2:
            # the wall we "place" the corner onto

            main_layer = self.get_main_layer()

            # mid of those vectors -> the point in the corner between both walls and 45 degrees from both walls
            mid = self._get_corner_direction()

            # normalise rotation
            mid = quaternion.rotate_vectors(main_layer.parent.get_rotation().inverse(), mid)
            z_rot = np.arctan2(mid[1], mid[0])
            # subtract the 45 degrees from above
            ret = quaternion.from_euler_angles(0, 0, z_rot - math.pi / 4.0)
        return ret

    def reduce_corner_layer_length(self, bond: Bond):
        """
        :param bond: bond we want to use
        reduces the length of all layers that form the corner by the amount of length the corner takes up with given bond
        """
        for layer in self.layers:
            self.reduce_layer_length(layer, bond)

    def reduce_layer_length(self, layer: WallLayer, bond: Bond):
        """
        :param layer: layer we want to reduce
        :param bond: bond we want to use
        """
        assert layer in self.layers

        main_layer = self.get_main_layer()
        angle = self.get_rotation()

        relative_rotation = (layer.parent.get_rotation() * main_layer.parent.get_rotation().inverse())
        a = relative_rotation.angle()

        # we know the relative_rotation represents the z-rotation difference
        relative_rotation = quaternion.from_euler_angles(0, 0, a) * angle

        # how far the corner stretches into the layer (x direction)
        corner_length = bond.get_corner_length(self.plan_offset, relative_rotation)

        # calculate the outer corner point
        mid = self._get_corner_direction()
        width = layer.parent.wall.width / 2.0
        outer_corner_point = self.point - np.array([width, width, 1]) * mid
        # finally reduce the length of the layer starting from outer_corner_point and going corner_length into the layer
        layer.move_edge(outer_corner_point, corner_length)

    def set_main_layer(self):
        """
        sets the main layer of the corner
        """
        self.main_layer = self.get_main_layer()

    def get_corner_index(self):
        """
        :return: the index/height of the corner in the wall it is in
        """
        ids = [l.parent.id for l in self.layers]
        main_layer = self.get_main_layer()
        ids.remove(main_layer.parent.id)
        layers = main_layer.parent.get_sorted_layers()

        vals = []
        for i, layers_in_height in enumerate(layers):
            for layer in layers_in_height:
                if layer in self.layers:
                    return i
                for l in layer.left_connections:
                    if l.parent.id in ids:
                        vals.append(i)
                for l in layer.right_connections:
                    if l.parent.id in ids:
                        vals.append(i)
        return min(vals)

    def get_unrotated_world_coordinates(self):
        """
        :return: the world coordinates of the corner without parent rotation applied
        """
        p = quaternion.rotate_vectors(self.main_layer.parent.get_rotation().inverse(), self.point)
        return np.round(p, 6)

    def __eq__(self, other: "Corn"):
        if type(other) is Corn:
            return np.allclose(self.point, other.point)
        return False

    def __str__(self):
        a = [l.parent.id for l in self.layers]
        a = set(sorted(a))
        return "corner: " + str(a) + " " + str(self.id)


class Corns:
    """
    A class that handles a list of corners.
    If we find a corner that already exists, we simply add the layers to the existing layers that already
    form the corner. (T- Joints or Crossings can be made possible this way)
    """

    def __init__(self):
        self.corners = []

    def add_corner(self, corner: Corn):
        """
        Creates a new corner or append to an existing corner if there already is a corner at the same location
        :param corner: a corner to add to the list of corners
        """
        for c in self.corners:
            if c == corner:
                c.layers.update(corner.layers)
                if len(c.layers) == 3:
                    print("TJoint")
                if len(c.layers) == 4:
                    print("Crossing!")
                return
        self.corners.append(corner)

    def grouped_by_walls(self) -> Dict[Tuple[int], List[Corn]]:
        dic = {}
        for corner in self.corners:
            walls = [layer.parent.id for layer in corner.layers]
            key = tuple(sorted(walls))
            if key not in dic.keys():
                dic[key] = []
            dic[key].append(corner)
        return dic

    def get_corner(self, layers: List[WallLayer]) -> Optional[Corn]:
        """
        :param layers: a list of layers
        :return: the corner that is formed by the given layers
        """
        if len(layers) == 0:
            return None
        for corn in self.corners:
            found = True
            for l in layers:
                if l not in corn.layers:
                    found = False
                    break
            if found:
                return corn
        return None

    def get_bottom_corner(self, corner: Corn):
        """
        :param corner: a corner
        :return: the corner below the given corner
        """
        l = []
        for layer in corner.layers:
            bottoms = layer.bottoms
            if bottoms is not None and len(bottoms) > 0:
                l.append(bottoms)

        for entry in list(itertools.product(*l)):
            c = self.get_corner(entry)
            if c is not None:
                return c
        return None

    def get_top_corner(self, corner: Corn):
        """
        :param corner: a corner
        :return: the corner above the given corner
        """
        l = []
        for layer in corner.layers:
            tops = layer.tops
            if tops is not None and len(tops) > 0:
                l.append(tops)

        for entry in list(itertools.product(*l)):
            c = self.get_corner(entry)
            if c is not None:
                return c
        return None

    def get_corners_sorted_by_z(self):
        """
        :return: a list of corners sorted by their z coordinate
        """
        return sorted(self.corners, key=lambda x: x.get_unrotated_world_coordinates()[2])


def check_for_corners(wall_layer_groups: List[WallLayerGroup]) -> Corns:
    """
    TODO needs to be faster (look at neighborhood calculation of bricks, maybe sth like that works here too)
    :param wall_layer_groups: a list of wall_layer_groups
    :return: a list of corners
    """
    corners = Corns()

    for i, w1 in enumerate(wall_layer_groups):
        for j in range(i + 1, len(wall_layer_groups)):
            w2 = wall_layer_groups[j]
            r1 = w1.get_rotation()
            r2 = w2.get_rotation()
            diff = r2 * r1.inverse()
            angle = round(diff.angle(), 6)

            # check if rotation of wall leads to parallel corners
            z_part1 = quaternion.rotate_vectors(r1, np.array([0.0, 0.0, 1.0]))
            z_part2 = quaternion.rotate_vectors(r2, np.array([0.0, 0.0, 1.0]))
            z_parallel = np.isclose(abs(np.dot(z_part1, z_part2)), 1.0)

            x_part1 = quaternion.rotate_vectors(r1, np.array([1.0, 0.0, 0.0]))
            x_part2 = quaternion.rotate_vectors(r2, np.array([1.0, 0.0, 0.0]))
            x_parallel = np.isclose(abs(np.dot(x_part1, x_part2)), 1.0)

            degree90 = (angle == round(math.pi / 2, 6) or angle == round(math.pi * 1.5, 6))
            # touching = w1.is_touching(w2)
            same_wall_type = w1.module == w2.module
            # if not (z_parallel and degree90 and touching and same_wall_type):
            #    continue

            for l1 in w1.layers:
                for l2 in w2.layers:
                    if l1.is_touching(l2):
                        line1 = Line(l1.left_edge, l1.right_edge)
                        line2 = Line(l2.left_edge, l2.right_edge)

                        intersection = line1.intersection(line2)

                        if intersection is None:
                            continue

                        width = w1.wall.width
                        if l1.is_touching_at_endpoints(l2,
                                                       tolerance=width) and z_parallel and degree90 and same_wall_type:
                            c = Corn(intersection)
                            c.layers.update([l1, l2])
                            corners.add_corner(c)

                            if np.linalg.norm(intersection - l1.left_edge) < width:
                                l1.left_connections.append(l2)
                            elif np.linalg.norm(intersection - l1.right_edge) < width:
                                l1.right_connections.append(l2)

                            assert len(set(l1.right_connections) & set(l1.left_connections)) == 0

                            if np.linalg.norm(intersection - l2.left_edge) < width:
                                l2.left_connections.append(l1)
                            elif np.linalg.norm(intersection - l2.right_edge) < width:
                                l2.right_connections.append(l1)

                            assert len(set(l2.right_connections) & set(l2.left_connections)) == 0

                        elif not x_parallel:
                            # check if the intersection is on both lines and in between their endpoints
                            a = line1.on_line(intersection, between=True, tolerance=width)
                            b = line2.on_line(intersection, between=True, tolerance=width)

                            if a and b:
                                t_joint = (np.linalg.norm(intersection - l1.left_edge) < width
                                           or np.linalg.norm(intersection - l1.right_edge) < width
                                           or np.linalg.norm(intersection - l2.left_edge) < width
                                           or np.linalg.norm(intersection - l2.right_edge) < width)
                                # c = Corn(intersection)
                                # c.layers.update([l1, l2, t_joint])
                                # corners.add_corner(c)
                                # print("T-Joint" if t_joint else "Crossing", [round(i, 6) for i in intersection], x_parallel)
    # TODO
    # for now just remove TJoints and Crossings
    # print(len(corners.corners))
    # for corner in corners.corners.copy():
    #    if True in corner.layers or False in corner.layers:
    #        corners.corners.remove(corner)
    # print(len(corners.corners))
    return corners
