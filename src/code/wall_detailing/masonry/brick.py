import json
import math
from enum import Enum, unique
from typing import List

import numpy as np
import quaternion

from OCC.Core.BRepBndLib import brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec
from OCC.Core.Bnd import Bnd_Box


@unique
class Neighbor(str, Enum):
    LEFT = "left"
    RIGHT = "right"
    FRONT = "front"
    BACK = "back"
    TOP = "top"
    BOTTOM = "bottom"

    @staticmethod
    def opposite(neighbor: 'Neighbor'):
        tmp = {
            Neighbor.LEFT: Neighbor.RIGHT,
            Neighbor.RIGHT: Neighbor.LEFT,
            Neighbor.FRONT: Neighbor.BACK,
            Neighbor.BACK: Neighbor.FRONT,
            Neighbor.TOP: Neighbor.BOTTOM,
            Neighbor.BOTTOM: Neighbor.TOP
        }
        return tmp[neighbor]


class BrickInformation:
    """
    This class stores simple information about a brick:
    width, length and height where:
     length describes the scale along the x-axis and length >= width,
     width describes the scale along the y-axis and width <= length,
     height describes the scale along the z axis
    grid: rastermaß
    """

    def __init__(self, length, width, height, grid: np.array):
        self.length, self.width = max(length, width), min(length, width)
        self.height = height
        self.grid = grid

    def volume(self):
        """
        :return: volume of the brick
        """
        return self.length * self.width * self.height

    def is_valid(self) -> bool:
        """
        :return: true if the brick is valid
        """
        return self.volume() > 0.0

    def get_rotated_dimensions(self, rotation: quaternion):
        """
        :param rotation: the rotation we need the box in
        :return: the dimensions of the bounding box of the rotated box
            first value is the "new" length aka. the scale along the x axis
            second is the "new" width aka. the scale along the y axis
            thrid is the new height / z axis

        Example: you want the box to be rotated by 90 degrees around the z axis but then need to know the distance you
        make along the x axis with the placement of the rotated brick
        """
        corner = gp_Pnt(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0)
        if round(self.length, 6) == 0.0 or round(self.width, 6) == 0.0 or round(self.height, 6) == 0.0:
            return [self.length, self.width, self.height]
        shape = BRepPrimAPI_MakeBox(corner, self.length, self.width, self.height).Shape()
        # ...to rotate around local rotation....
        transformation = gp_Trsf()
        rotation = rotation
        rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # create a boundingbox around the shape
        bounding_box = Bnd_Box()
        brepbndlib.Add(shape, bounding_box)

        # Get the minimum and maximum coordinates of the bounding box
        xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()

        # Calculate the dimensions of the rotated box
        length = round(xmax - xmin, 6)
        width = round(ymax - ymin, 6)
        height = round(zmax - zmin, 6)
        return [length, width, height]

    def is_inside(self, point: np.array) -> bool:
        """
        :param point: point to check
        :return: true if the given point is inside the brick
        """
        return (0 <= point[0] <= self.length and
                0 <= point[1] <= self.width and
                0 <= point[2] <= self.height)

    def __eq__(self, other):
        if isinstance(other, BrickInformation):
            return other.length == self.length and other.width == self.width and other.height == self.height
        return False

    def __str__(self):
        return f"BrickInformation({self.length}, {self.width}, {self.height})"


class Brick:
    """
    Represents an actual brick which has the assigned brick-information
    initial rotation and translation can be set and will be handled in following order:
    local rotation is the rotation around the bricks own center
    position is the translation of the brick
    global rotation is the rotation around the origin after translation happened
    """

    def __init__(self, brick_information: BrickInformation):
        self.__brick_information = brick_information
        self.length = self.__brick_information.length
        self.width = self.__brick_information.width
        self.height = self.__brick_information.height
        self.id = None

        # MAYDO actually let the user set this
        self.offset = min(max(self.__brick_information.grid) / 20.0, 0.0325)

        self.neighbors = {Neighbor.LEFT: set(),
                          Neighbor.RIGHT: set(),
                          Neighbor.FRONT: set(),
                          Neighbor.BACK: set(),
                          Neighbor.TOP: set(),
                          Neighbor.BOTTOM: set()}

        self.shape = BRepPrimAPI_MakeBox(gp_Pnt(self.offset, self.offset, self.offset),
                                         self.length - self.offset * 2,
                                         self.width - self.offset * 2,
                                         self.height - self.offset * 2).Shape()

    def get_dimensions(self):
        """
        :return: dimensions of the brick
        """
        return np.array([self.length, self.width, self.height])

    def get_grid(self):
        """
        :return: grid size of the brick
        """
        return self.__brick_information.grid

    @property
    def all_neighbors(self):
        """
        :return: all neighbor bricks of the brick
        """
        return (self.neighbors[Neighbor.LEFT] |
                self.neighbors[Neighbor.RIGHT] |
                self.neighbors[Neighbor.FRONT] |
                self.neighbors[Neighbor.BACK] |
                self.neighbors[Neighbor.TOP] |
                self.neighbors[Neighbor.BOTTOM])

    @property
    def position(self) -> np.array:
        """
        :return: position of the brick in global coordinates
        """
        return np.array([self.shape.Location().Transformation().TranslationPart().X(),
                         self.shape.Location().Transformation().TranslationPart().Y(),
                         self.shape.Location().Transformation().TranslationPart().Z()])

    @property
    def orientation(self) -> np.quaternion:
        """
        :return: orientation of the brick in global coordinates
        """
        return np.quaternion(self.shape.Location().Transformation().GetRotation().W(),
                             self.shape.Location().Transformation().GetRotation().X(),
                             self.shape.Location().Transformation().GetRotation().Y(),
                             self.shape.Location().Transformation().GetRotation().Z())

    def translate(self, translation: np.array):
        """
        :param translation: translation to apply
        :return: self
        """
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*translation))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()
        return self

    def center(self):
        """
        :return: center of the brick in global coordinates
        """
        mid = quaternion.rotate_vectors(self.orientation,
                                        np.array([self.length / 2.0, self.width / 2.0, self.height / 2.0]))
        return self.position + mid

    def _bottom_left_corner_offset(self):
        """
        see get_rotated_dimensions
        returns the difference between the bricks center and its bottom left corner (needed to rotate around the center)
        """
        l, w, h = self.__brick_information.get_rotated_dimensions(self.orientation)
        mid = np.array([l / 2.0, w / 2.0, h / 2.0])
        return mid

    def rotate(self, rotation: np.quaternion):
        """
        rotates around center of brick
        """
        pos = self.shape.Location().Transformation().TranslationPart()

        # get current dimensions of the axis aligned bounding box of the brick to center the brick
        mid = self._bottom_left_corner_offset()

        # ...translate to 0 0 0...
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(pos.Reversed()))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*-mid))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()

        # ...rotate....
        transformation = gp_Trsf()
        rotation_ = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation_)
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()

        # now get the new dimensions of the axis aligned bounding box of the brick to move the bricks center to the
        # original location
        mid = self._bottom_left_corner_offset()

        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*mid))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()

        # ...translate back to original position...
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(pos))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()
        return self

    def rotate_around(self, rotation: np.quaternion, pivot_point: np.array = np.array([0.0, 0.0, 0.0])):
        """
        :param rotation:
        :param pivot_point:
        :return:
        """
        # ...translate...
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*-pivot_point))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()

        # ...rotate....
        transformation = gp_Trsf()
        rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation)
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()

        # ...then translate back...
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*pivot_point))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()
        return self

    """
    Note:
    This:
        vec = np.array([module.width / 2, module.width / 2, 0.0])
        b.translate(-vec)
        b.rotate_around(corner.get_rotation())
        b.translate(vec)
    is the same as:
        b.rotate_around(corner.get_rotation(), vec)
    """

    def get_neighbour_positions(self, grid: np.array):
        """
        :param grid: grid size
        :return: all possible neighbour positions of the brick using the given grid size
        """
        # bottom front left corner + rotated half of grid size
        pos = self.position + quaternion.rotate_vectors(self.orientation, grid / 2.0)

        length_steps = (self.length / grid[0])
        width_steps = (self.width / grid[1])
        height_steps = (self.height / grid[2])

        # calculate the rotated axis vectors
        right_v = quaternion.rotate_vectors(self.orientation, np.array([1, 0, 0]))  # x dir
        back_v = quaternion.rotate_vectors(self.orientation, np.array([0, 1, 0]))  # y dir
        top_v = quaternion.rotate_vectors(self.orientation, np.array([0, 0, 1]))  # z dir

        neighbor_positions = {Neighbor.LEFT: [],
                              Neighbor.RIGHT: [],
                              Neighbor.FRONT: [],
                              Neighbor.BACK: [],
                              Neighbor.TOP: [],
                              Neighbor.BOTTOM: []}

        for i in range(math.floor(length_steps)):
            front_face = pos.copy() + back_v * grid[1] * width_steps  # one step in front of bricks y
            back_face = pos.copy() - back_v * grid[1]  # all steps behind bricks y
            bottom_face = pos.copy() - top_v * grid[2]  # one step below bricks z
            top_face = pos.copy() + top_v * grid[2] * height_steps  # all steps above bricks z

            front_face += right_v * i * grid[0]  # move in x dir
            back_face += right_v * i * grid[0]  # move in x dir
            bottom_face += right_v * i * grid[0]  # move in x dir
            top_face += right_v * i * grid[0]  # move in x dir

            for step in range(math.floor(height_steps)):
                p1 = front_face.copy() + top_v * grid[2] * step  # move in z dir
                p2 = back_face.copy() + top_v * grid[2] * step  # move in z dir
                neighbor_positions[Neighbor.FRONT].append(np.round(p1, decimals=6))
                neighbor_positions[Neighbor.BACK].append(np.round(p2, decimals=6))

            for step in range(math.floor(width_steps)):
                p1 = bottom_face.copy() + back_v * grid[1] * step  # move in y dir
                p2 = top_face.copy() + back_v * grid[1] * step  # move in y dir
                neighbor_positions[Neighbor.BOTTOM].append(np.round(p1, decimals=6))
                neighbor_positions[Neighbor.TOP].append(np.round(p2, decimals=6))

        for i in range(math.floor(width_steps)):
            left_face = pos.copy() - right_v * grid[0]  # one step left of bricks x
            right_face = pos.copy() + right_v * grid[0] * length_steps  # all steps right of bricks x

            left_face += back_v * i * grid[1]  # move in y dir
            right_face += back_v * i * grid[1]  # move in y dir

            for step in range(math.floor(height_steps)):
                p1 = left_face.copy() + top_v * grid[2] * step  # move in z dir
                p2 = right_face.copy() + top_v * grid[2] * step  # move in z dir
                neighbor_positions[Neighbor.LEFT].append(np.round(p1, decimals=6))
                neighbor_positions[Neighbor.RIGHT].append(np.round(p2, decimals=6))

        return neighbor_positions

    def is_inside(self, position: np.array):
        """
        :param position: position to check
        :return: true if the given position is inside the brick
        """
        relative_point = position - self.position
        relative_point = quaternion.rotate_vectors(self.orientation.inverse(), relative_point)
        return self.__brick_information.is_inside(relative_point)


def calculate_neighborhood(bricks: List[Brick]):
    """
    :param bricks: list of bricks to calculate the neighborhood for
    calculates all neighbours of each brick using the given grid as step size
    waaaay faster than the old method (calculate_neighborhood_bruteforce) because we're looking at all bricks at once

    btw: if a brick is a neighbor of itself, probably the given grid size is too big
    """
    # first retrieve all possible neighbor positions for each brick
    # while simultaneously remembering which brick has which position
    # key is the point as tuple, value is a list of tuples (brick_index, neighbor)
    dic = {}
    for i, brick in enumerate(bricks):
        neighbors = brick.get_neighbour_positions(brick.get_grid())
        for key in neighbors.keys():
            for pos in neighbors[key]:
                if tuple(pos) in dic.keys():
                    dic[tuple(pos)].append((i, key))
                else:
                    dic[tuple(pos)] = [(i, key)]

    # now iterate over each brick and check what neighbor position are inside of it (all at once)
    points = np.array([np.array([d[0], d[1], d[2]]) for d in dic.keys()])
    for brick in bricks:
        # convert points into local coordinate system of the current brick
        relative_points = points - brick.position
        relative_points = quaternion.rotate_vectors(brick.orientation.inverse(), relative_points)

        # since its position is at [0.0, 0.0, 0.0] we just have to check if a points is inside the bricks dimensions
        inside_mask = np.all((relative_points >= 0.0) & (relative_points <= brick.get_dimensions()), axis=1)
        for index in np.where(inside_mask)[0]:
            for brick_index, neighborhood in dic[tuple(points[index])]:
                # this is not always true (it depends on the rotation of the neighbor)
                # opp = Neighbor.opposite(neighborhood)
                # brick.neighbors[opp].add(bricks[brick_index])
                bricks[brick_index].neighbors[neighborhood].add(brick)


def calculate_neighborhood_bruteforce(bricks: List[Brick]):
    """
    DEPRECATED
    :param bricks: list of bricks to calculate the neighborhood for
    calculates all neighbours of each brick using the given grid as step size
    """
    s = 0
    for i, brick in enumerate(bricks):
        if i % 10 == 0:
            print(i, "/", len(bricks))
        grid = brick.get_grid()
        neighbor_positions = brick.get_neighbour_positions(grid)

        for neighbor_key in neighbor_positions.keys():
            for pos in neighbor_positions[neighbor_key]:
                for other in bricks[i + 1:]:
                    if other.is_inside(pos):
                        opp = Neighbor.opposite(neighbor_key)

                        if other not in brick.neighbors[neighbor_key]:
                            print("Ohhhh")
                        brick.neighbors[neighbor_key].add(other)
                        if brick not in other.neighbors[opp]:
                            print("Ohhhh")
                        other.neighbors[opp].add(brick)
                        # we found a brick for this position, so we can stop searching
                        # we suppose that there is only one brick for each position
                        # otherwise there would be a collision between bricks
                        break
        s += len(brick.all_neighbors)
