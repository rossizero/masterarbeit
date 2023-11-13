import math
from typing import List

import numpy as np
import quaternion

from OCC.Core.BRepBndLib import brepbndlib_Add, brepbndlib
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec
from OCC.Core.Bnd import Bnd_Box


class BrickInformation:
    """
    This class stores simple information about a brick:
    width, length and height where:
     length describes the scale along the x-axis and length >= width,
     width describes the scale along the y-axis and width <= length,
     height describes the scale along the z axis
    grid: rastermaß
    """

    def __init__(self, length, width, height, grid=np.array):
        self.length, self.width = max(length, width), min(length, width)
        self.height = height
        self.grid = grid

    def volume(self):
        """
        :return: volume of the brick
        """
        return self.length * self.width * self.height

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

    def __eq__(self, other):
        if other is BrickInformation:
            return other.length == self.length and other.width == self.width and other.height == self.height
        return False


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

        self.offset = 0.0325

        self.shape = BRepPrimAPI_MakeBox(gp_Pnt(self.offset, self.offset, self.offset),
                                         self.length - self.offset * 2,
                                         self.width - self.offset * 2,
                                         self.height - self.offset * 2).Shape()

    @property
    def position(self) -> np.array:
        return np.array([self.shape.Location().Transformation().TranslationPart().X(),
                         self.shape.Location().Transformation().TranslationPart().Y(),
                         self.shape.Location().Transformation().TranslationPart().Z()])

    @property
    def orientation(self) -> np.quaternion:
        return np.quaternion(self.shape.Location().Transformation().GetRotation().W(),
                             self.shape.Location().Transformation().GetRotation().X(),
                             self.shape.Location().Transformation().GetRotation().Y(),
                             self.shape.Location().Transformation().GetRotation().Z())

    def translate(self, translation: np.array):
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*translation))
        self.shape = BRepBuilderAPI_Transform(self.shape, transformation).Shape()
        return self

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
        pos = self.position + grid / 2.0  # bottom front left corner + half of grid size

        l = math.floor(self.length / grid[0])
        w = math.floor(self.width / grid[1])
        h = math.floor(self.height / grid[2])

        right_v = quaternion.rotate_vectors(self.orientation, np.array([1, 0, 0]))  # x dir
        back_v = quaternion.rotate_vectors(self.orientation, np.array([0, 1, 0]))  # y dir
        top_v = quaternion.rotate_vectors(self.orientation, np.array([0, 0, 1]))  # z dir

        ret = []

        for i in range(l):
            front_face = pos.copy() + back_v * grid[1]  # one step in front of bricks y
            back_face = pos.copy() - back_v * grid[1] * w  # all steps behind bricks y
            bottom_face = pos.copy() - top_v * grid[2]  # one step below bricks z
            top_face = pos.copy() + top_v * grid[2] * h  # all steps above bricks z

            front_face += right_v * i * grid[0]  # move in x dir
            back_face += right_v * i * grid[0]  # move in x dir
            bottom_face += right_v * i * grid[0]  # move in x dir
            top_face += right_v * i * grid[0]  # move in x dir

            for j in range(h):
                p1 = front_face.copy() + top_v * grid[2] * j  # move in z dir
                p2 = back_face.copy() + top_v * grid[2] * j  # move in z dir
                ret.append(p1)
                ret.append(p2)

            for j in range(w):
                p1 = bottom_face.copy() - back_v * grid[1] * j  # move in y dir
                p2 = top_face.copy() - back_v * grid[1] * j  # move in y dir
                ret.append(p1)
                ret.append(p2)

        for i in range(w):
            left_face = pos.copy() - right_v * grid[0]  # one step left of bricks x
            right_face = pos.copy() + right_v * grid[0] * l  # all steps right of bricks x

            for j in range(h):
                p1 = left_face.copy() + top_v * grid[2] * j  # move in z dir
                p2 = right_face.copy() + top_v * grid[2] * j  # move in z dir
                ret.append(p1)
                ret.append(p2)

        return ret

    def is_inside(self, position: np.array):
        """
        :param position: position to check
        :return: true if the given position is inside the brick
        """
        relative_point = position - self.position
        relative_point = quaternion.rotate_vectors(self.orientation.inverse(), relative_point)

        return (0 <= relative_point[0] <= self.length and
                0 <= relative_point[1] <= self.width and
                0 <= relative_point[2] <= self.height)


def calculate_neighbourhood(bricks: List[Brick], grid: np.array):
    """
    calculates all neighbours of each brick using the given grid as step size
    """
    for brick in bricks:
        neighbor_positions = brick.get_neighbour_positions(grid)

        for other in bricks:
            if brick == other:
                continue

            for pos in neighbor_positions:
                if other.is_inside(pos):
                    pass
                    # brick.neighbours.append(other)
