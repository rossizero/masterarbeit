import numpy as np
import quaternion

from OCC.Core.BRepBndLib import brepbndlib_Add
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec
from OCC.Core.Bnd import Bnd_Box


class BrickInformation:
    """
    This class stores simple information about a brick:
    width, length and height where:
     length describes the scale along the x axis and length >= width
     width describes the scale along the y axis and width <= length
     height describes the scale along the z axis
    """
    def __init__(self, length, width, height):
        self.length, self.width = max(length, width), min(length, width)
        self.height = height

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
        brepbndlib_Add(shape, bounding_box)

        # Get the minimum and maximum coordinates of the bounding box
        xmin, ymin, zmin, xmax, ymax, zmax = bounding_box.Get()

        # Calculate the dimensions of the rotated box
        length = round(xmax - xmin, 6)
        width = round(ymax - ymin, 6)
        height = round(zmax - zmin, 6)
        return [length, width, height]


class Brick:
    """
    Represents an actual brick which has the assigned brick-information
    initial rotation and translation can be set and will be handled in following order:
    local rotation is the rotation around the bricks own center
    position is the translation of the brick
    global rotation is the rotation around the origin after translation happened
    """
    def __init__(self, brick_information: BrickInformation,
                 position: np.array = np.array([0.0, 0.0, 0.0]),
                 global_rotation: quaternion = np.quaternion(1, 0, 0, 0),
                 local_rotation: quaternion = np.quaternion(1, 0, 0, 0)):
        self.__brick_information = brick_information
        self.length = self.__brick_information.length
        self.width = self.__brick_information.width
        self.height = self.__brick_information.height

        self.offset = 0.0325

        self.shape = BRepPrimAPI_MakeBox(gp_Pnt(self.offset, self.offset, self.offset), self.length - self.offset * 2, self.width - self.offset * 2,
                                    self.height - self.offset * 2).Shape()
        self.rotate(local_rotation).translate(position).rotate_around(global_rotation)

    @property
    def position(self):
        return np.array([self.shape.Location().Transformation().TranslationPart().X(),
                         self.shape.Location().Transformation().TranslationPart().Z(),
                         self.shape.Location().Transformation().TranslationPart().Y()])

    @property
    def orientation(self):
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
