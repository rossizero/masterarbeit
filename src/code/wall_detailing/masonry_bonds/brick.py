import numpy as np
import quaternion
from OCC.Core.BRepBndLib import brepbndlib_Add

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec
from OCC.Core.Bnd import Bnd_Box


class BrickInformation:
    def __init__(self, length, width, height):
        self.length, self.width = max(length, width), min(length, width)
        self.height = height

    def volume(self):
        return self.length * self.width * self.height

    def get_rotated_dimensions(self, rotation: quaternion):
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
    def __init__(self, brick_information: BrickInformation, position: np.array, global_rotation: quaternion, local_rotation: quaternion):
        self.position = position
        self.global_rotation = global_rotation
        self.local_rotation = local_rotation
        self.__brick_information = brick_information
        self.length = self.__brick_information.length
        self.width = self.__brick_information.width
        self.height = self.__brick_information.height

    def get_brep_shape(self):
        offset = 0.02
        corner = gp_Pnt(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0)

        shape = BRepPrimAPI_MakeBox(corner, self.length - offset, self.width - offset, self.height - offset).Shape()

        # translate the center of the shape to 0 0 0...
        #transformation = gp_Trsf()
        #transformation.SetTranslation(gp_Vec(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0))
        #shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # ...to rotate around local rotation....
        transformation = gp_Trsf()
        rotation = self.local_rotation
        rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # ...then translate back to...
        transformation = gp_Trsf()
        l, w, h = self.__brick_information.get_rotated_dimensions(self.local_rotation)
        transformation.SetTranslation(gp_Vec(l / 2.0, w / 2.0, h / 2.0))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # ...translate to global position
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.position))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # Now rotate globally
        transformation = gp_Trsf()
        rotation = gp_Quaternion(self.global_rotation.x, self.global_rotation.y, self.global_rotation.z, self.global_rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        return shape
