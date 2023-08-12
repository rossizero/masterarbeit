import math
import numpy as np
import quaternion

from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec


class BrickInformation:
    def __init__(self, length, width, height):
        self.length = max(length, width)
        self.width = min(length, width)
        self.height = height

    def volume(self):
        return self.length * self.width * self.height


class Brick(BrickInformation):
    def __init__(self, length, width, height, position: np.array, rotation: quaternion):
        super().__init__(length, width, height)
        self.position = position
        self.rotation = rotation

    def get_brep_shape(self):
        offset = 0.01
        corner = gp_Pnt(0, 0, 0)

        shape = BRepPrimAPI_MakeBox(corner, self.length - offset, self.width - offset, self.height - offset).Shape()

        # translate to center shape around 0 0 0
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # rotate around local rotation
        transformation = gp_Trsf()
        rotation = quaternion.from_euler_angles(0, 0, 0)
        rotation = gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # translate back
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(self.length / 2.0, self.width / 2.0, self.height / 2.0))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # translate to global position
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.position))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        # rotate
        transformation = gp_Trsf()
        rotation = gp_Quaternion(self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w)
        transformation.SetRotation(rotation)
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        return shape