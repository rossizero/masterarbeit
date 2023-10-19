from typing import Tuple

import numpy as np
import quaternion
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Trsf, gp_Quaternion, gp_Vec

from wall_detailing.detailing.wall import Wall


class Opening:
    """
    basically a box that describes a hole in a wall
    """
    def __init__(self, parent: Wall, translation: np.array, rotation: quaternion, dimensions: Tuple[float, float, float]):
        self.parent = parent
        self.translation = translation  # relative to parent
        self.rotation = rotation  # relative to parent
        self.length = dimensions[0]
        self.width = dimensions[1]
        self.height = dimensions[2]

    def get_position(self, relative: bool = False):
        ret = self.translation.copy()

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    def get_shape(self):
        corner = gp_Pnt(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0)

        shape = BRepPrimAPI_MakeBox(corner, self.length, self.width, self.height).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.get_position(True)))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(self.parent.get_rotation().x,
                                                 self.parent.get_rotation().y,
                                                 self.parent.get_rotation().z,
                                                 self.parent.get_rotation().w))

        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        transformation = gp_Trsf()
        transformation.SetTranslation(gp_Vec(*self.parent.get_translation()))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        return shape