from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Trsf, gp_Quaternion, gp_Vec
from typing import Tuple
from detailing.wall import Wall
from masonry.brick import BrickInformation

import numpy as np
import quaternion


class Opening:
    """
    basically a box that describes a hole in a wall
    """
    def __init__(self, parent: Wall, translation: np.array, rotation: quaternion, dimensions: Tuple[float, float, float]):
        self.parent = parent
        self.translation = translation  # relative to parents bottom left corner
        self.rotation = rotation  # relative to parent
        self.length = dimensions[0]
        self.width = dimensions[1]
        self.height = dimensions[2]
        self.lintel = BrickInformation(self.length + 1, 1.0, 0.5)

    def get_rotation(self):
        return self.rotation

    def get_position(self, relative: bool = False):
        ret = self.translation.copy()
        ret -= np.array([self.parent.length / 2, self.parent.width / 2, self.parent.height / 2])
        ret += np.array([self.length / 2, self.width / 2, self.height / 2])

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation() - np.array([self.parent.length/2, self.parent.width/2, self.parent.height/2])
        return ret

    def get_lintel_position(self, relative: bool = False):
        relative_position = self.get_position(True)
        relative_position -= np.array([self.lintel.length / 2.0, self.width / 2.0, -self.height / 2.0])
        return relative_position

    def get_shape(self):
        corner = gp_Pnt(-self.length / 2.0, -self.width / 2.0, -self.height / 2.0)

        shape = BRepPrimAPI_MakeBox(corner, self.length, self.width, self.height).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(self.rotation.x, self.rotation.y, self.rotation.z, self.rotation.w))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        transformation = gp_Trsf()
        relative_position = self.get_position(True)
        transformation.SetTranslation(gp_Vec(*relative_position))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

        transformation = gp_Trsf()
        transformation.SetRotation(gp_Quaternion(self.parent.get_rotation().x,
                                                 self.parent.get_rotation().y,
                                                 self.parent.get_rotation().z,
                                                 self.parent.get_rotation().w))

        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        transformation = gp_Trsf()
        global_position = self.parent.get_translation()
        transformation.SetTranslation(gp_Vec(*global_position))
        shape = BRepBuilderAPI_Transform(shape, transformation).Shape()
        return shape
