from abc import ABC, abstractmethod

from OCC.Core.BRepPrimAPI import BRepPrimAPI_MakeBox
from OCC.Core.gp import gp_Pnt, gp_Quaternion, gp_Trsf, gp_Vec
from OCC.Core.BRepBuilderAPI import BRepBuilderAPI_Transform

from detailing.wall import Wall

import numpy as np
import quaternion
import math


def make_wall(length, width, height, position, rotation, ifc_wall_type, name=""):
    length, width = max(length, width), min(length, width)
    corner = gp_Pnt(-length/2.0, -width/2.0, -height/2.0)

    shape = BRepPrimAPI_MakeBox(corner, length, width, height).Shape()

    transformation = gp_Trsf()
    transformation.SetRotation(gp_Quaternion(rotation.x, rotation.y, rotation.z, rotation.w))
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

    transformation = gp_Trsf()
    transformation.SetTranslation(gp_Vec(*position))
    shape = BRepBuilderAPI_Transform(shape, transformation).Shape()

    wall = Wall(shape, ifc_wall_type, name)
    return wall


class Scenario(ABC):
    def __init__(self):
        self.walls = self.get_walls()

    @abstractmethod
    def get_walls(self):
        return []


class FancyCorners(Scenario):
    def get_walls(self):
        an = math.pi / 2 * 1.22432
        w1 = make_wall(10, 1, 5, np.array([5.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
                       ifc_wall_type="test", name="w1")
        w11 = make_wall(10, 1, 10, np.array([5.5, 10.0, -1.0]),
                        quaternion.from_euler_angles(0, 0, math.pi / 2 + math.pi), ifc_wall_type="test", name="w11")
        w11_ = make_wall(5, 1, 10, np.array([5.5, 10.0, 9.0]),
                         quaternion.from_euler_angles(0, 0, math.pi / 2 + math.pi), ifc_wall_type="test", name="w11_")
        w11_2 = make_wall(5, 1, 10, np.array([5.5, 10.0, -11.0]),
                          quaternion.from_euler_angles(0, 0, math.pi / 2 + math.pi), ifc_wall_type="test", name="w11_2")
        w111 = make_wall(5, 1, 10, np.array([5.5, -7.5, 1.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
                         ifc_wall_type="test", name="w111")
        w2 = make_wall(10, 1, 5, np.array([10.0, 4.5, 1.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w2")
        w3 = make_wall(10, 1, 5, np.array([-5.5, 0.0, -1.0]), quaternion.from_euler_angles(0.0, 0.0, math.pi / 2),
                       ifc_wall_type="test", name="w3")
        w4 = make_wall(10, 1, 5, np.array([0.0, -4.5, -3.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w4")
        w4_ = make_wall(4, 1, 5, np.array([0.0, -4.5, 2]), quaternion.from_euler_angles(0.0, 0.0, 0),
                        ifc_wall_type="test", name="w4_")
        w41_ = make_wall(4, 1, 5, np.array([1.5, -2.0, 3]), quaternion.from_euler_angles(0.0, 0.0, math.pi / 2),
                         ifc_wall_type="test", name="w41_")

        w1.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w11.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w11_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w11_2.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w111.rotate_around(quaternion.from_euler_angles(0.3, an, an))

        w2.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w4.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w4_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w41_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        return [w1, w2, w3, w4, w4_, w41_]


class SimpleCorners(Scenario):
    def get_walls(self):
        an = math.pi / 2 * 1.22432
        w1 = make_wall(10, 1, 5, np.array([5.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2  + math.pi),
                       ifc_wall_type="test", name="w1")
        w2 = make_wall(10, 1, 5, np.array([10.0, 4.5, 0.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w2")
        w3 = make_wall(10, 1, 5, np.array([14.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
                       ifc_wall_type="test", name="w3")
        w4 = make_wall(10, 1, 5, np.array([10.0, -4.5, 0.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w4")

        #w11 = make_wall(10, 1, 5, np.array([5.5, 20.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
        #               ifc_wall_type="test", name="w11")
        #w22 = make_wall(10, 1, 5, np.array([10.0, 24.5, 0.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
        #               ifc_wall_type="test", name="w22")

        w1.rotate_around(quaternion.from_euler_angles(0, an, an))
        w2.rotate_around(quaternion.from_euler_angles(0, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0, an, an))
        w4.rotate_around(quaternion.from_euler_angles(0, an, an))
        #w11.rotate_around(quaternion.from_euler_angles(0, an, an))
        #w22.rotate_around(quaternion.from_euler_angles(0, an, an))
        return [w1, w2]


class SimpleCorners2(Scenario):
    def get_walls(self):
        an = math.pi / 2 * 1.22432
        an = 0
        height = 0.5 * 2
        w0 = make_wall(10, 1, height, np.array([5.5, 0.0, height/2]), quaternion.from_euler_angles(0, 0, math.pi / 2 + 0* math.pi),
                       ifc_wall_type="test", name="w0")
        w1 = make_wall(10, 1, height, np.array([10.0, 5.5, height/2]), quaternion.from_euler_angles(0.0, 0.0, 1*math.pi),
                       ifc_wall_type="test", name="w1")
        w2 = make_wall(10, 1, height, np.array([14.5, 0.0, height/2]), quaternion.from_euler_angles(0, 0, math.pi / 2 + 0* math.pi),
                       ifc_wall_type="test", name="w2")
        w3 = make_wall(10, 1, height, np.array([10.0, -4.5, height/2]), quaternion.from_euler_angles(0.0, 0.0, 0* math.pi),
                       ifc_wall_type="test", name="w3")

        #w11 = make_wall(10, 1, 5, np.array([5.5, 20.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
        #               ifc_wall_type="test", name="w11")
        #w22 = make_wall(10, 1, 5, np.array([10.0, 24.5, 0.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
        #               ifc_wall_type="test", name="w22")

        w0.rotate_around(quaternion.from_euler_angles(0, an, an))
        w1.rotate_around(quaternion.from_euler_angles(0, an, an))
        w2.rotate_around(quaternion.from_euler_angles(0, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0, an, an))
        #w11.rotate_around(quaternion.from_euler_angles(0, an, an))
        #w22.rotate_around(quaternion.from_euler_angles(0, an, an))
        return [w1, w2]