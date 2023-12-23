import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class BasicsStretchedBond(Scenario):
    def get_walls(self):
        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([0.5, 0.5, 0.5]))
        bond_type = "StretchedBond"
        width = 1
        height = 4

        w1 = Wall.make_wall(8.5, width, height, np.array([0.0, 0.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        return [w1]


class BasicsCrossBond(Scenario):
    def get_walls(self):
        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([0.5, 0.5, 0.5]))
        bond_type = "CrossBond"
        width = 2
        height = 4

        w1 = Wall.make_wall(8, width, height, np.array([0.0, 0.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        return [w1]


class BasicsGothicBond(Scenario):
    def get_walls(self):
        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([0.5, 0.5, 0.5]))
        bond_type = "GothicBond"
        width = 2
        height = 4

        w1 = Wall.make_wall(8.5, width, height, np.array([0.0, 0.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        return [w1]


class BasicsHeadBond(Scenario):
    def get_walls(self):
        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([0.5, 0.5, 0.5]))
        bond_type = "HeadBond"
        width = 2
        height = 4

        w1 = Wall.make_wall(8, width, height, np.array([0.0, 0.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        return [w1]