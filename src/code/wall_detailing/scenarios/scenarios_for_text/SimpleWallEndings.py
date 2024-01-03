import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class Single_Wall_Slim(Scenario):
    def get_walls(self):
        width = 1
        height = 5

        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([0.5, 0.5, 0.5]))
        bond_type = "StretchedBond"

        w1 = Wall.make_wall(5, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)
        return [w1]


class Single_Wall_Thick(Scenario):
    def get_walls(self):
        width = 2
        height = 5

        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([1.0, 1.0, 0.5]))
        bond_type = "CrossBond"

        w1 = Wall.make_wall(5, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)
        return [w1]
