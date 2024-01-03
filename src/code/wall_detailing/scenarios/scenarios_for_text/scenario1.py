import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class Scenario1(Scenario):
    def get_walls(self):
        width = 1
        height = 20

        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([0.5, 0.5, 0.5]))

        bond_type = "StretchedBond" if width == 1 else "CrossBond"

        w0 = Wall.make_wall(10, width, height, np.array([5.0, width/2, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="test", base_module=base_module, bond_type=bond_type)
        w1 = Wall.make_wall(10, width, height, np.array([5.0, 10.0 - width/2, height / 2.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        w2 = Wall.make_wall(10, width, height, np.array([width/2, 5.0, height / 2.0]),
                             quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                             ifc_wall_type="test", base_module=base_module, bond_type=bond_type)
        w3 = Wall.make_wall(8, width, height, np.array([10 - width / 2, 5.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)
        return [w0, w1, w2, w3]
