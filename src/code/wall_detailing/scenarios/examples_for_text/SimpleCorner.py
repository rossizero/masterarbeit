import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class SimpleCorner(Scenario):
    def get_walls(self):
        width = 1
        height = 2
        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([1.0, 1.0, 0.5]))
        bond_type = "StretchedBond"

        w00 = Wall.make_wall(2, width, height * 0.5, np.array([0.0, 0.0, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)

        w01 = Wall.make_wall(2, width, height * 0.5, np.array([0.5, 0.5, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                             ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)

        return [w00, w01]
