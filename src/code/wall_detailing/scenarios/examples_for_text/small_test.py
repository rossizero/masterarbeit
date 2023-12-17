import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class SmallTestToCompareIFC(Scenario):
    def get_walls(self):
        base_module = BrickInformation(0.4, 0.2, 0.12, grid=np.array([0.1, 0.1, 0.12]))
        bond_type = "HeadBond"
        width = 0.4

        w1 = Wall.make_wall(1.0, width, 0.24, np.array([0.9, 5.2, 0.12]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        w2 = Wall.make_wall(1.0, width, 0.24, np.array([0.2, 5.5, 0.12]),
                             quaternion.from_euler_angles(0, 0, 1.5 * math.pi),
                             ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)

        w1_small = Wall.make_wall(1.0, 0.2, 0.24, np.array([0.7, 1.1, 0.12]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type="StretchedBond")
        w2_small = Wall.make_wall(1.0, 0.2, 0.24, np.array([0.1, 1.5, 0.12]),
                            quaternion.from_euler_angles(0, 0, 1.5 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type="StretchedBond")

        all = [w1, w2, w1_small, w2_small]
        #for wall in all:
        #    print("wall_", wall.length, wall.width, wall.height, wall.get_translation(), wall.translation, wall.get_rotation(), wall.rotation)
        return all