import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class RealisationExportScenario(Scenario):
    def get_walls(self):
        base_module = BrickInformation(1.0, 1.0, 1.0, grid=np.array([0.5, 0.5, 0.5]))
        bond_type = "StretchedBond"
        width = 1.0

        w1 = Wall.make_wall(3.0, width, 1.0, np.array([0.0, 0.0, 0.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        w2 = Wall.make_wall(2.0, width, 1.0, np.array([0.0, 0.0, 1.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)
        w1_small = Wall.make_wall(1.0, width, 1.0, np.array([0.0, 0.0, 2.0]),
                            quaternion.from_euler_angles(0, 0, 0),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type="StretchedBond")

        all = [w2, w1, w1_small]
        return all