import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class Scenario4_Ontology(Scenario):
    def get_walls(self):
        base_module = BrickInformation(2.0, 2.0, 1, grid=np.array([0.5, 0.5, 1.0]))
        bond_type = "StretchedBond"
        width = 2
        height = 8
        an = 0.0

        w1 = Wall.make_wall(6, width, height, np.array([0.0, 0.0, height / 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0", base_module=base_module, bond_type=bond_type)

        w1.rotate_around(quaternion.from_euler_angles(0, 0.0, an))
        return [w1]