import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class CombinationExampleForText(Scenario):
    def get_walls(self):
        width = 1
        height = 5
        small = False
        base_module = BrickInformation(2.0, 1.0, 0.5, grid=np.array([1.0, 1.0, 0.5]))
        bond_type = "StretchedBond"

        if small:
            bond_type = "CrossBond"
            base_module = BrickInformation(1.0, 0.5, 0.25, grid=np.array([0.5, 0.5, 0.25]))

        w1 = Wall.make_wall(9, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        w2 = Wall.make_wall(9, width, height * 0.5, np.array([-3.5 + 4, 1.0, height / 4.0 + 1.5]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        w3 = Wall.make_wall(3, width, 3 * 0.5, np.array([-3.5 + 4 + 6, 1.0, height / 4.0 + 1.5]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        w4 = Wall.make_wall(3, width, 3 * 0.5, np.array([-3.5 + 4 + 6, 1.0, height / 4.0 - 0.5]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test",base_module=base_module, bond_type=bond_type)

        w5 = Wall.make_wall(7, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0 + 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        w6 = Wall.make_wall(7, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0 + 4.0]),
                                 quaternion.from_euler_angles(0, 0, 0 * math.pi),
                                 ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        w7 = Wall.make_wall(4, width, 3 * 0.5, np.array([-3.5 + 6.5, 1.0, height / 4.0 + 3.5]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", base_module=base_module, bond_type=bond_type)

        an = 0  # 0.4886921905584123
        w1.rotate_around(quaternion.from_euler_angles(0, an, an))
        w2.rotate_around(quaternion.from_euler_angles(0, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0, an, an))
        w4.rotate_around(quaternion.from_euler_angles(0, an, an))
        w5.rotate_around(quaternion.from_euler_angles(0, an, an))
        w6.rotate_around(quaternion.from_euler_angles(0, an, an))
        w7.rotate_around(quaternion.from_euler_angles(0, an, an))
        return [w1, w2, w3, w4, w5, w6, w7]
