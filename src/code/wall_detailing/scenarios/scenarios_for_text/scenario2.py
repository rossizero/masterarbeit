import math

import numpy as np
import quaternion

from wall_detailing.detailing.opening import Opening
from wall_detailing.detailing.wall import Wall
from wall_detailing.masonry.brick import BrickInformation
from wall_detailing.scenarios.scenarios import Scenario


class Scenario2(Scenario):
    def get_walls(self):
        thin_width = 0.008
        thick_width = 0.016

        height = 0.0768
        base_module_thick = BrickInformation(0.032, 0.016, 0.0096, grid=np.array([0.008, 0.008, 0.0096]))
        base_module_thin = BrickInformation(0.016, 0.008, 0.0096, grid=np.array([0.008, 0.008, 0.0096]))
        bond_type = "StretchedBond"

        w0 = Wall.make_wall(0.16, thick_width, height, np.array([0.0160 + 0.08, 0.008, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="LegoWallType2", base_module=base_module_thick, bond_type=bond_type)

        o0 = Opening(w0, np.array([0.136 - 13 * thin_width, 0, 2 * 0.0096]), quaternion.from_euler_angles(0, 0, 0), (4 * thin_width, 1 * thick_width, 4 * 0.0096))
        w0.openings.append(o0)

        w1 = Wall.make_wall(0.16, thick_width, height, np.array([0.0160 + 0.08, 0.128 + 0.008, height / 2.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="LegoWallType2", base_module=base_module_thick, bond_type=bond_type)

        w2 = Wall.make_wall(0.144, thick_width, height, np.array([0.008, 0.144 / 2.0, height / 2.0]),
                             quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                             ifc_wall_type="LegoWallType2", base_module=base_module_thick, bond_type=bond_type)

        o1 = Opening(w1, np.array([0.136 - 5 * thin_width, 0, 0]), quaternion.from_euler_angles(0, 0, 0), (4 * thin_width, 1 * thick_width, 6 * 0.0096))
        o1_2 = Opening(w1, np.array([0.136 - 13 * thin_width, 0, 2 * 0.0096]), quaternion.from_euler_angles(0, 0, 0), (4 * thin_width, 1 * thick_width, 4 * 0.0096))
        w1.openings.append(o1)
        w1.openings.append(o1_2)

        w3 = Wall.make_wall(0.144, thick_width, height, np.array([0.192 - 0.008, 0.144 / 2.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                            ifc_wall_type="LegoWallType2", base_module=base_module_thick, bond_type=bond_type)

        o3 = Opening(w3, np.array([thin_width * 6, 0, 2 * 0.0096]), quaternion.from_euler_angles(0, 0, 0),
                     (4 * thin_width, 1 * thick_width, 4 * 0.0096))
        w3.openings.append(o3)

        w4 = Wall.make_wall(0.064, thin_width, height, np.array([0.064 / 2 + 0.016, 0.004 + 0.072, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, 0),
                            ifc_wall_type="LegoWallType1", base_module=base_module_thin, bond_type=bond_type)

        o2 = Opening(w4, np.array([2 * thin_width, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, 0),
                     (4 * thin_width, 1 * thin_width, 6 * 0.0096))
        w4.openings.append(o2)

        w5 = Wall.make_wall(0.032, thin_width, height, np.array([0.088 - 0.004, 0.048 + 0.032 / 2, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                            ifc_wall_type="LegoWallType1", base_module=base_module_thin, bond_type=bond_type)
        w6 = Wall.make_wall(0.024, thin_width, height, np.array([0.024 / 2 + 0.088, 0.004 + 0.048, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, 0),
                            ifc_wall_type="LegoWallType1", base_module=base_module_thin, bond_type=bond_type)
        w7 = Wall.make_wall(0.040, thin_width, height, np.array([0.120 - 0.004, 0.016 + 0.040 / 2, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                            ifc_wall_type="LegoWallType1", base_module=base_module_thin, bond_type=bond_type)
        return [w0, w1, w2, w3, w4, w5, w6, w7]
