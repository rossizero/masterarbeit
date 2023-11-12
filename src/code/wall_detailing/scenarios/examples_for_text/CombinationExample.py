import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.scenarios.scenarios import Scenario


class CombinationExampleForText(Scenario):
    def get_walls(self):
        width = 1
        height = 5
        w1 = Wall.make_wall(9, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w2 = Wall.make_wall(9, width, height * 0.5, np.array([-3.5 + 4, 1.0, height / 4.0 + 1.5]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w1")

        w3 = Wall.make_wall(3, width, 3 * 0.5, np.array([-3.5 + 4 + 6, 1.0, height / 4.0 + 1.5]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w2")

        w4 = Wall.make_wall(3, width, 3 * 0.5, np.array([-3.5 + 4 + 6, 1.0, height / 4.0 - 0.5]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test", name="w3")

        w5 = Wall.make_wall(7, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0 + 4.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w4")

        w6 = Wall.make_wall(7, width, height * 0.5, np.array([-3.5, 1.0, height / 4.0 + 4.0]),
                                 quaternion.from_euler_angles(0, 0, 0 * math.pi),
                                 ifc_wall_type="test", name="w4_same")

        w7 = Wall.make_wall(4, width, 3 * 0.5, np.array([-3.5 + 6.5, 1.0, height / 4.0 + 3.5]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w3")
        return [w1, w2, w3, w4, w5, w6, w7]
