import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.scenarios.scenarios import Scenario


class SimpleCorner(Scenario):
    def get_walls(self):
        width = 1
        height = 2

        w00 = Wall.make_wall(2, width, height * 0.5, np.array([0.0, 0.0, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test", name="w0")

        w01 = Wall.make_wall(2, width, height * 0.5, np.array([0.5, 0.5, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                             ifc_wall_type="test", name="w0")

        return [w00, w01]
