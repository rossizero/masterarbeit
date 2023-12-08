import math

import numpy as np
import quaternion

from wall_detailing.detailing.wall import Wall
from wall_detailing.scenarios.scenarios import Scenario


class Scenario1(Scenario):
    def get_walls(self):
        width = 2
        height = 20

        w0 = Wall.make_wall(10, width, height, np.array([5.0, width/2, height/2.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(10, width, height, np.array([5.0, 10.0 - width/2, height / 2.0]),
                             quaternion.from_euler_angles(0, 0, 0),
                             ifc_wall_type="test", name="w0")

        w2 = Wall.make_wall(10, width, height, np.array([width/2, 5.0, height / 2.0]),
                             quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                             ifc_wall_type="test", name="w0")
        w3 = Wall.make_wall(8, width, height, np.array([10 - width / 2, 5.0, height / 2.0]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2.0),
                            ifc_wall_type="test", name="w0")
        return [w0, w1, w2, w3]
