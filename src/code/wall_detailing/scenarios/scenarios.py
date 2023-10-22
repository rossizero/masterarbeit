from abc import ABC, abstractmethod

import numpy as np
import quaternion
import math
from detailing.wall import Wall

from masonry.opening import Opening


class Scenario(ABC):
    def __init__(self):
        self.walls = self.get_walls()

    @abstractmethod
    def get_walls(self):
        return []


class FancyCorners(Scenario):
    def get_walls(self):
        an = math.pi / 2 * 1.22432
        an = 0
        w1 = Wall.make_wall(10, 1, 5, np.array([5.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
                       ifc_wall_type="test", name="w1")
        w11 = Wall.make_wall(10, 1, 10, np.array([5.5, 10.0, -1.0]),
                        quaternion.from_euler_angles(0, 0, math.pi / 2 + math.pi), ifc_wall_type="test", name="w11")
        w11_ = Wall.make_wall(5, 1, 10, np.array([5.5, 10.0, 9.0]),
                         quaternion.from_euler_angles(0, 0, math.pi / 2 + math.pi), ifc_wall_type="test", name="w11_")
        w11_2 = Wall.make_wall(5, 1, 10, np.array([5.5, 10.0, -11.0]),
                          quaternion.from_euler_angles(0, 0, math.pi / 2 + math.pi), ifc_wall_type="test", name="w11_2")
        w111 = Wall.make_wall(5, 1, 10, np.array([5.5, -7.5, 1.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
                         ifc_wall_type="test", name="w111")
        w2 = Wall.make_wall(10, 1, 5, np.array([10.0, 4.5, 1.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w2")
        w3 = Wall.make_wall(10, 1, 5, np.array([-5.5, 0.0, -1]), quaternion.from_euler_angles(0.0, 0.0, math.pi / 2),
                       ifc_wall_type="test", name="w3")
        w4 = Wall.make_wall(10, 1, 5, np.array([0.0, -4.5, -2]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w4")
        w4_ = Wall.make_wall(4, 1, 5, np.array([0.0, -4.5, 2]), quaternion.from_euler_angles(0.0, 0.0, 0),
                        ifc_wall_type="test", name="w4_")
        w41_ = Wall.make_wall(4, 1, 5, np.array([1.5, -2.0, 4]), quaternion.from_euler_angles(0.0, 0.0, math.pi / 2),
                         ifc_wall_type="test", name="w41_")

        w1.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w11.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w11_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w11_2.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w111.rotate_around(quaternion.from_euler_angles(0.3, an, an))

        w2.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w4.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w4_.rotate_around(quaternion.from_euler_angles(0.3, an, an))
        w41_.rotate_around(quaternion.from_euler_angles(0.3, an, an))

        all = [w1, w2, w3, w4, w4_, w41_]
        bug = [w3, w4, w4_, w41_]  # solved
        return all


class SimpleCorners(Scenario):
    def get_walls(self):
        an = math.pi / 2 * 1.22432
        an = 0
        height = 0.5 * 7
        w1 = Wall.make_wall(10, 1, height, np.array([5.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2  + math.pi),
                       ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(10, 1, height, np.array([10.0, 4.5, 0.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w2")
        w3 = Wall.make_wall(10, 1, height, np.array([14.5, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, math.pi / 2),
                       ifc_wall_type="test", name="w3")
        w4 = Wall.make_wall(10, 1, height, np.array([10.0, -4.5, 0.0]), quaternion.from_euler_angles(0.0, 0.0, 0),
                       ifc_wall_type="test", name="w4")

        w1.rotate_around(quaternion.from_euler_angles(0, an, an))
        w2.rotate_around(quaternion.from_euler_angles(0, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0, an, an))
        w4.rotate_around(quaternion.from_euler_angles(0, an, an))
        return [w1, w2, w3, w4]


class SimpleCorners2(Scenario):
    def get_walls(self):
        an = 0
        height = 0.5 * 4
        height2 = 0.5 * 7
        w0 = Wall.make_wall(10, 1, height2, np.array([5.5, 0.0, height2 * 0.5]), quaternion.from_euler_angles(0, 0, math.pi / 2 + 0* math.pi),
                       ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(8, 1, height2, np.array([9.0, 5.5, height2 * 0.5]), quaternion.from_euler_angles(0.0, 0.0, 0*math.pi),
                       ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(10, 1, height, np.array([13.5, 1.0, height/2]), quaternion.from_euler_angles(0, 0, math.pi / 2 + 0* math.pi),
                       ifc_wall_type="test", name="w2")
        w3 = Wall.make_wall(9, 1, height, np.array([9.5, -4.5, height/2]), quaternion.from_euler_angles(0.0, 0.0, 0* math.pi),
                       ifc_wall_type="test", name="w3")
        w4 = Wall.make_wall(6, 1, height, np.array([9.5+1.5, -4.5, height / 2]),
                            quaternion.from_euler_angles(0.0, 0.0, 0 * math.pi),
                            ifc_wall_type="test", name="w3")

        w0.rotate_around(quaternion.from_euler_angles(0, an, an))
        w1.rotate_around(quaternion.from_euler_angles(0, an, an))
        w2.rotate_around(quaternion.from_euler_angles(0, an, an))
        w3.rotate_around(quaternion.from_euler_angles(0, an, an))
        return [w0, w1, w2, w3]


class Window1(Scenario):
    def get_walls(self):
        w0 = Wall.make_wall(10, 1, 4 * 0.5, np.array([0.0, 0.0, 2.0]), quaternion.from_euler_angles(0, 0, 1 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(3, 1, 6 * 0.5, np.array([-5 + 1.5, 0.0, 3.0 + 1.5]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi),
                            ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(7, 1, 4 * 0.5, np.array([1.5, 0.0, 2.0 + 3.0]), quaternion.from_euler_angles(0, 0, 1 * math.pi),
                            ifc_wall_type="test", name="w2")
        w3 = Wall.make_wall(3, 1, 2 * 0.5, np.array([3.5, 0.0, 1.0 + 2.5]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi),
                            ifc_wall_type="test", name="w3")

        return [w0, w1, w2, w3]


class DoppelEck1(Scenario):
    def get_walls(self):
        w0 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 0.0, 2.5]), quaternion.from_euler_angles(0, 0, 1 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(3, 1, 5 * 0.5, np.array([-4.5, -2, 1.25]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(7, 1, 5 * 0.5, np.array([-4.5, 4.0, 3.75]), quaternion.from_euler_angles(0, 0, 1 * math.pi + + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        w3 = Wall.make_wall(3, 1, 2 * 0.5, np.array([3.5, 0.0, 7.5]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi),
                            ifc_wall_type="test", name="w3")
        return [w0, w2, w1]


class SimpleOffset(Scenario):
    def get_walls(self):
        w0 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 0.0, 2.5]), quaternion.from_euler_angles(0, 0, 1 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(6, 1, 5 * 0.5, np.array([0.0, 0.0, 6.25]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi),
                            ifc_wall_type="test", name="w1")
        return [w0, w1]


class Bug1(Scenario):
    # TODO
    """
    In this Scenario wall 1 and 2 overlap and will be combined. This leads to the solver graph thinking wall 1 is connected from the left and the right
    (if solver starts with wall 1)
              |
             0|
    ____1_____|__1(2)__
    """
    def get_walls(self):
        w0 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 0.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(5, 1, 5 * 0.5, np.array([-4.5, -2, 1.25]),
                            quaternion.from_euler_angles(0.0, 0.0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(7, 1, 5 * 0.5, np.array([-4.5, 4.0, 3.75]), quaternion.from_euler_angles(0, 0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        bug4 = [w0, w1, w2]
        return bug4


class DoppelEck2_Closed(Scenario):
    """
    |----6-----|--5--|
    |3        0|    4|
    |____1_____|__2__|
    """
    def get_walls(self):
        w0 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 0.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(5, 1, 5 * 0.5, np.array([-4.5, -3, 1.25]),
                            quaternion.from_euler_angles(0.0, 0.0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(7, 1, 5 * 0.5, np.array([-4.5, 4.0, 3.75]), quaternion.from_euler_angles(0, 0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")

        w3 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 7.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w4 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, -6.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w5 = Wall.make_wall(5, 1, 5 * 0.5, np.array([4.5, -3, 3.75]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name = "w2")
        w6 = Wall.make_wall(7, 1, 5 * 0.5, np.array([4.5, 4, 1.25]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        dummy1 = Wall.make_wall(2, 1, 0.5, np.array([1000.0, 0.0, 0.0]),
                            quaternion.from_euler_angles(0, 0, 0),
                            ifc_wall_type="test", name="dummy")
        dummy2 = Wall.make_wall(2, 1, 0.5, np.array([2000.0, 0.0, 0.0]),
                                quaternion.from_euler_angles(0, 0, 0),
                                ifc_wall_type="test", name="dummy")
        bug1 = [w0, w2, w3]  # fixed (tested with every wall as starting wall)
        bug2 = [w0, w1, w2]  # fixed (tested with every wall as starting wall) # TODO
        bug3 = [dummy1, w1, dummy1, dummy2, w4]#, w5]  # (error with 1)
        bug3 = [w4, w1]
        all_ = [w0, w1, w2, w3, w4, w5, w6]
        return all_


class DoppelEck2_Closed_TJoint(Scenario):
    """
    |-------------5--|
    |3        0|    4|
    |____1___________|

    __   _   _   _  _ 2
    """
    def get_walls(self):
        w0 = Wall.make_wall(8, 1, 10 * 0.5, np.array([0.0, -2.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w0.openings.append(Opening(w0, np.array([3.0, 0.0, 0]), quaternion.from_euler_angles(0, 0, 0), (2, 1, 4)))
        w1 = Wall.make_wall(13, 1, 10 * 0.5, np.array([-4.5, 1.0, 2.5]), quaternion.from_euler_angles(0, 0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        w1.openings.append(Opening(w1, np.array([3, 0.0, 1.5]), quaternion.from_euler_angles(0, 0, 0), (4, 1, 2.5)))
        w3 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 7.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w3.openings.append(Opening(w3, np.array([3.0, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, 0), (2, 1, 4)))
        w4 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, -6.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w4.openings.append(Opening(w4, np.array([4, 0.0, 1.5]), quaternion.from_euler_angles(0, 0, 0), (3, 1, 3)))
        w5 = Wall.make_wall(13.0, 1, 10 * 0.5, np.array([4.5, 0, 2.5]),
                            quaternion.from_euler_angles(0.0, 0.0,  math.pi + math.pi / 2),
                            ifc_wall_type="test", name = "w2")
        w5.openings.append(Opening(w5, np.array([3, 0.0, 0.0]), quaternion.from_euler_angles(0, 0, 0), (2, 1, 4)))

        w2 = Wall.make_wall(20, 1, 10 * 0.5, np.array([-10.5, 1.0, 2.5]),
                            quaternion.from_euler_angles(0, 0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        w2.openings.append(Opening(w2, np.array([2, 0.0, 1.0]), quaternion.from_euler_angles(0, 0, 0), (3, 1, 3)))
        w2.openings.append(Opening(w2, np.array([6, 0.0, 1.0]), quaternion.from_euler_angles(0, 0, 0), (3, 1, 3)))
        w2.openings.append(Opening(w2, np.array([10, 0.0, 1.0]), quaternion.from_euler_angles(0, 0, 0), (3, 1, 3)))
        w2.openings.append(Opening(w2, np.array([14, 0.0, 1.0]), quaternion.from_euler_angles(0, 0, 0), (3, 1, 3)))
        w2.openings.append(Opening(w2, np.array([18, 0.0, 1.0]), quaternion.from_euler_angles(0, 0, 0), (1, 1, 3)))

        door = [w4, w5, w1]
        tmp = [w0, w5]
        all_ = [w0, w1, w2, w3, w4, w5]
        return all_


class DoppelEck3_Closed(Scenario):
    def get_walls(self):
        w0 = Wall.make_wall(10, 1, 10 * 0.5, np.array([0.0, 0.0, 2.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(5, 1, 5 * 0.5, np.array([-4.5, -3, 1.25]),
                            quaternion.from_euler_angles(0.0, 0.0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w1")
        w2 = Wall.make_wall(7, 1, 5 * 0.5, np.array([-4.5, 4.0, 3.75]), quaternion.from_euler_angles(0, 0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")

        w3 = Wall.make_wall(10, 1, 14 * 0.5, np.array([0.0, 7.0, 3.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w4 = Wall.make_wall(10, 1, 14 * 0.5, np.array([0.0, -6.0, 3.5]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w5 = Wall.make_wall(5, 1, 5 * 0.5, np.array([4.5, -3, 3.75]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name = "w2")
        w6 = Wall.make_wall(7, 1, 5 * 0.5, np.array([4.5, 4, 1.25]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        w7 = Wall.make_wall(14, 1, 4 * 0.5, np.array([4.5, 0.5, 6.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        w8 = Wall.make_wall(12, 1, 4 * 0.5, np.array([-4.5, 0.5, 6.0]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w2")
        all_ = [w0, w1, w2, w3, w4, w5, w6, w7, w8]  # bug on the top layer
        return all_


class SmallWall(Scenario):
    def get_walls(self):
        width = 1

        # normal small walls
        w0_ = Wall.make_wall(1, width, 5 * 0.5, np.array([-3.0, 0.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test", name="w0")

        w0 = Wall.make_wall(2, width, 5 * 0.5, np.array([0.0, 0.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w1 = Wall.make_wall(3, width, 5 * 0.5, np.array([4.0, 0.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w2 = Wall.make_wall(4, width, 5 * 0.5, np.array([9.0, 0.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w3 = Wall.make_wall(5, width, 5 * 0.5, np.array([15, 0.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        # small corners
        w00 = Wall.make_wall(2, width, 5 * 0.5, np.array([0.0, 5.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w10 = Wall.make_wall(3, width, 5 * 0.5, np.array([4.0, 5.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w20 = Wall.make_wall(4, width, 5 * 0.5, np.array([9.0, 5.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w30 = Wall.make_wall(5, width, 5 * 0.5, np.array([15, 5.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")

        w01 = Wall.make_wall(2, width, 5 * 0.5, np.array([0.5, 6.5, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                             ifc_wall_type="test", name="w0")

        w11 = Wall.make_wall(3, width, 5 * 0.5, np.array([5.0, 7.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                             ifc_wall_type="test", name="w0")

        w21 = Wall.make_wall(4, width, 5 * 0.5, np.array([10.5, 7.5, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                             ifc_wall_type="test", name="w0")

        w31 = Wall.make_wall(5, width, 5 * 0.5, np.array([17, 8, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                            ifc_wall_type="test", name="w0")
        all = [w0_, w0, w1, w2, w3, w00, w10, w20, w30, w01, w11, w21, w31]
        return all


class TJoint1(Scenario):
    def get_walls(self):
        width = 1

        w0 = Wall.make_wall(5, width, 5 * 0.5, np.array([-3.0, 0.0, 1.25]), quaternion.from_euler_angles(0, 0, 0 * math.pi),
                             ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(5, width, 5 * 0.5, np.array([-3.0, 7.0, 1.25]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w2 = Wall.make_wall(10, width, 5 * 0.5, np.array([-3.0, 4.5, 1.25]),
                             quaternion.from_euler_angles(0, 0, 0 * math.pi + math.pi / 2),
                             ifc_wall_type="test", name="w0")
        w3 = Wall.make_wall(5, width, 5 * 0.5, np.array([-30.0, 3.0, 1.25]),
                            quaternion.from_euler_angles(0, 0, 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        return [w0, w1, w2, w3]