import unittest

from detailing.solver import Solver, GraphSolver
from masonry.Corner import Corns, check_for_corners
from masonry.bond import Bond, StrechedBond
from detailing.wall import Wall
from detailing.wall_layer_group import WallLayerGroup
from masonry.brick import BrickInformation
from copy import deepcopy

import numpy as np
import quaternion
import math


class TestSolver(unittest.TestCase):
    def setUp(self) -> None:
        self.num_layers = 2
        self.brick_height = 0.5
        self.height = self.brick_height * self.num_layers

        self.brick_information = {"test": [BrickInformation(2, 1, self.brick_height)]}
        self.length = 10
        self.rotated_w0 = True

        # two walls standing on z = 0
        w0 = Wall.make_wall(self.length, 1, self.height, np.array([5.5, 0.0, self.height / 2]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2 + 0 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(8, 1, self.height, np.array([9.0, 5.5, self.height / 2]),
                            quaternion.from_euler_angles(0.0, 0.0, 1 * math.pi),
                            ifc_wall_type="test", name="w1")

        self.walls = [w0, w1]
        self.wall_layer_groups = []
        self.module = self.brick_information["test"][0]
        # convert walls to layergroups
        for w in self.walls:
            layer_group = WallLayerGroup.from_wall(w, self.module)
            self.wall_layer_groups.append(layer_group)

        self.bond = StrechedBond(self.module)
        self.corns = check_for_corners(self.wall_layer_groups)
        self.solver = GraphSolver(self.corns, self.bond)

    def testSetup(self):
        self.assertEqual(2, len(self.corns.corners))

    def test1(self):
        for i in range(len(self.bond.plan)):
            for wall in self.wall_layer_groups:
                print("wall ", wall.id, "with", i)
                wall.plan_offset = i
                cs1 = self.solver.get_all_corners_of_wall(wall.id, True)
                cs2 = self.solver.get_all_corners_of_wall(wall.id, False)
                for cs in cs2:
                    print("level", cs.get_corner_index())
                print("cs", len(cs1), len(cs2))
                corners = deepcopy(cs1)
                #a = self.solver.holes_between_corner_and_wall(Corns.from_corner_list(corners), wall.id)
                #print(a)
                corners = deepcopy(cs2)
                #a = self.solver.holes_between_corner_and_wall(Corns.from_corner_list(corners), wall.id)
                #print(a)
                #corners = deepcopy(cs)
                #a = self.solver.holes_between_corner_and_wall(Corns.from_corner_list(corners), wall.id)
                #print(a)
                print("")


if __name__ == '__main__':
    unittest.main()