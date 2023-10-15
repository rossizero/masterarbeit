import math
import unittest
from copy import deepcopy

import numpy as np
import quaternion

from detailing.wall import Wall
from detailing.wall_layer_group import WallLayerGroup
from masonry.bond import StrechedBond
from masonry.brick import BrickInformation


class TestWallLayer(unittest.TestCase):
    def setUp(self):
        self.num_layers = 2
        self.brick_height = 0.5
        self.height = self.brick_height * self.num_layers

        self.brick_information = {"test": [BrickInformation(2, 1, self.brick_height)]}
        self.length = 10
        self.rotated_w0 = True

        # two walls standing on z = 0
        w0 = Wall.make_wall(self.length, 1, self.height, np.array([5.5, 0.0, self.height / 2]),
                            quaternion.from_euler_angles(0, 0, math.pi / 2 + 1 * math.pi),
                            ifc_wall_type="test", name="w0")
        w1 = Wall.make_wall(self.length, 1, self.height, np.array([9.0, 5.5, self.height / 2]),
                            quaternion.from_euler_angles(0.0, 0.0, 0 * math.pi),
                            ifc_wall_type="test", name="w1")

        self.walls = [w0, w1]
        self.wall_layer_groups = []
        self.module = self.brick_information["test"][0]
        self.bond = StrechedBond(self.module)

        # convert walls to layergroups
        for w in self.walls:
            layer_group = WallLayerGroup.from_wall(w, self.module)
            self.wall_layer_groups.append(layer_group)

    def test_right_to_left(self):
        self.assertEqual(len(self.wall_layer_groups), 2)
        self.assertEqual(len(self.wall_layer_groups[0].layers), self.num_layers)
        self.assertEqual(len(self.wall_layer_groups[1].layers), self.num_layers)
        layer1_og = self.wall_layer_groups[0].layers[0]
        layer2 = self.wall_layer_groups[0].layers[1]

        # reduce left
        layer1 = deepcopy(layer1_og)
        self.assertEqual(layer1.length, 10)
        layer1.reduce_length(1, from_left=True, from_right=False)
        self.assertEqual(layer1.length, 9)
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(4, num_bricks)
        self.assertEqual(1, left)
        self.assertEqual(0, right)

        layer1.reversed = True
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(4, num_bricks)
        self.assertEqual(1, right)
        self.assertEqual(0, left)

        # reduce right
        layer1 = deepcopy(layer1_og)
        self.assertEqual(layer1.length, 10)
        layer1.reduce_length(1, from_left=False, from_right=True)
        self.assertEqual(layer1.length, 9)
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(4, num_bricks)
        self.assertEqual(0, left)
        self.assertEqual(1, right)

        layer1.reversed = True
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(4, num_bricks)
        self.assertEqual(1, left)
        self.assertEqual(0, right)

        # reduce right, then left
        layer1 = deepcopy(layer1_og)
        self.assertEqual(layer1.length, 10)
        layer1.reduce_length(1, from_left=False, from_right=True)
        layer1.reduce_length(1, from_left=True, from_right=False)

        self.assertEqual(layer1.length, 8)
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(3, num_bricks)
        self.assertEqual(1, left)
        self.assertEqual(1, right)

        layer1.reversed = True
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(3, num_bricks)
        self.assertEqual(1, left)
        self.assertEqual(1, right)

        layer1.parent.plan_offset = 1
        layer1.reversed = False
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(4, num_bricks)
        self.assertEqual(0, left)
        self.assertEqual(0, right)

        layer1.reversed = True
        left, right, num_bricks = self.bond.leftover_of_layer(layer1.length, layer1.get_layer_plan_index(),
                                                              layer1.relative_x_offset(), reversed=layer1.reversed)
        self.assertEqual(4, num_bricks)
        self.assertEqual(0, left)
        self.assertEqual(0, right)