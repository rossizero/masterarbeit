import math
import unittest
from copy import deepcopy
from typing import Dict

import numpy as np
import quaternion

from detailing.wall import Wall
from detailing.wall_layer_group import WallLayerGroup
from detailing.wall_type_group import WallTypeGroup
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
        # convert walls to layergroups
        for w in self.walls:
            layer_group = WallLayerGroup.from_wall(w, self.module)
            self.wall_layer_groups.append(layer_group)

    def test(self):
        self.assertEqual(len(self.wall_layer_groups), 2)
        self.assertEqual(len(self.wall_layer_groups[0].layers), self.num_layers)
        self.assertEqual(len(self.wall_layer_groups[1].layers), self.num_layers)

        layer1 = self.wall_layer_groups[0].layers[0]
        layer2 = self.wall_layer_groups[0].layers[1]

        self.assertEqual(layer1.length, self.length)
        self.assertEqual(layer2.length, self.length)
        self.assertTrue(np.array_equal(layer1.translation, np.array([0, 0, 0])))
        self.assertTrue(np.array_equal(layer2.translation, np.array([0, 0, 0.5])))

        self.assertTrue(np.array_equal(layer1.center, np.array([5.5, 0, 0.5])))
        self.assertTrue(np.array_equal(layer2.center, np.array([5.5, 0, 1.0])))

        left_edge = layer1.get_left_edge(True)
        right_edge = layer1.get_right_edge(True)

        l = deepcopy(layer1)
        l.reduce_length(1, True, False)
        self.assertTrue(l.length, self.length - 1)
        self.assertTrue(np.array_equal(left_edge + [1, 0, 0], l.get_left_edge(True)))
        self.assertTrue(np.array_equal(right_edge, l.get_right_edge(True)))
        self.assertEqual(1, l.relative_x_offset())

        l = deepcopy(layer1)
        l.reduce_length(1, False, True)
        self.assertTrue(l.length, self.length - 1)
        self.assertTrue(np.array_equal(right_edge - [1, 0, 0], l.get_right_edge(True)))
        self.assertTrue(np.array_equal(left_edge, l.get_left_edge(True)))
        self.assertEqual(0, l.relative_x_offset())

        l = deepcopy(layer1)
        l.reduce_length(1, True, True)
        self.assertTrue(l.length, self.length - 1)
        self.assertTrue(np.array_equal(right_edge - [0.5, 0, 0], l.get_right_edge(True)))
        self.assertTrue(np.array_equal(left_edge + [0.5, 0, 0], l.get_left_edge(True)))
        self.assertEqual(0.5, l.relative_x_offset())

        l = deepcopy(layer1)
        l.reduce_length(1, False, False)
        self.assertTrue(l.length, self.length - 1)
        self.assertTrue(np.array_equal(right_edge - [0.5, 0, 0], l.get_right_edge(True)))
        self.assertTrue(np.array_equal(left_edge + [0.5, 0, 0], l.get_left_edge(True)))
        self.assertEqual(0.5, l.relative_x_offset())

        left_edge = layer1.get_left_edge(False)
        right_edge = layer1.get_right_edge(False)

        l = deepcopy(layer1)
        l.move_edge(right_edge, 2)
        self.assertEqual(l.length, self.length - 2)

        l = deepcopy(layer1)
        l.move_edge(left_edge, 2)
        self.assertEqual(l.length, self.length - 2)

        l = deepcopy(layer1)
        l.move_edge(right_edge, 2)
        l.move_edge(left_edge, 2)
        self.assertEqual(l.length, self.length - 4)

        l = deepcopy(layer1)
        p = right_edge - [0, 1, 0]  # one meter right of the right edge (aka outside the layer)
        l.move_edge(p, 2)
        self.assertEqual(self.length - 1, l.length)
        n = p + [0.0, 2.0, 0.0]
        self.assertTrue(np.allclose(n, l.right_edge))

        l = deepcopy(layer1)
        p = right_edge + [0, 1, 0]  # one meter left of the right edge (aka inside the layer)
        l.move_edge(p, 2)
        self.assertEqual(self.length - 3, l.length)
        n = p + [0.0, 2.0, 0.0]
        self.assertTrue(np.allclose(n, l.right_edge))

        l = deepcopy(layer1)
        p = left_edge - [0, 1, 0]  # one meter right of the left edge (aka outside the layer)
        l.move_edge(p, 2)
        self.assertEqual(self.length - 3, l.length)
        n = p - [0.0, 2.0, 0.0]
        self.assertTrue(np.allclose(n, l.left_edge))

        l = deepcopy(layer1)
        p = left_edge + [0, 1, 0]  # one meter left of the left edge (aka inside the layer)
        l.move_edge(p, 2)
        self.assertEqual(self.length - 1, l.length)
        n = p - [0.0, 2.0, 0.0]
        self.assertTrue(np.allclose(n, l.left_edge))


if __name__ == '__main__':
    unittest.main()
