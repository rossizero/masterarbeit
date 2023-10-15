from abc import ABC, abstractmethod
from copy import deepcopy
from typing import List, Tuple

import numpy as np

from detailing.wall_layer import WallLayer
from detailing.wall_layer_group import WallLayerGroup
from masonry.Corner import Corns, Corn
from masonry.bond import Bond


class Solver(ABC):
    def __init__(self, corners: Corns, bond: Bond):
        self.bond = bond
        self.corners = corners

    def tmp(self, layer: WallLayer, look_at_wall: int = -1):
        if look_at_wall != -1:
            if look_at_wall != layer.parent.id:
                return 0

        leftover_left, leftover_right, _ = self.bond.leftover_of_layer(layer.length, layer.get_layer_plan_index(), layer.relative_x_offset())
        c = leftover_left if len(layer.left_connections) > 0 else 0.0
        d = leftover_right if len(layer.right_connections) > 0 else 0.0
        # print("  ", c, d, "sum", c + d, "x_off:", layer.relative_x_offset(), "len:", layer.length, "ind", layer.get_layer_index(), self.bond.layer, num_bricks)
        return c + d

    def score(self, corners: Corns, look_at_wall: int = -1):
        val = 0
        walls = set()
        for corner in corners.corners:
            if look_at_wall != -1:
                for layer in corner.layers:
                    if layer.parent.id == look_at_wall:
                        corner.reduce_layer_length(layer, self.bond)
            else:
                corner.reduce_corner_layer_length(self.bond)
            for l in corner.layers:
                walls.add(l.parent)

        for wall in walls:
            for layer in wall.layers:
                val += self.tmp(layer, look_at_wall=look_at_wall)
        #print("score", val, "on wall", look_at_wall)
        return val

    def all_holes(self, corners: Corns):
        val = 0
        corners = deepcopy(corners)
        for corner in corners.corners:
            for layer in corner.layers:
                val += self.holes_between_corner_and_wall(Corns.from_corner_list([corner]), look_at_wall=layer.parent.id, reduce=False)
        return val

    def holes_between_corner_and_layer(self, corner: Corn, layer: WallLayer):
        corner = deepcopy(corner)

        left = False
        for l in corner.layers:
            if l.parent.id == layer.parent.id:
                corner.reduce_layer_length(l, self.bond)
                leftover_left, leftover_right, num = self.bond.leftover_of_layer(l.length,
                                                                                 l.get_layer_plan_index(),
                                                                                 l.relative_x_offset(),
                                                                                 l.reversed)

            if l.parent.id in [l2.parent.id for l2 in layer.left_connections]:
                left = True

        return leftover_left if left else leftover_right

    def holes_between_corner_and_wall(self, corners: Corns, look_at_wall: int, reduce: bool = True):
        corners = deepcopy(corners)
        current_wall_layers = []

        # reduce all necessary layer lengths
        for corner in corners.corners:
            for layer in corner.layers:
                if layer.parent.id == look_at_wall:
                    if reduce:
                        corner.reduce_layer_length(layer, self.bond)
                    current_wall_layers.append(layer)

        # then count holes between given corner and wall by looking at which side the corner is relative to the wall
        # and only use those leftover_values that actually are between the both
        val = 0
        for corner in corners.corners:
            leftover_left, leftover_right = 0, 0
            current_wall_layer = None
            for layer in corner.layers:
                if layer in current_wall_layers:
                    leftover_left, leftover_right, num = self.bond.leftover_of_layer(layer.length,
                                                                                     layer.get_layer_plan_index(),
                                                                                     layer.relative_x_offset(),
                                                                                     layer.reversed)

                    # print(leftover_left, leftover_right, num, layer.length, layer.relative_x_offset())
                    # if no blocks have been set because the layer length is so small,
                    # there is no actual hole in the wall (will be filled later)
                    if num == 0:  # TODO check if right case also true
                        pass
                        #leftover_left = 0.0
                        #leftover_right = 0.0
                    # print("\t", layer.length, layer.relative_x_offset(), num)
                    current_wall_layer = layer
                    break
            assert current_wall_layer is not None

            left = None
            for layer in corner.layers:
                if layer in current_wall_layer.left_connections:
                    left = True
                    break
                elif layer in current_wall_layer.right_connections:
                    left = False
                    break

            assert left is not None
            c = leftover_left if left else 0.0
            d = leftover_right if not left else 0.0
            val += c + d
        return val

    @abstractmethod
    def solve(self):
        pass