from abc import ABC, abstractmethod
from copy import deepcopy
from typing import List, Tuple

import numpy as np

from wall_detailing.detailing.wall_layer import WallLayer
from wall_detailing.detailing.wall_layer_group import WallLayerGroup
from wall_detailing.masonry.Corner import Corns, Corn
from wall_detailing.masonry.bond import Bond


class Solver(ABC):
    def __init__(self, corners: Corns, bond: Bond):
        self.bond = bond
        self.corners = corners

    def tmp(self, layer: WallLayer, look_at_wall: int = -1):
        if look_at_wall != -1:
            if look_at_wall != layer.parent.id:
                return 0

        leftover_left, leftover_right, _ = self.bond.leftover_of_layer(layer.length, layer.get_layer_index(), layer.relative_x_offset())
        c = leftover_left if len(layer.left_connections) > 0 else 0.0
        d = leftover_right if len(layer.right_connections) > 0 else 0.0
        # print("  ", c, d, "sum", c + d, "x_off:", layer.relative_x_offset(), "len:", layer.length, "ind", layer.get_layer_index(), self.bond.layer, num_bricks)
        return c + d

    def score(self, corners: Corns, look_at_wall: int = -1):
        val = 0
        walls = set()
        for corner in corners.corners:
            corner.reduce_corner_layer_length(self.bond)
            for l in corner.layers:
                walls.add(l.parent)

        for wall in walls:
            for layer in wall.layers:
                val += self.tmp(layer, look_at_wall=look_at_wall)
        print("score", val, "on wall", look_at_wall)
        return val

    @abstractmethod
    def solve(self):
        pass


class GraphSolver(Solver):
    def __init__(self, corners: Corns, bond: Bond):
        super().__init__(corners, bond)

    def get_all_corners_of_wall(self, wall_id: int, left: bool = True):
        wall = self.get_wall(wall_id)

        s = set()
        for layer in wall.layers:
            if left:
                for l in layer.left_connections:
                    s.add(tuple(sorted([l.parent.id, wall_id])))
            else:
                for l in layer.right_connections:
                    s.add(tuple(sorted([l.parent.id, wall_id])))

        groups = self.corners.grouped_by_walls()
        ret = []
        for key in groups.keys():
            if key in s:
                ret.extend(groups[key])
        return ret

    def get_wall(self, wall_id: int):
        for corner in self.corners.corners:
            for layer in corner.layers:
                if layer.parent.id == wall_id:
                    return layer.parent
        return None

    def fit_corner_to_wall(self, corner: Tuple[int, int], wall_id: int):
        wall = self.get_wall(wall_id)

        is_left = False
        for layer in wall.layers:
            for left in layer.left_connections:
                if left.parent.id in corner:
                    is_left = True
                    break

        cs = self.get_all_corners_of_wall(wall_id, is_left)
        corner_offset = 0
        result = 0
        val = len(cs) * 4

        print("fit corner", corner, "to wall", wall_id)

        while corner_offset < self.bond.get_corner_plan_repeat_step():
            corners = deepcopy(cs)
            for corner in corners:
                corner.plan_offset = corner_offset

            score = self.score(Corns.from_corner_list(corners), look_at_wall=wall_id)
            if score < val:
                val = score
                result = corner_offset

            corner_offset += 1
        for corner in cs:
            corner.plan_offset = result
        pass

    def fit_wall_to_corner(self, wall_id: int, corner: Tuple[int, int]):
        wall = self.get_wall(wall_id)
        print("fit wall", wall_id, "to corner", corner)

        is_left = False
        for layer in wall.layers:
            for left in layer.left_connections:
                if left.parent.id in corner:
                    is_left = True
                    break

        cs = self.get_all_corners_of_wall(wall_id, is_left)

        val = len(cs) * 2

        wall_offset = 0
        result = 0
        while wall_offset < self.bond.repeat_layer:
            wall.plan_offset = wall_offset
            corners = deepcopy(cs)
            score = self.score(Corns.from_corner_list(corners), look_at_wall=wall_id)
            if score < val:
                val = score
                result = wall_offset

            wall_offset += 1

        wall.plan_offset = result

    def solve(self):
        if len(self.corners.corners) == 0:
            return True

        class Node:
            def __init__(self, name, val):
                self.name = name
                self.val = val
                self.right = set()
                self.left = set()

        # build graph and look for a start node
        dic = {}
        start = None

        for corner in self.corners.corners:
            for layer in corner.layers:
                if layer.parent.id not in dic.keys():
                    dic[layer.parent.id] = Node(layer.parent.id, layer.parent)

                for obj in layer.right_connections:
                    dic[layer.parent.id].right.add(obj.parent.id)
                for obj in layer.left_connections:
                    dic[layer.parent.id].left.add(obj.parent.id)

        # go through the graph
        visited = []

        def fit(n: Node):
            if n.name not in visited:
                visited.append(n.name)

                #for left in n.left:
                #    if left not in visited:
                #        corn = tuple(sorted([n.name, left]))
                #        self.fit_corner_to_wall(corn, n.name)
                #        self.fit_wall_to_corner(dic[left].name, corn)
                #        fit(dic[left])

                for right in n.right:
                    if right not in visited:
                        corn = tuple(sorted([n.name, right]))
                        self.fit_corner_to_wall(corn, n.name)
                        self.fit_wall_to_corner(dic[right].name, corn)
                        fit(dic[right])

        #corn = tuple(sorted([start.name, list(start.left)[0]]))
        #self.fit_wall_to_corner(start.name, corn)

        for key in dic.keys():
            if key == 0:
                start = dic[key]
                break

        fit(start)
        pass


class BruteForceSolver(Solver):
    """
    A solver that tries all possible combinations of plan offsets and returns the best one
    Note: There might still be holes in the walls due to the orientation of the walls itself
    """

    def __init__(self, corners: Corns, bond: Bond):
        super().__init__(corners, bond)

    def solve(self):
        num_groups = len(self.corners.grouped_by_walls().keys())
        counter = 0
        current_min = len(self.corners.corners) * 2

        def bin_array(num, m):
            """Convert a positive integer num into an m-bit bit vector"""
            return np.array(list(np.binary_repr(num).zfill(m))).astype(np.int8)

        current_best = bin_array(counter, num_groups)

        while counter < 2 ** num_groups:
            corners = deepcopy(self.corners)
            grouped2 = corners.grouped_by_walls()

            arr = bin_array(counter, num_groups)
            counter += 1

            for i, key in enumerate(grouped2.keys()):
                for c in grouped2[key]:
                    c.plan_offset = arr[i]

            val = self.score(corners)
            if val < current_min:
                current_min = val
                current_best = arr.copy()
                if val == 0:
                    break
            print("value", val)

        print("result", current_min, current_best)

        # set the plan_offsets of actual corners to those we found
        corners = deepcopy(self.corners)
        grouped2 = corners.grouped_by_walls()
        grouped1 = self.corners.grouped_by_walls()
        for i, key in enumerate(grouped2.keys()):
            for c in grouped1[key]:
                c.plan_offset = current_best[i]
