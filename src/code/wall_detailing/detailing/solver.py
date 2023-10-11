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


class GraphSolver(Solver):
    def __init__(self, corners: Corns, bond: Bond):
        super().__init__(corners, bond)

    def get_all_corners_of_wall2(self, wall_id: int):
        groups = self.corners.grouped_by_walls()
        ret = []
        for key in groups.keys():
            if wall_id in key:
                ret.extend(groups[key])
        return ret

    def get_all_connected_corners(self, wall_id: int, left: bool = True):
        """
        returns all corners that are connected to given wall and those that are indirectly connected to it
        (see scenario DoppelEck1: pipapo:  _|-  )
        """
        wall = self.get_wall(wall_id)

        s = set()
        s2 = set()
        for layer in wall.layers:
            if left:
                for l in layer.left_connections:
                    s.add(tuple(sorted([l.parent.id, wall_id])))
                    s2.add(l.parent.id)
            else:
                for l in layer.right_connections:
                    s.add(tuple(sorted([l.parent.id, wall_id])))
                    s2.add(l.parent.id)

        for w in s2.copy():
            w = self.get_wall(w)
            # find out if wall with wall_id is connected to w on the left or right
            left = False
            for layer in w.layers:
                if wall_id in [l.parent.id for l in layer.left_connections]:
                    left = True
                    break

            if left:
                for layer in w.layers:
                    for con in layer.left_connections:
                        s.add(tuple(sorted([con.parent.id, w.id])))
                        s2.add(con.parent.id)
            else:
                for layer in w.layers:
                    for con in layer.right_connections:
                        s.add(tuple(sorted([con.parent.id, w.id])))
                        s2.add(con.parent.id)

        groups = self.corners.grouped_by_walls()
        ret = []
        for key in s:
            if key in groups.keys():
                ret.extend(groups[key])
        return ret

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
        og_corner = corner
        wall = self.get_wall(wall_id)
        print("fit corner", corner, "to wall", wall_id, wall.touched)

        is_left = False
        for layer in wall.layers:
            for left in layer.left_connections:
                if left.parent.id in corner:
                    is_left = True
                    break

        cs = self.get_all_corners_of_wall(wall_id, is_left)
        # only value the corners that are actually between the two walls of og_corner
        curr = []
        for corner in cs:
            add = True
            for layer in corner.layers:
                if layer.parent.id not in og_corner:
                    add = False
                    break
            if add:
                curr.append(corner)

        for corner in curr:
            for layer in corner.layers:
                if layer.parent.id != wall_id:
                    print(corner, layer.parent.id)
                    if is_left and layer.touched_left:
                        print("touched left")
                        return
                    elif not is_left and layer.touched_right:
                        print("touched right")
                        return

        corner_offset = 0
        result = 0
        val = len(cs) * 2

        while corner_offset < self.bond.get_corner_plan_repeat_step():
            corners = deepcopy(curr)
            for corner in corners:
                corner.plan_offset = corner_offset

            score2 = self.holes_between_corner_and_wall(Corns.from_corner_list(corners), wall_id)
            score = self.score(Corns.from_corner_list(corners), look_at_wall=wall_id)

            print("-", score, score2)
            if score2 < val:
                val = score2
                result = corner_offset

            corner_offset += 1

        print("solution: ", result)
        for corner in curr:
            corner.set_plan_offset(result)
            for layer in corner.layers:
                if layer in wall.layers:
                    old_l = layer.length
                    corner.reduce_layer_length(layer, self.bond)
                    #print(old_l, layer.length, layer.parent.id, layer.relative_x_offset())

    def fit_wall_to_corner(self, wall_id: int, corner: Tuple[int, int]):
        og_corner = corner
        wall = self.get_wall(wall_id)
        if wall.touched:
            return
        assert not wall.touched

        is_left = False
        for layer in wall.layers:
            for left in layer.left_connections:
                if left.parent.id in corner:
                    is_left = True
                    break

        cs = self.get_all_corners_of_wall(wall_id, is_left)
        # only value the corners that are actually between the two walls of og_corner
        curr = []
        for corner in cs:
            add = True
            for layer in corner.layers:
                if layer.parent.id not in og_corner:
                    add = False
                    break
            if add:
                for layer in corner.layers:
                    if layer.parent.id == wall_id and not is_left:
                        layer.reversed = True
                curr.append(corner)

        print("fit wall", wall_id, "(", wall.name, ")", "to", corner, len(cs), len(curr), is_left)
        if len(curr) == 0:
            return
        print("offset", curr[0].plan_offset)

        val = len(cs) * 2 + 1

        wall_offset = 0
        result = 0
        while wall_offset < self.bond.repeat_layer:
            wall.plan_offset = wall_offset
            corners = deepcopy(curr)

            score2 = self.holes_between_corner_and_wall(Corns.from_corner_list(corners), wall_id)
            score = self.score(Corns.from_corner_list(corners), look_at_wall=wall_id)
            print("-", score, score2)

            if score2 <= val:
                val = score2
                result = wall_offset

            wall_offset += 1

        wall.set_plan_offset(result)

        print("solution: ", result)
        for corner in curr:
            for layer in corner.layers:
                if layer in wall.layers:
                    old_l = layer.length
                    corner.reduce_layer_length(layer, self.bond)
                    print(old_l, layer.length, layer.relative_x_offset())

    def solve(self):
        if len(self.corners.corners) == 0:
            return True

        class Node:
            def __init__(self, name, val):
                self.name = name
                self.val = val
                self.right = set()
                self.left = set()

            def weight(self) -> int:
                return len(self.right) + len(self.left)

        # build graph and look for a start node
        dic = {}

        for corner in self.corners.corners:
            corner.set_main_layer()

            for layer in corner.layers:
                if layer.parent.id not in dic.keys():
                    layer.parent.set_x_offsets()  # TODO maybe remove?
                    dic[layer.parent.id] = Node(layer.parent.id, layer.parent)

                for obj in layer.right_connections:
                    dic[layer.parent.id].right.add(obj.parent.id)
                    assert len(dic[layer.parent.id].right & dic[layer.parent.id].left) == 0
                for obj in layer.left_connections:
                    dic[layer.parent.id].left.add(obj.parent.id)
                    assert len(dic[layer.parent.id].right & dic[layer.parent.id].left) == 0

                assert len(dic[layer.parent.id].right & dic[layer.parent.id].left) == 0

        # go through the graph
        visited = []
        todo = []

        def fit(n: Node):
            visited.append(n.name)
            print("---handling---", n.name)
            print("   left corns")
            for left in n.left:
                if left not in visited:
                    corn = tuple(sorted([n.name, left]))
                    self.fit_corner_to_wall(corn, n.name)

            print("   right corns")
            for right in n.right:
                if right not in visited:
                    corn = tuple(sorted([n.name, right]))
                    self.fit_corner_to_wall(corn, n.name)

            print("   left walls")
            for left in n.left:
                if left not in visited:
                    corn = tuple(sorted([n.name, left]))
                    self.fit_wall_to_corner(left, corn)
                    todo.append(left)

            print("   right walls")
            for right in n.right:
                if right not in visited:
                    corn = tuple(sorted([n.name, right]))
                    self.fit_wall_to_corner(right, corn)
                    todo.append(right)

        start = [n for n in dic.values()]
        start.sort(key=lambda x: x.weight(), reverse=True)
        start = start[0]
        print("starting with", start.name)
        todo.append(start.name)
        while len(todo) > 0:
            # todo.sort(key=lambda x: dic[x].weight(), reverse=True)
            for to in todo.copy():
                todo.remove(to)
                if to not in visited:
                    fit(dic[to])

        print("score", self.all_holes(self.corners))


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
