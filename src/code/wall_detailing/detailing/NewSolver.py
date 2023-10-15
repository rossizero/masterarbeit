from abc import ABC, abstractmethod
from copy import deepcopy
from typing import List, Tuple, Optional

import numpy as np

from detailing.solver import Solver
from detailing.wall_layer import WallLayer
from detailing.wall_layer_group import WallLayerGroup
from masonry.Corner import Corns, Corn
from masonry.bond import Bond


class NewSolver(Solver):
    def __init__(self, corners: Corns, bond: Bond):
        super(NewSolver, self).__init__(corners, bond)

    def get_complete_layer(self, corn: Corn) -> List[Corn]:
        ret = []
        todo = [corn]

        while len(todo) > 0:
            for curr in todo.copy():
                todo.remove(curr)
                if curr not in ret:
                    ret.append(curr)
                    for layer1 in curr.layers:
                        for layer2 in layer1.left_connections:
                            todo.append(self.corners.get_corner(layer1, layer2))
                        for layer2 in layer1.right_connections:
                            todo.append(self.corners.get_corner(layer1, layer2))
        return ret

    def fit_layer_to_corner(self, layer: WallLayer, corner: Corn):
        is_left = False
        for left in layer.left_connections:
            if left in corner.layers:
                is_left = True
                break

        #if not is_left:
        #    layer.reversed = True

        # if the wall the layer lies in has already been touched, we only need to adjust the corners of the layer
        # else we need to find out the best plan_offset for the parent to fit to the corner
        if layer.parent.touched:
            print("set layer of wall", layer.parent.id, "to", corner, corner.plan_offset)
            corner.reduce_layer_length(layer, self.bond)
        else:
            print("fit layer of wall", layer.parent.id, "to", corner)
            wall_offset = 0
            result = 0
            val = 3
            if not is_left:
                pass
                #layer.parent.reversed = True
                #print("reverse!")

            while wall_offset < self.bond.repeat_layer:
                layer.parent.plan_offset = wall_offset
                score2 = self.holes_between_corner_and_layer(corner, layer)
                print("-", score2, wall_offset, layer.get_layer_plan_index())
                if score2 < val:
                    val = score2
                    result = wall_offset

                wall_offset += 1

            print("result", result)

            layer.parent.set_plan_offset(result)
            corner.reduce_layer_length(layer, self.bond)

    def fit_corner_to_layer(self, corner: Corn, layer: WallLayer):
        if not corner.touched:
            print("fit", corner, "to layer of wall", layer.parent.id)

            corner_offset = 0
            result = 0
            val = 3

            while corner_offset < self.bond.get_corner_plan_repeat_step():
                corner.plan_offset = corner_offset
                score2 = self.holes_between_corner_and_layer(corner, layer)

                if set([l.parent.id for l in corner.layers]) == {3, 2}:
                    print("-", score2)
                if score2 < val:
                    val = score2
                    result = corner_offset

                corner_offset += 1
            corner.set_plan_offset(result)
            for l in corner.layers:
                corner.reduce_layer_length(l, self.bond)
        else:
            print("not fitting", corner, "to layer of wall", layer.parent.id)

    def solve_layer(self, complete_layer: List[Corn]):
        start = complete_layer[2]
        for corner in complete_layer:
            bottom = self.corners.get_bottom_corner(corner)
            if bottom is not None:
                corner.set_plan_offset(bottom.plan_offset + 1)
                start = corner
                print("FOUND BOTTOM LAYER!", corner, corner.plan_offset, bottom)
                # break
        print("z: ", list(start.layers)[0].get_center()[2])
        start.touched = True
        todo = [start]
        done = []

        def fit(corner: Corn):
            done.append(corner)

            for layer in corner.layers:
                self.fit_layer_to_corner(layer, corner)

                for left in layer.left_connections:
                    c = self.corners.get_corner(layer, left)
                    if c not in done:
                        self.fit_corner_to_layer(c, layer)
                        todo.append(c)
                for right in layer.right_connections:
                    c = self.corners.get_corner(layer, right)
                    if c not in done:
                        self.fit_corner_to_layer(c, layer)
                        todo.append(c)

        while len(todo) > 0:
            for to in todo.copy():
                todo.remove(to)
                if to not in done:
                    fit(to)

    def solve(self):
        corners = self.corners.corners.copy()
        done = []
        layers = []
        for corn in corners:
            corn.set_main_layer()
            for layer in corn.layers:
                layer.parent.set_x_offsets()
            if corn in done:
                continue
            corners_of_layer = self.get_complete_layer(corn)
            done.extend(corners_of_layer)
            layers.append(corners_of_layer)

        for layer in layers:
            print("\n")
            self.solve_layer(layer)

        print(len(layers))


