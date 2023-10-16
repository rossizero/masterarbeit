import itertools
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
    debug = False

    def __init__(self, corners: Corns, bond: Bond):
        super(NewSolver, self).__init__(corners, bond)

    def fit_layer_to_corner(self, layer: WallLayer, corner: Corn):
        # if the wall the layer lies in has already been touched, we only need to adjust the corners of the layer
        # else we need to find out the best plan_offset for the parent to fit to the corner
        if layer.parent.touched:
            if NewSolver.debug:
                print("set layer of wall", layer.parent.id, "to", corner, corner.plan_offset)
            corner.reduce_layer_length(layer, self.bond)
        else:
            if NewSolver.debug:
                print("fit layer of wall", layer.parent.id, "to", corner)
            wall_offset = 0
            result = 0
            val = 3

            while wall_offset < self.bond.repeat_layer:
                layer.parent.plan_offset = wall_offset
                score = self.holes_between_corner_and_layer(corner, layer)
                if score < val:
                    val = score
                    result = wall_offset
                wall_offset += 1
            layer.parent.set_plan_offset(result)
            corner.reduce_layer_length(layer, self.bond)

    def fit_corner_to_layer(self, corner: Corn, layer: WallLayer):
        if corner.touched:
            if NewSolver.debug:
                print("not fitting", corner, "to layer of wall", layer.parent.id)
        else:
            if NewSolver.debug:
                print("fit", corner, "to layer of wall", layer.parent.id)
            corner_offset = 0
            result = 0
            val = 3

            while corner_offset < self.bond.get_corner_plan_repeat_step():
                corner.plan_offset = corner_offset
                score = self.holes_between_corner_and_layer(corner, layer)
                if score < val:
                    val = score
                    result = corner_offset
                corner_offset += 1
            corner.set_plan_offset(result)
            corner.reduce_corner_layer_length(self.bond)

    def fit(self, corner: Corn, corners: Corns, done: List[Corn]):
        todo = []
        for layer in corner.layers:
            self.fit_layer_to_corner(layer, corner)

            for left in layer.left_connections:
                c = corners.get_corner(layer, left)
                if c not in done:
                    self.fit_corner_to_layer(c, layer)
                    todo.append(c)
            for right in layer.right_connections:
                c = corners.get_corner(layer, right)
                if c not in done:
                    self.fit_corner_to_layer(c, layer)
                    todo.append(c)
        return todo

    def solve_layer(self, complete_layer: List[Corn], corners: Corns, start_index: int = 0) -> bool:
        start = None
        ret = False

        for corner in complete_layer:
            # check if there are any layers below this corner
            bottom = corners.get_bottom_corner(corner)
            if bottom is not None:
                # compare the parent ids of the layers of the corner with the parent ids of the layers of the bottom
                # if they are the same, we can set the plan_offset of the corner to the plan_offset of the bottom + 1
                # else we need to find the correct plan_offset for the corner
                par = [layer.parent.id for layer in bottom.layers]
                all_found = True
                for layer in corner.layers:
                    if layer.parent.id not in par:
                        all_found = False
                        ret = True
                if all_found:
                    corner.set_plan_offset(bottom.plan_offset + 1)
                    start = corner
                else:
                    print(corner, par)

        if start is None:
            print("hhhh")
            start = complete_layer[start_index]
            ret = True

        start.touched = True
        todo = [start]
        done = []

        while len(todo) > 0:
            for to in todo.copy():
                todo.remove(to)
                if to not in done:
                    done.append(to)
                    todos = self.fit(to, corners, done)
                    todo.extend(todos)
        return ret

    def freeze(self, corners: Corns):
        """
        "freezes" the x offsets of all layers of all walls to current values
        "freezes" the main layers for each corner to current main layer
        this needs to be done to prevent errors after reducing the length of the wall_layers
        """
        for corn in corners.corners:
            corn.set_main_layer()
            for layer in corn.layers:
                layer.parent.set_x_offsets()

    def solve(self):
        starters = []

        # first run -> collect all possible starter indices
        corners = deepcopy(self.corners)
        self.freeze(corners)
        done = []
        layers = []

        for corn in corners.corners:
            if corn not in done:
                corners_of_layer = self.get_complete_layer(corn, corners)
                done.extend(corners_of_layer)
                layers.append(corners_of_layer)

        for layer in layers:
            new_corner = self.solve_layer(layer, corners)
            if new_corner:
                starters.append([i for i in range(len(layer))])

        # if score > 0 we continue trying out all possible starter combinations, to find the best one
        min_config = [s[0] for s in starters]
        score = self.all_holes(corners)

        if score > 0:
            print("----------------")
            print("trying", starters, len(list(itertools.product(*starters))), "combinations")
            for config in reversed(list(itertools.product(*starters))):
                corners = deepcopy(self.corners)
                self.freeze(corners)

                index = 0
                done = []
                layers = []
                for corn in corners.corners:
                    if corn not in done:
                        corners_of_layer = self.get_complete_layer(corn, corners)
                        done.extend(corners_of_layer)
                        layers.append(corners_of_layer)

                for layer in layers:
                    new_corner = self.solve_layer(layer, corners, start_index=config[index])
                    if new_corner and index < len(config) - 1:
                        index += 1
                tmp = self.all_holes(corners)
                print(config, tmp)
                if tmp <= score:
                    min_config = config
                    score = tmp
                if score == 0:
                    break

        # apply solution to og corners
        done = []
        layers = []
        index = 0

        corners = self.corners
        self.freeze(corners)
        for corn in corners.corners:
            if corn not in done:
                corners_of_layer = self.get_complete_layer(corn, corners)
                done.extend(corners_of_layer)
                layers.append(corners_of_layer)

        for layer in layers:
            new_corner = self.solve_layer(layer, corners, start_index=min_config[index])
            if new_corner and index < len(min_config) - 1:
                index += 1

        a = 0