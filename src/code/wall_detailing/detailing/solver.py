from abc import ABC, abstractmethod
from copy import deepcopy
from typing import List
from detailing.wall_layer import WallLayer
from masonry.corner_rep import Corns, Corn
from masonry.bond import Bond


class Solver(ABC):
    def __init__(self, corners: Corns, bond: Bond):
        self.bond = bond
        self.corners = corners

    def get_complete_layer(self, corn: Corn, corners: Corns = None) -> List[Corn]:
        """
        Looks up corn in corners to find all corners that are connected to it by layers on the same height in the building
        :param corn: a corner
        :param corners: a list of corners
        :return: all corners that are connected to given corner by layers on the same height in the building
        """
        ret = []
        todo = [corn]
        if corners is None:
            corners = self.corners

        while len(todo) > 0:
            for curr in todo.copy():
                todo.remove(curr)
                if curr not in ret:
                    ret.append(curr)
                    for layer1 in curr.layers:
                        for layer2 in layer1.left_connections:
                            todo.append(corners.get_corner([layer1, layer2]))
                        for layer2 in layer1.right_connections:
                            todo.append(corners.get_corner([layer1, layer2]))
        return ret

    def all_holes(self, corners: Corns):
        """
        :param corners: a list of corners
        :return: the number of holes between all corners and their layers
        """
        val = 0
        corners = deepcopy(corners)
        for corner in corners.corners:
            for layer in corner.layers:
                val += self.holes_between_corner_and_layer(corner, layer)
        return val

    def holes_between_corner_and_layer(self, corner: Corn, layer: WallLayer):
        """
        :param corner: a corner
        :param layer: a layer
        :return: the number of holes between the corner and the layer
        """
        corner = deepcopy(corner)

        left = False
        for l in corner.layers:
            if l.parent.id == layer.parent.id:
                corner.reduce_layer_length(l, self.bond)
                leftover_left, leftover_right, num = self.bond.leftover_of_layer(l.length,
                                                                                 l.get_layer_plan_index(),
                                                                                 l.relative_x_offset(),
                                                                                 l.parent.reversed)
                if leftover_right + leftover_left >= l.length:
                    leftover_left = 0
                    leftover_right = 0
            if l.parent.id in [l2.parent.id for l2 in layer.left_connections]:
                left = True

        return leftover_left if left else leftover_right

    @abstractmethod
    def solve(self):
        pass
