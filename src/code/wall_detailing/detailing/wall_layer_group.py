import math
from typing import List

import numpy as np
import quaternion

from detailing.wall import Wall
from detailing.wall_layer import WallLayer
from masonry.brick import BrickInformation


class WallLayerGroup:
    def __init__(self, module: BrickInformation, name: str = None):
        self.module = module
        self.layers: List[WallLayer] = []
        self.rotation = np.quaternion(1, 0, 0, 0)
        self.translation = np.array([0, 0, 0])  # of wall mid
        self.name = name

    def combine(self, other: 'WallLayerGroup'):
        """
        :param other: the wall we try to combine with ourselfs
        :return: whether or not we actually combined anything. If we did, the incoming object is now obsolete
        """
        # first lets check if the situation would allow the combination of the two walls
        # we do this by looking up the angle between the walls and check if their z-direction is parallel
        a1 = self.get_rotation()
        a2 = other.get_rotation()
        diff = a2 * a1.inverse()
        angle = round(diff.angle(), 6)

        z_part1 = quaternion.rotate_vectors(a1, np.array([0.0, 0.0, 1.0]))
        z_part2 = quaternion.rotate_vectors(a2, np.array([0.0, 0.0, 1.0]))
        z_parallel = np.isclose(abs(np.dot(z_part1, z_part2)), 1.0)
        to_combine = other.module == self.module and z_parallel and (np.isclose(angle, math.pi) or np.isclose(angle, 0.0))

        # now lets see if there are any layers actually touching each other and combine the pairs that do
        combined = False
        if to_combine:
            others = other.layers.copy()
            for l1 in self.layers:
                for l2 in others.copy():
                    if l1.is_touching(l2):
                        others.remove(l2)
                        l1.combine(l2)
                        combined = True
            if combined:
                for l in others:
                    local_mid = l.center - self.get_translation()
                    local_mid = quaternion.rotate_vectors(self.get_rotation().inverse(), local_mid)
                    l.translation = local_mid
                    l.parent = self
                    self.layers.append(l)
        return to_combine and combined

    @property
    def min_height(self):
        if len(self.layers) > 0:
            return min([l.height for l in self.layers])
        return None

    @property
    def max_height(self):
        if len(self.layers) > 0:
            return max([l.height for l in self.layers])
        return None

    def get_rotation(self) -> quaternion:
        """
        returns the rotation part of this walls transformation
        """
        return self.rotation.copy()

    def get_translation(self) -> np.array:
        """
        returns the translation part of this walls transformation
        """
        return self.translation.copy()

    def get_lowest_local_x(self):
        if len(self.layers) > 0:
            lefts = min([l.get_left_edge(True)[0] for l in self.layers])
            rights = min([l.get_right_edge(True)[0] for l in self.layers])  # TODO maybe unnecessary
            return min(lefts, rights)
        return None

    def get_sorted_layers(self) -> List[List[WallLayer]]:
        """

        :return: a list containing lists of layers that share the same z height sorted
        """
        self.layers.sort(key=lambda x: x.translation[2])
        ret = []
        curr = []
        last_height = self.layers[0].translation[2]

        for layer in self.layers:
            if last_height == layer.translation[2]:
                curr.append(layer)
                if len(curr) > 1:
                    pass
            else:
                ret.append(curr)
                curr = [layer]
                last_height = layer.translation[2]
        if len(curr) > 0:
            ret.append(curr)
        return ret

    @classmethod
    def from_wall(cls, wall: Wall, module: BrickInformation):
        """
        creates a WallLayerGroup from a wall
        :param wall: the wall we want to be represented layerwise
        :param module: the module we want to apply
        :return: WallLayerGroup Object containing the incoming wall cut into layers
        """
        ret = cls(module, wall.name)
        ret.rotation = wall.get_rotation()
        ret.translation = wall.get_translation()
        length, width, height = wall.length, wall.width, wall.height

        layers = int(height / module.height)
        leftover = height % module.height
        z = -height * 0.5
        for layer in range(layers):
            translation = np.array([0.0, 0.0, 0.0])
            translation[2] = z + (layer + 1) * module.height  # z position relative to the walls center
            wall_layer = WallLayer(ret, length, translation=translation)
            ret.layers.append(wall_layer)

        if leftover > 0.0:
            translation = np.array([0.0, 0.0, 0.0])
            translation[2] = height * 0.5 - leftover * 0.5  # TODO: Check if this is correct
            wall_layer = WallLayer(ret, length, translation=translation, height=leftover)
            ret.layers.append(wall_layer)
        return ret
