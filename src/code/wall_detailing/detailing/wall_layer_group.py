import numpy as np

from detailing.wall import Wall
from detailing.wall_layer import WallLayer
from masonry.brick import BrickInformation


class WallLayerGroup:
    def __init__(self, module: BrickInformation):
        self.module = module
        self.layers = []
        self.rotation = np.quaternion(1, 0, 0, 0)
        self.translation = np.array([0, 0, 0])  # of wall mid

    @classmethod
    def from_wall(cls, wall: Wall, module: BrickInformation):
        ret = cls(module)
        ret.rotation = wall.get_rotation()
        ret.translation = wall.get_translation()
        length, width, height = wall.width, wall.width, wall.height

        layers = int(height / module.height)
        leftover = height % module.height
        z = -height * 0.5
        for layer in range(layers):
            translation = ret.translation.copy()
            translation[2] = z + (layer + 1) * module.height * 0.5  # z position relative to the walls center
            wall_layer = WallLayer(module, length, translation=translation)
            ret.layers.append(wall_layer)

        if leftover > 0.0:
            translation = ret.translation.copy()
            translation[2] = height * 0.5 - leftover * 0.5  # TODO: Check if this is correct
            wall_layer = WallLayer(module, length, translation=translation, height=leftover)
            ret.layers.append(wall_layer)
        return ret
