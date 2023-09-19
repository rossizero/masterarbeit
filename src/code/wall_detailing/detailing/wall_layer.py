import numpy as np

from wall_detailing.masonry.brick import BrickInformation


class WallLayer:
    """
    A straight layer of a wall
    """
    def __init__(self, module: BrickInformation, length: float, translation: np.array = np.array([0.0, 0.0, 0.0]), height: float = None):
        self.module = module
        self.translation = translation  # relative to walls translation
        self.height = module.height if height is None else height
        self.length = length

    def combine(self, other: 'WallLayer'):
        ret = WallLayer()
        return ret
