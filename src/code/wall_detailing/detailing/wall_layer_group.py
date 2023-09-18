from detailing.wall import Wall
from masonry.brick import BrickInformation


class WallLayerGroup:
    def __init__(self, module: BrickInformation):
        self.module = module
        self.layers = []


    @classmethod
    def from_wall(cls, wall: Wall, module: BrickInformation):
        ret = cls(module)
        return ret
