from typing import List

from detailing.wall_layer_group import WallLayerGroup
from masonry.bond import StrechedBond
from masonry.brick import BrickInformation


class WallTypeGroup:
    def __init__(self, ifc_class: str, brick_information: List[BrickInformation]):
        self.ifc_class = ifc_class
        self.brick_information = brick_information
        self.brick_information.sort(key=lambda x: x.volume(), reverse=True)
        self.module = self.brick_information[0]
        self.bond = StrechedBond(self.module)
        self.layer_groups: List[WallLayerGroup] = []
