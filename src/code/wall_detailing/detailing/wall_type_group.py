from typing import List
from detailing.wall_layer_group import WallLayerGroup
from wall_detailing.detailing.wall import WallDetailingInformation
from masonry.bond.abstract_bond import Bond


class WallTypeGroup:
    """
    A group of walls with the same WallDetailingInformation
    """
    def __init__(self, detailing_information: WallDetailingInformation):
        self.detailing_information = detailing_information
        self.module = self.detailing_information.base_module
        self.bond = Bond.BondTypes[self.detailing_information.bond_type](self.module)

        self.layer_groups: List[WallLayerGroup] = []
