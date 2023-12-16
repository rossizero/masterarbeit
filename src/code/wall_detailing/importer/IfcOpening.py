from typing import Tuple

import numpy as np
import quaternion

from wall_detailing.detailing.opening import Opening
from wall_detailing.detailing.wall import Wall


class IfcOpening(Opening):
    def __init__(self, parent: Wall, translation: np.array, rotation: quaternion, dimensions: Tuple[float, float, float]):
        super().__init__(parent, translation, rotation, dimensions)

    def get_position(self, relative: bool = False):
        ret = self.translation.copy()
        #ret -= np.array([self.parent.length / 2, self.parent.width / 2, self.parent.height / 2])
        #ret += np.array([self.length / 2, self.width / 2, self.height / 2])

        #if not relative:
        #    ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
        #    ret += self.parent.get_translation() - np.array([self.parent.length/2, self.parent.width/2, self.parent.height/2])
        return ret