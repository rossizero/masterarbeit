from typing import Tuple

import numpy as np
import quaternion


class Opening:
    """
    basically a box that describes a hole in a wall
    """
    def __init__(self, position: np.array, rotation: quaternion, dimensions: Tuple[float, float, float]):
        self.position = position
        self.rotation = rotation
        self.length = max(dimensions[0], dimensions[1])
        self.width = min(dimensions[0], dimensions[1])
        self.height = dimensions[2]
