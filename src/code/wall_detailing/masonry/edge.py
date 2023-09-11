import numpy as np
import quaternion


class Edge:
    def __init__(self, translation: np.array = np.array([0, 0, 0]), rotation: np.quaternion = np.quaternion(1, 0, 0, 0), height: float = 0):
        self.translation = translation
        self.rotation = rotation
        self.height = height