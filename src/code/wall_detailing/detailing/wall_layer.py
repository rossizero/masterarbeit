import numpy as np
import quaternion


class WallLayer:
    """
    A straight layer of a wall
    """
    def __init__(self, parent: 'WallLayerGroup', length: float, translation: np.array = np.array([0.0, 0.0, 0.0]), height: float = None):
        self.parent = parent
        self.translation = translation  # relative to walls translation
        self.height = parent.module.height if height is None else height
        self.length = length

    def combine(self, other: 'WallLayer'):
        total_length = self.length + other.length
        global_mid = (self.center * self.length + other.center * other.length) / total_length
        local_mid = global_mid - self.parent.get_translation()
        local_mid = quaternion.rotate_vectors(self.parent.get_rotation().inverse(), local_mid)
        self.translation = local_mid
        self.length = total_length

    @property
    def left_edge(self, relative: bool = False) -> np.array:
        ret = self.translation.copy()
        ret[0] -= self.length/2.0

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    @property
    def right_edge(self, relative: bool = False) -> np.array:
        ret = self.translation.copy()
        ret[0] += self.length / 2.0

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    @property
    def center(self, relative: bool = False) -> np.array:
        ret = self.translation.copy()

        if not relative:
            ret = quaternion.rotate_vectors(self.parent.get_rotation(), ret)
            ret += self.parent.get_translation()
        return ret

    def is_touching(self, other: 'WallLayer'):
        a = np.allclose(self.right_edge, other.left_edge)
        b = np.allclose(self.right_edge, other.right_edge)
        c = np.allclose(self.left_edge, other.left_edge)
        d = np.allclose(self.left_edge, other.right_edge)
        return a or b or c or d
